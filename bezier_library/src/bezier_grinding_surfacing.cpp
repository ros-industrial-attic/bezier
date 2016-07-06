#include "bezier_library/bezier_grinding_surfacing.hpp"

BezierGrindingSurfacing::BezierGrindingSurfacing(const std::string input_mesh,
                                                 const double grinding_disk_machining_width,
                                                 const unsigned covering_percentage,
                                                 const double extrication_radius,
                                                 const double lean_angle,
                                                 const AXIS_OF_ROTATION axis_of_rotation,
                                                 const Eigen::Vector3d &slicing_orientation) :
        Bezier(),
        grinding_disk_machining_width_(grinding_disk_machining_width),
        covering_percentage_(covering_percentage),
        extrication_radius_(extrication_radius),
        lean_angle_(lean_angle),
        axis_of_rotation_(axis_of_rotation),
        slicing_orientation_(slicing_orientation)
{
  // FIXME Throw exception here!
  appendInputMesh(input_mesh);
  input_mesh_absolute_path_ = input_mesh;
}

BezierGrindingSurfacing::BezierGrindingSurfacing(const std::string input_mesh) :
        BezierGrindingSurfacing(input_mesh, 0, 0, 0, 0, Y, Eigen::Vector3d::Identity())
{
}

void BezierGrindingSurfacing::setMeshesPublishers(std::shared_ptr<ros::Publisher> &input_mesh_publisher,
                                                  std::shared_ptr<ros::Publisher> &dilated_mesh_publisher)
{
  input_mesh_pub_ = input_mesh_publisher;
  dilated_mesh_pub_ = dilated_mesh_publisher;

  if (input_mesh_pub_)
    displayMesh(input_mesh_pub_, std::string("file://"+input_mesh_absolute_path_), 0.3, 0.2, 0.2);
}

std::string BezierGrindingSurfacing::generateTrajectory(EigenSTL::vector_Affine3d &trajectory,
                                                        const bool display_markers)
{
  visual_tools_->deleteAllMarkers();
  std::string error_string;

  // Make sure all parameters allows to generate a trajectory
  error_string = validateParameters();
  if (!error_string.empty())
    return error_string;

  // Estimate a slicing orientation if it was not provided
  // Otherwise, use the last/provided slicing orientation
  Eigen::Vector3d global_mesh_normal;
  if (slicing_orientation_ == Eigen::Vector3d::Zero())
  {
    if (!estimateSlicingOrientation(input_meshes_[SURFACE_MESH], global_mesh_normal, slicing_orientation_))
      return "Error estimating slicing orientation";
  }

  // Dilate the mesh using the extrication radius
  vtkSmartPointer<vtkPolyData> dilated_mesh = vtkSmartPointer<vtkPolyData>::New();
  dilated_mesh->DeepCopy(input_meshes_[SURFACE_MESH]);
  if (!dilate(dilated_mesh, extrication_radius_))
    return "Error dilating mesh";

  if (display_markers)
  {
    std::string mesh_path(ros::package::getPath("bezier_library") + "/meshes/dilated_mesh.stl");

    if (saveMesh(mesh_path, dilated_mesh))
    {
      if (dilated_mesh_pub_)
        displayMesh(dilated_mesh_pub_, std::string("file://" + mesh_path), 0.1, 0.1, 0.1, 0.5);
    }
    else
      ROS_WARN_STREAM("Could not save dilated mesh, aborting visualization of the dilated mesh!");
  }

  vtkSmartPointer<vtkPolyData> extrication_mesh = vtkSmartPointer<vtkPolyData>::New();
  if (!keepUpperPartofDilatedMesh(input_meshes_[SURFACE_MESH], dilated_mesh, extrication_mesh))
    return "Failed to extract upper part of dilated mesh";

  // Compute normals
  if (!computeNormals(input_meshes_[SURFACE_MESH]))
    return "Error computing normals in surface mesh";
  if (!computeNormals(extrication_mesh))
    return "Error computing normals in extrication mesh";

  if (display_markers)
  {
    std::string mesh_path(ros::package::getPath("bezier_library") + "/meshes/extrication_mesh.stl");

    if (saveMesh(mesh_path, extrication_mesh))
    {
      if (dilated_mesh_pub_)
        displayMesh(dilated_mesh_pub_, std::string("file://" + mesh_path), 0.1, 0.1, 0.1, 0.5);
    }
    else
      ROS_WARN_STREAM("Could not save dilated mesh, aborting visualization of the extrication mesh!");
  }

  // Compute the plane equations to slice the mesh
  EigenSTL::vector_Vector4d grinding_planes_equations;
  Eigen::Vector3d input_mesh_centroid(input_meshes_[SURFACE_MESH]->GetCenter());
  estimateGrindingSlicingPlanes(input_meshes_[SURFACE_MESH], slicing_orientation_, input_mesh_centroid,
                                grinding_planes_equations);

  // Generate all grinding trajectories
  std::vector<vtkSmartPointer<vtkStripper> > grinding_strippers;
  if (!sliceMeshWithPlanes(input_meshes_[SURFACE_MESH], grinding_planes_equations, input_mesh_centroid, grinding_strippers))
    return "Could not slice polydata for grinding trajectories";

  for (vtkSmartPointer<vtkStripper> stripper : grinding_strippers)
    if (stripper->GetOutput()->GetNumberOfLines() > 1)
    {
      ROS_ERROR_STREAM("Grinding stripper has " << stripper->GetOutput()->GetNumberOfLines() << " lines");
      // FIXME Handle this case!
      //return "Grinding tripper has more than 1 line (mesh has a hole). Not implemented yet!";
    }

  std::vector<EigenSTL::vector_Affine3d> grinding_trajectories;
  for (std::vector<vtkSmartPointer<vtkStripper> >::iterator it(grinding_strippers.begin());
      it != grinding_strippers.end(); ++it)
  {
    EigenSTL::vector_Affine3d traj;
    if (!generateRobotPosesAlongStripper(*it, traj))
      return "Could not generate robot poses for grinding trajectory";

    grinding_trajectories.push_back(traj);
  }

  // Application of the grinding lean angle
  for (EigenSTL::vector_Affine3d poses : grinding_trajectories)
  {
    for (Eigen::Affine3d pose : poses)
      applyLeanAngle(pose, axis_of_rotation_, lean_angle_);
  }

  // Generate all extrication trajectories
  EigenSTL::vector_Vector4d extrication_planes_equations;
  EigenSTL::vector_Vector3d extrication_planes_origins;

  for (std::vector<EigenSTL::vector_Affine3d>::iterator it(grinding_trajectories.begin());
      it != grinding_trajectories.end() - 1; ++it)
  {
    // Last point of grinding N line
    Eigen::Vector3d line_n_last_point((*it).back().translation());
    // First point of grinding N+1 line
    Eigen::Vector3d line_n1_first_point((*(it + 1)).front().translation());

    Eigen::Vector4d plane_equation;
    Eigen::Vector3d plane_origin;

    EigenSTL::vector_Affine3d traj;
    estimateExtricationSlicingPlane(line_n_last_point, line_n1_first_point, global_mesh_normal, plane_equation,
                                    plane_origin);

    extrication_planes_equations.push_back(plane_equation);
    extrication_planes_origins.push_back(plane_origin);
  }

  if (extrication_planes_equations.size() != extrication_planes_origins.size())
    return "Error: extrication plane equations/origins size are different!";

  std::vector<vtkSmartPointer<vtkStripper> > extrication_strippers;
  for (unsigned i = 0; i < extrication_planes_equations.size(); ++i)
  {
    vtkSmartPointer<vtkStripper> stripper = vtkSmartPointer<vtkStripper>::New();
    sliceMeshWithPlane(extrication_mesh, extrication_planes_equations[i], extrication_planes_origins[i], stripper);

      if (stripper->GetOutput()->GetNumberOfLines() > 1)
      {
        ROS_ERROR_STREAM("Extrication stripper has " << stripper->GetOutput()->GetNumberOfLines() << " lines");
        // FIXME Handle this case!
        //return "Extrication stripper has more than 1 line (mesh has a hole). Not implemented yet!";
      }

    extrication_strippers.push_back(stripper);
  }

  std::vector<EigenSTL::vector_Affine3d> extrication_trajectories;
  for (std::vector<vtkSmartPointer<vtkStripper> >::iterator it(extrication_strippers.begin());
      it != extrication_strippers.end(); ++it)
  {
    EigenSTL::vector_Affine3d traj;
    if (!generateRobotPosesAlongStripper(*it, traj))
      return "Could not generate robot poses for extrication trajectory";

    extrication_trajectories.push_back(traj);
  }

  unsigned index(0);
  if (display_markers)
  {
    for (EigenSTL::vector_Affine3d traj : grinding_trajectories)
    {
      if (index > 11)
        index = 0;
      displayTrajectory(traj, rviz_visual_tools::GREEN, true);
      //displayTrajectory(traj, visualToolsColorFromIndex(index++), true);
    }

    index = 0;
    // FIXME Display extrication trajectories
    for (EigenSTL::vector_Affine3d traj : extrication_trajectories)
    {
      if (index > 11)
        index = 0;
      //displayTrajectory(traj, rviz_visual_tools::RED, true);
      displayTrajectory(traj, visualToolsColorFromIndex(index++), true);
    }
  }

  trajectory.clear();
  // FIXME Fill the trajectory
  Eigen::Affine3d pose (Eigen::Affine3d::Identity());
  pose.translation() << 1.0, 0.2, 0.5;
  trajectory.push_back(pose);
  return "";
}

void BezierGrindingSurfacing::setSlicingOrientation(const Eigen::Vector3d &cutting_plane_normal)
{
  slicing_orientation_ = cutting_plane_normal;

  if (slicing_orientation_ != Eigen::Vector3d::Zero())
    slicing_orientation_.normalize();
}

// FIXME estimateGrindingSlicingPlanes does not find the exact number of planes
void BezierGrindingSurfacing::estimateGrindingSlicingPlanes(const vtkSmartPointer<vtkPolyData> &polydata,
                                                            const Eigen::Vector3d &slicing_orientation,
                                                            const Eigen::Vector3d &polydata_center,
                                                            EigenSTL::vector_Vector4d &planes_equations)
{
  vtkIdType min_point_index;
  vtkIdType max_point_index;
  double min = std::numeric_limits<double>::max();
  double max = std::numeric_limits<double>::min();
  //cout << "Polydata center : " << endl << polydata_center << endl << endl;
  for (vtkIdType index = 0; index < polydata->GetNumberOfPoints(); index++)
  {
    double p[3];
    polydata->GetPoint(index, p);
    Eigen::Vector3d point(p);
    Eigen::Vector3d vector_to_project(point - polydata_center);
    vector_to_project.normalize();
    double res = slicing_orientation.normalized().dot(vector_to_project);
    //cout << res << endl;
    if (res > max)
    {
      max = res;
      max_point_index = index;
    }
    else if (res < min)
    {
      min = res;
      min_point_index = index;
    }
  }

  //cout << "min : " << min << " max : " << max << endl;
  //cout << "min index : " << min_point_index << " max index : " << max_point_index << endl;

  double max_point_coord[3], min_point_coord[3];
  polydata->GetPoint(max_point_index, max_point_coord);
  polydata->GetPoint(min_point_index, min_point_coord);
  Eigen::Vector3d max_point(max_point_coord);
  Eigen::Vector3d min_point(min_point_coord);
  Eigen::Vector3d distance(max_point - min_point);
  //cout << "min point coord: " << endl << min_point << endl << endl;
  //cout << "max point coord: " << endl << max_point << endl << endl;
  //cout << "distance : " << distance.norm() << endl;
  double width = grinding_disk_machining_width_ * (1 - (double)covering_percentage_ / 100);
  double line_count = std::floor((distance.norm()) / width);
  //cout << "Number of lines: " << (distance.norm()) / width << endl;
  //cout << "Number of lines = " << line_count << endl;

  std::vector<double> offsets;
  offsets.push_back(0); // Add the plane at offset 0

  for (unsigned i = 0; i < (line_count / 2) + 1; ++i) // FIXME: Remove "+1" and find a fix the whole function!
  {
    double offset_upper_origin = width * (i + 1);
    offsets.push_back(offset_upper_origin);
    double offset_lower_origin = -width * (i + 1);
    offsets.push_back(offset_lower_origin);
  }

  // Sort planes in ascending order
  std::sort(offsets.begin(), offsets.end());

  // Add all planes/offsets in the vector
  planes_equations.clear();
  for (unsigned j = 0; j < offsets.size(); ++j)
  {
    Eigen::Vector4d plane;
    plane << slicing_orientation.normalized(), offsets[j];

    planes_equations.push_back(plane);
  }
}

void BezierGrindingSurfacing::estimateExtricationSlicingPlane(const Eigen::Vector3d &line_n_last_point,
                                                              const Eigen::Vector3d &line_n1_first_point,
                                                              const Eigen::Vector3d &global_mesh_normal,
                                                              Eigen::Vector4d &plane_equation_normal,
                                                              Eigen::Vector3d &plane_origin)
{
  // line_n_last_point = Point A
  const Eigen::Vector3d point_A = line_n_last_point;
  // line_n1_first_point = Point B
  const Eigen::Vector3d point_B = line_n1_first_point;

  // Compute a point which is the same point of end point of current line with an offset (in the direction of global normal)
  Eigen::Vector3d point_C(point_A + global_mesh_normal);

  // Compute vector between end point of current line and first point of next line
  Eigen::Vector3d vector_AB;
  vector_AB = point_B - point_A;

  // Compute vector between end point of current line and end_point offset
  Eigen::Vector3d vector_AC;
  vector_AC = point_C - point_A;

  // The normal of the plan defined by end_point_current_line, point_C and first_point_next_line
  // is the scalar product between the two vectors
  Eigen::Vector3d plane_normal(vector_AB.cross(vector_AC));

  plane_equation_normal << plane_normal, 0.0; // 0 offset because plane is centered

  // Plane origin is the middle between end_point_current_line and first_point_next_line
  plane_origin = (point_A + point_B) / 2;
}

void BezierGrindingSurfacing::applyLeanAngle(Eigen::Affine3d &pose,
                                             const AXIS_OF_ROTATION lean_angle_axis,
                                             const double angle_value)
{
  if (lean_angle_axis == X)
    pose.rotate(Eigen::AngleAxisd(angle_value, Eigen::Vector3d::UnitX()));
  else if (lean_angle_axis == Y)
    pose.rotate(Eigen::AngleAxisd(angle_value, Eigen::Vector3d::UnitY()));
  else if (lean_angle_axis == Z)
    pose.rotate(Eigen::AngleAxisd(angle_value, Eigen::Vector3d::UnitZ()));
}

bool BezierGrindingSurfacing::generateRobotPosesAlongStripper(const vtkSmartPointer<vtkStripper> &line,
                                                              EigenSTL::vector_Affine3d &trajectory)
{
  if (line->GetOutput()->GetNumberOfLines() == 0)
    return false;

  // This map is used to store each point of the line and the normal matching
  BezierPointNormalTable point_normal_table;

  // Get the points from the stripper
  vtkSmartPointer<vtkPoints> points = line->GetOutput()->GetPoints();

  if (!points)
    return false;

  // Get the cells from the stripper
  vtkSmartPointer<vtkCellArray> cells = line->GetOutput()->GetLines();

  if (!cells)
    return false;

  // Get the normals from stripper
  vtkSmartPointer<vtkFloatArray> point_normal_array = vtkFloatArray::SafeDownCast(
      line->GetOutput()->GetPointData()->GetNormals());

  if (!point_normal_array)
    return false;

  // We associate each point of the line to its normal
  cells->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
  cells->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
  vtk_observer_->Clear();
  cells->InitTraversal();

  if (vtk_observer_->GetWarning())
  {
    ROS_WARN_STREAM("BezierGrindingSurfacing::generateRobotPosesAlongStripper: " << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }
  if (vtk_observer_->GetError())
  {
    ROS_ERROR_STREAM(
        "BezierGrindingSurfacing::generateRobotPosesAlongStripper: " << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }

  vtkIdType* indices; // FIXME Use a vtkSmartPointer
  vtkIdType number_of_points;
  vtk_observer_->Clear();
  while (cells->GetNextCell(number_of_points, indices))
  {
    if (vtk_observer_->GetWarning())
    {
      ROS_WARN_STREAM(
          "BezierGrindingSurfacing::generateRobotPosesAlongStripper: " << vtk_observer_->GetWarningMessage());
      vtk_observer_->Clear();
      return false;
    }
    if (vtk_observer_->GetError())
    {
      ROS_ERROR_STREAM(
          "BezierGrindingSurfacing::generateRobotPosesAlongStripper: " << vtk_observer_->GetWarningMessage());
      vtk_observer_->Clear();
      return false;
    }

    for (vtkIdType i = 0; i < number_of_points; i++)
    {
      // Get the point N
      double point_n[3];
      points->GetPoint(indices[i], point_n);
      Eigen::Vector3d point_n_vector(point_n);

      // Get Z normal
      double normal[3];
      point_normal_array->GetTuple(indices[i], normal);
      Eigen::Vector3d normal_n_vector(normal);
      normal_n_vector *= -1; // Inverse the orientation of the normal vector

      // Save the pair point/vector into the point_normal_table
      PointNormal pn(point_n_vector, normal_n_vector);
      point_normal_table.push_back(pn);
    }
  }

  if(!filterNeighborPosesTooClose(point_normal_table, 1e-2))
  {
    ROS_ERROR_STREAM("BezierGrindingSurfacing::generateRobotPosesAlongStripper: cannot filter grinding trajectory");
    return false;
  }

  if (point_normal_table.empty())
  {
    ROS_ERROR_STREAM("BezierGrindingSurfacing::generateRobotPosesAlongStripper: Empty point/normal table");
    return false;
  }

  // At this stage all the points of the line and the normal matching for each point
  // are stored into the container point_normal_map.
  // We now iterate through this map and generate robot pose for each point
  Eigen::Affine3d last_pose(Eigen::Affine3d::Identity());
  // Iteration Loop through the table

  for (BezierPointNormalTable::iterator it(point_normal_table.begin()); it != point_normal_table.end(); ++it)
  {
    if (it == point_normal_table.end() - 1)
    {
      // The current pose is the last one of the line
      // We keep the same orientation than the previous pose,
      // and we make a translation to the last point
      Eigen::Affine3d pose(Eigen::Affine3d::Identity());
      pose.translation() << (*it).first;
      pose.linear() = last_pose.linear();
      trajectory.push_back(pose);
      break;
    }

    // Iterator pointing on the next point of the line
    BezierPointNormalTable::iterator it_next = it + 1;

    // Normals
    Eigen::Vector3d normal_x(Eigen::Vector3d::Zero());
    Eigen::Vector3d normal_y(Eigen::Vector3d::Zero());
    // We get the normal Z computed and stored in the table
    Eigen::Vector3d normal_z((*it).second);

    // Points
    Eigen::Vector3d point((*it).first); // Current Point
    Eigen::Vector3d next_point((*it_next).first); // Next point

    // Robot Pose
    Eigen::Affine3d pose(Eigen::Affine3d::Identity());

    // Compute the normal X. It matches the next point direction
    normal_x = next_point - point;

    if (normal_x == Eigen::Vector3d::Zero())
    {
      // Something goes wrong during the computation of the X normal of this point
      // We skip the generation for this pose
      ROS_ERROR_STREAM("BezierGrindingSurfacing::generateRobotPosesAlongStripper: Normal X is zero!");
      //return false;
    }

    // The normal Y is generated through the cross product between Z and X
    normal_y = normal_z.cross(normal_x);

    // Translation Part
    pose.translation() << point;
    // Rotation Part
    pose.linear().col(0) << normal_x.normalized();
    pose.linear().col(1) << normal_y.normalized();
    pose.linear().col(2) << normal_z.normalized();

    if (pose.matrix().hasNaN())
    {
      ROS_ERROR_STREAM("BezierGrindingSurfacing::generateRobotPosesAlongStripper: Generated pose has NaN!");
      //return false;
    }

    // Keep the last pose in memory.
    // It is used to generate the orientation of the last pose of the line
    last_pose = pose;
    //ROS_ERROR_STREAM("Pose " << endl << pose.matrix());
    trajectory.push_back(pose);
  }

  return true;
}

bool BezierGrindingSurfacing::filterNeighborPosesTooClose(BezierPointNormalTable &trajectory,
                                                          const double minimal_distance)
{

  if (trajectory.size() <= 1)
    return false;

  // Vector that contain the indices for all points to be removed from the trajectory vector
  std::vector<unsigned> indices_to_be_removed;
  // The offset variable allows to count how much point will be skipped before reaching a point
  // located far enough from the current point. Every point located between the index of the
  // current point and the next point located at index of current point + offset is deleted.
  // The offset is set to 1 at initialization, so the next point will be immediately the one
  // following the current into the trajectory vector
  int offset = 1;
  // Main loop through the trajectory vector
  for (unsigned index = 0; index < trajectory.size() - 1; index++)
  {
    // We apply the filter on all the points except for the last point of the trajectory
    if (index < trajectory.size() - 1)
    {
      // Current point from which the distance is computed
      Eigen::Vector3d point(trajectory[index].first);
      // Next point from which the distance is computed
      Eigen::Vector3d next_point(trajectory[index + 1].first);
      double point_values[3] = {point[0], point[1], point[2]};
      double next_point_values[3] = {next_point[0], next_point[1], next_point[2]};
      // Distance computing between the current point and the designated next point
      double distance = sqrt(vtkMath::Distance2BetweenPoints(point_values, next_point_values));
      if (distance < minimal_distance)
      {
        // The distance is less than minimal distant constraint of the filter
        // We loop over the trajectory vector until we reached a point located far enough from the current point
        // or we reached the last point of the trajectory
        while (distance <= minimal_distance && (index + offset) < trajectory.size())
        {
          // At this stage, the next point found is not far enough, so it must be removed from
          // the trajectory vector. In order to do that, his index is saved into the indices vector
          indices_to_be_removed.push_back(index + offset);
          // Increment the offset : This offset represent the distance inside the trajectory vector
          // from the current point and the next point with which the distance will be computed
          offset++;
          // the next point index is the current point index plus the offset
          next_point = trajectory[index + offset].first;
          next_point_values[0] = next_point[0];
          next_point_values[1] = next_point[1];
          next_point_values[2] = next_point[2];
          // The new distance between the current point and the next point is computed
          distance = sqrt(vtkMath::Distance2BetweenPoints(point_values, next_point_values));
          // if the distance doesn't match our criteria, we loop back and take as next point, a
          // point located at the position index of the next point + 1
        }

        // The loop has reached the last point of the trajectory, the filter is not applied!
        if ((index + offset) >= trajectory.size())
          break;

        // The filter loop has reached a point located far enough. So we jump to that point
        // in order to compute the distance between this point and his next neighbor
        index += offset - 1;
        // The offset is reset
        offset = 1;
      }
      else
      {
        // The next point is located far enough, we do not apply the filter
        // The offset is kept at the value 1
        offset = 1;
      }
    }
  }

  // Temporary buffer used to store non filtered point
  BezierPointNormalTable tmp_trajectory;

  unsigned position(0);
  for (PointNormal value : trajectory)
  {
    if (!(std::find(indices_to_be_removed.begin(), indices_to_be_removed.end(), position) != indices_to_be_removed.end()))
    {
      // The current index is not contained in the vector containing index to be removed
      // so we can add it to the buffer storing non filtered points
      tmp_trajectory.push_back(value);
    }
    position++;
  }

  // At this stage, tmp_trajectory contains all the non filtered points
  // so we clear the trajectory vector and we transfer all the points
  trajectory.clear();
  trajectory = tmp_trajectory;

  if (trajectory.size() <= 1)
    return false;

  return true;
}

bool BezierGrindingSurfacing::filterExtricationTrajectory(const vtkSmartPointer<vtkPolyData> polydata,
                                                          const Eigen::Vector3d first_point,
                                                          const Eigen::Vector3d first_point_normal,
                                                          const Eigen::Vector3d last_point,
                                                          const Eigen::Vector3d last_point_normal,
                                                          const double extrication_radius,
                                                          EigenSTL::vector_Affine3d &trajectory)
{
  //FIXME Does not compile
  /*
  // Extrication Orientation Manager (EOM)
  // For the N first point and N last point, we respectively keep the same orientation
  // than the first point and the last point. In order to do that, we compute the scalar product
  // between the normal of the first grinding point A for the first N points of the extrication path, and the normal of the last
  // grinding point B and the last N points of the extrication path.
  // N will be determined by the scalar product result. As long as the result is not about the dilated distance,
  // we keep the orientation of the A or B.
  // The extrication lines is divided into 3 part :
  // - the first one contain all the poses having the same orientation than the start pose of the grinding line.
  // - the intermediate part contain all poses having orientation generated depending on the normals of the mesh
  // - the last one contain all the poses having the same orientation than the end pose of the grinding line.
  // All the poses and their orientation are computed by the following 3 loops (respectively for the first part, the intermediate
  // and the last one)

  if (trajectory.empty())
    return false;

  if (extrication_radius <= 0)
    return false;

  // Used to determine poses orientation in extrication path generation, see below (EOM)
  const double upper_bound(extrication_radius * 10 / 100 + extrication_radius);
  const double lower_bound(extrication_radius - extrication_radius * 5 / 100);

  // Get the first point of the grinding line
  Eigen::Vector3d line_first_point(first_point);
  // Get the normal of the first point of the grinding line
  Eigen::Vector3d line_first_point_normal(first_point_normal);
  // Get the last point of the grinding line
  Eigen::Vector3d line_last_point(last_point);
  // Get the normal of the last point of the grinding line
  Eigen::Vector3d line_last_point_normal(last_point_normal);

  // Index allowing to store the start of the intermediate part (see EOM explanation below)
  EigenSTL::vector_Affine3d::iterator start_of_intermediate_part(trajectory.begin());

  for (EigenSTL::vector_Affine3d::iterator it(trajectory.begin()); it != trajectory.end(); it++)
  {
    // vect represent the vector between the first point of the grinding line and the current point of the extrication path
    Eigen::Vector3d vect( (*it).translation() - line_first_point );
    double scalar_res = line_first_point_normal.dot(vect);
    if (scalar_res < upper_bound && scalar_res > lower_bound)
    {
      // the current point is near the point N, we start changing the orientation
      // The orientation change is implicitly changed in the generateRobotPoses function
      // so we just don't override it.
      start_of_intermediate_part = it;
      break; // Break the loop process
    }
  }

  // Index allowing to store the end of the intermediate part (see EOM explanation below)
  EigenSTL::vector_Affine3d::iterator end_of_intermediate_part(trajectory.end() - 1);

  // FIXME Check if the reverse iterator works!
  for (EigenSTL::vector_Affine3d::reverse_iterator it(trajectory.begin()); it != trajectory.begin(); it++)
  {
    // Vect represent the vector between the last point of the grinding line and the current point of the extrication path
    Eigen::Vector3d Vect((*it).translation() - line_last_point);
    double scalar_res = line_last_point_normal.dot(Vect);
    if (scalar_res < upper_bound && scalar_res > lower_bound)
    {
      // the current point is near the point N, we start changing the orientation
      // The orientation change is implicitly changed in the generateRobotPoses function
      // so we just don't override it.
      end_of_intermediate_part = it;
      break; // Break the loop process
    }
  }

  if (start_of_intermediate_part > end_of_intermediate_part)
  {
    ROS_ERROR_STREAM("BezierGrindingSurfacing::filterExtricationTrajectory: Bad start/end indices!");
    return false;
  }

  EigenSTL::vector_Affine3d filtered_trajectory;
  // Gather all the poses respecting the EOM condition into a vector of pose
  while (start_of_intermediate_part <= end_of_intermediate_part)
  {
    Eigen::Affine3d filtered_pose = *start_of_intermediate_part;
    filtered_trajectory.push_back(filtered_pose);
    start_of_intermediate_part++;
  }
  // Clear the current trajectory vector
  trajectory.clear();
  trajectory.resize(filtered_trajectory.size());
  // Insert all the filtered points into the trajectory vector
  trajectory.insert(trajectory.begin(), filtered_trajectory.begin(), filtered_trajectory.end());
  */
  return true;
}

std::string BezierGrindingSurfacing::validateParameters()
{
  if (grinding_disk_machining_width_ <= 0.0)
    return "Grinding disk diameter must be > 0";

  if (covering_percentage_ > 99)
    return "Covering percentage must be lower than 100%";
  return "";
}
