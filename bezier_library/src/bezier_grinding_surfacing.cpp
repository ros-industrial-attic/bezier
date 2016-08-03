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
  if(!appendInputMesh(input_mesh))
  {
    AppendBezierException load_mesh_exception("BezierLibrary::appendInputMesh: Cannot load input mesh");
    throw load_mesh_exception;
  }
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
                                                        std::vector<bool> &is_grinding_pose,
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
      ROS_WARN_STREAM(
          "BezierGrindingSurfacing::generateTrajectory: Could not save the mesh, aborting visualization of the dilated mesh!");
  }

  vtkSmartPointer<vtkPolyData> extrication_mesh = vtkSmartPointer<vtkPolyData>::New();
  if (!keepUpperPartofDilatedMesh(input_meshes_[SURFACE_MESH], dilated_mesh, extrication_mesh))
    return "Failed to extract upper part of dilated mesh";

  if(!removeIsolatedTrianglesFilter(extrication_mesh, 20))
    return "Failed to filter isolated triangles in extrication mesh";

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
      ROS_WARN_STREAM(
          "BezierGrindingSurfacing::generateTrajectory: Could not save the mesh, aborting visualization of the extrication mesh!");
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
  {
    if (stripper->GetOutput()->GetNumberOfLines() > 1)
    {
      ROS_ERROR_STREAM(
          "BezierGrindingSurfacing::generateTrajectory: Grinding stripper has " << stripper->GetOutput()->GetNumberOfLines() << " lines");
      // FIXME Handle this case!
      //return "Grinding stripper has more than 1 line (mesh has a hole). Not implemented yet!";
    }
  }

  // Compute the vector used to harmonize all the trajectories directions and display it
  Eigen::Vector3d direction_reference(slicing_orientation_.cross(global_mesh_normal));
  Eigen::Affine3d pose_dir_reference(Eigen::Affine3d::Identity());
  double centroid[3];
  centroid[0] = input_meshes_[SURFACE_MESH]->GetCenter()[0];
  centroid[1] = input_meshes_[SURFACE_MESH]->GetCenter()[1];
  centroid[2] = input_meshes_[SURFACE_MESH]->GetCenter()[2];
  pose_dir_reference.translation() << centroid[0], centroid[1], centroid[2];
  pose_dir_reference.affine().col(0).head<3>() << direction_reference;
  pose_dir_reference.affine().col(2).head<3>() << direction_reference[2], 0, -direction_reference[0];
  pose_dir_reference.affine().col(1).head<3>() << pose_dir_reference.affine().col(2).head<3>().cross(pose_dir_reference.affine().col(0).head<3>());
  visual_tools_->publishArrow(pose_dir_reference, rviz_visual_tools::GREEN, rviz_visual_tools::XXSMALL);
  pose_dir_reference.translation() += 0.07 * pose_dir_reference.affine().col(0).head<3>();
  visual_tools_->publishText(pose_dir_reference, "Direction reference", rviz_visual_tools::GREEN, rviz_visual_tools::SMALL, false);

  std::vector<EigenSTL::vector_Affine3d> grinding_trajectories;
  for (std::vector<vtkSmartPointer<vtkStripper> >::iterator it(grinding_strippers.begin());
      it != grinding_strippers.end(); ++it)
  {
    EigenSTL::vector_Affine3d traj;
    if (!generateRobotPosesAlongStripper(*it, traj))
    {
      ROS_WARN_STREAM(
          "BezierGrindingSurfacing::generateTrajectory: Could not generate robot poses for grinding trajectory");
      continue;
    }

    if (harmonizeLineOrientation(traj, direction_reference))
      ROS_INFO_STREAM("BezierGrindingSurfacing::generateTrajectory: Grinding line reversed");

    grinding_trajectories.push_back(traj);
  }

  // Generate all extrication trajectories
  EigenSTL::vector_Vector4d extrication_planes_equations;
  EigenSTL::vector_Vector3d extrication_planes_origins;
  // Compute extrication direction vector orientation
  EigenSTL::vector_Vector3d extrication_direction_vector;

  for (std::vector<EigenSTL::vector_Affine3d>::iterator it(grinding_trajectories.begin());
      it != grinding_trajectories.end() - 1; ++it)
  {
    // Last point of grinding N line
    Eigen::Vector3d line_n_last_point((*it).back().translation());
    // First point of grinding N+1 line
    Eigen::Vector3d line_n1_first_point((*(it + 1)).front().translation());

    extrication_direction_vector.push_back(line_n1_first_point - line_n_last_point);

    Eigen::Vector4d plane_equation;
    Eigen::Vector3d plane_origin;

    EigenSTL::vector_Affine3d traj;
    estimateExtricationSlicingPlane(line_n_last_point, line_n1_first_point, global_mesh_normal, plane_equation,
                                    plane_origin);

    extrication_planes_equations.push_back(plane_equation);
    extrication_planes_origins.push_back(plane_origin);
  }

  if (extrication_planes_equations.size() != extrication_planes_origins.size() ||
      extrication_planes_equations.size() != extrication_direction_vector.size())
    return "Error: extrication plane equations/origins size are different!";

  std::vector<vtkSmartPointer<vtkStripper> > extrication_strippers;
  for (unsigned i = 0; i < extrication_planes_equations.size(); ++i)
  {
    vtkSmartPointer<vtkStripper> stripper = vtkSmartPointer<vtkStripper>::New();
    sliceMeshWithPlane(extrication_mesh, extrication_planes_equations[i], extrication_planes_origins[i], stripper);

      if (stripper->GetOutput()->GetNumberOfLines() > 1)
      {
      ROS_ERROR_STREAM(
          "BezierGrindingSurfacing::generateTrajectory: Extrication stripper has " << stripper->GetOutput()->GetNumberOfLines() << " lines");
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

    // We generated the pose from N to N+1 but when extricating the grinder
    // the orientation should stay the same as the one when grinding.
    // So we revert the X axis to travel backwards:
    invertXAxisOfPoses(traj);

    extrication_trajectories.push_back(traj);
  }

  // Iterator allowing to move through the grinding trajectories
  std::vector<EigenSTL::vector_Affine3d>::iterator grinding_iterator(grinding_trajectories.begin());

  for (unsigned i = 0; i < extrication_trajectories.size(); ++i)
  {
    if (harmonizeLineOrientation(extrication_trajectories[i], -direction_reference))
      ROS_INFO_STREAM("BezierGrindingSurfacing::generateTrajectory: Extrication line reversed");

    // Last point and normal of the grinding line N
    Eigen::Vector3d first_point((*grinding_iterator).back().translation());
    Eigen::Vector3d first_point_normal((*grinding_iterator).back().linear().col(2));
    first_point_normal *= -1; // FIXME: 2 options:
    // Move ApplyLeanAngle AFTER this loop so that the grinding traj normal == polydata normal
    // Use VTK polydata normal instead of the grinding normal (it has been modified by ApplyLeanAngle)

    // Move the iterator to the next grinding line
    grinding_iterator++;

    // Last point and normal of the grinding line N + 1
    Eigen::Vector3d last_point((*grinding_iterator).front().translation());
    Eigen::Vector3d last_point_normal((*grinding_iterator).front().linear().col(2));
    last_point_normal *= -1; // FIXME: See line 212 (first_point_normal *= -1;)

    // Application of the extrication filter on the current extrication line
    if (!filterExtricationTrajectory(first_point, first_point_normal, last_point, last_point_normal,
                                     extrication_planes_equations[i], extrication_trajectories[i]))
    {
      return "Could not filter extrication trajectory";
    }
  }

  // Generate last extrication trajectory from last grinding point to first grinding point
  if (grinding_trajectories.empty())
    return "Grinding trajectories are empty, could not generate last extrication trajectory";

  if (grinding_trajectories.front().empty() || grinding_trajectories.back().empty())
    return "First or last grinding trajectory is empty, could not generate last extrication trajectory";

  Eigen::Vector3d grinding_first_point (grinding_trajectories.front().front().translation());
  Eigen::Vector3d grinding_last_point (grinding_trajectories.back().back().translation());

  Eigen::Vector4d last_extrication_plane_eq;
  Eigen::Vector3d last_extrication_plane_origin;
  estimateExtricationSlicingPlane(grinding_last_point, grinding_first_point, global_mesh_normal,
                                  last_extrication_plane_eq, last_extrication_plane_origin);

  vtkSmartPointer<vtkStripper> last_extrication_stripper = vtkSmartPointer<vtkStripper>::New();
  if(!sliceMeshWithPlane(extrication_mesh, last_extrication_plane_eq, last_extrication_plane_origin, last_extrication_stripper))
    return "Could not slice mesh for last extrication trajectory";

  if (last_extrication_stripper->GetOutput()->GetNumberOfLines() > 1)
  {
    ROS_ERROR_STREAM(
        "BezierGrindingSurfacing::generateTrajectory: Extrication stripper has " <<
        last_extrication_stripper->GetOutput()->GetNumberOfLines() << " lines");
    // FIXME Handle this case!
    //return "Extrication stripper has more than 1 line (mesh has a hole). Not implemented yet!";
  }

  EigenSTL::vector_Affine3d last_extrication_traj;
  if (!generateRobotPosesAlongStripper(last_extrication_stripper, last_extrication_traj))
    return "Could not generate robot poses for last extrication trajectory";

  const Eigen::Vector3d last_extrication_orientation_reference(grinding_first_point - grinding_last_point);
  if (last_extrication_orientation_reference == Eigen::Vector3d::Zero())
    return "Could not harmonize last extrication trajectory, reference vector is Zero !";

  if (harmonizeLineOrientation(last_extrication_traj, last_extrication_orientation_reference))
    ROS_INFO_STREAM("BezierGrindingSurfacing::generateTrajectory: Last extrication line reversed");

  // Last point and normal of the last grinding line
  Eigen::Vector3d first_point(grinding_trajectories.back().back().translation());
  Eigen::Vector3d first_point_normal(grinding_trajectories.back().back().linear().col(2));
  first_point_normal *= -1;

  // First point and normal of the first grinding line
  Eigen::Vector3d last_point(grinding_trajectories.front().front().translation());
  Eigen::Vector3d last_point_normal(grinding_trajectories.front().front().linear().col(2));
  last_point_normal *= -1;

  // Application of the extrication filter on the last extrication line
  if (!filterExtricationTrajectory(first_point, first_point_normal, last_point, last_point_normal,
                                   last_extrication_plane_eq, last_extrication_traj))
    return "Could not filter last extrication trajectory";

  invertXAxisOfPoses(last_extrication_traj);
  extrication_trajectories.push_back(last_extrication_traj);

  // Application of the grinding lean angle
  for (EigenSTL::vector_Affine3d &poses : grinding_trajectories)
  {
    for (Eigen::Affine3d &pose : poses)
      applyLeanAngle(pose, axis_of_rotation_, lean_angle_);
  }

  // The first pose of the trajectory is a point located above the first grinding pose.
  // We use the last pose of the last extrication trajectory which is located above the
  // first pose of the first grinding line. This point is stored as the first pose of the trajectory.
  Eigen::Affine3d start_pose(extrication_trajectories.back().back());
  // We keep the same orientation than the one of the first grinding pose of the first grinding line
  start_pose.linear() << grinding_trajectories.front().front().linear();

  // Generate Intermediate poses between the start pose of the trajectory and the first grinding line
  EigenSTL::vector_Affine3d first_grinding_line_intermediate_poses;
  Eigen::Vector3d end_pose(grinding_trajectories.front().front().translation());
  generateIntermediatePoseOnLine(first_grinding_line_intermediate_poses, start_pose.translation(), end_pose, 2);
  for (Eigen::Affine3d &pose : first_grinding_line_intermediate_poses)
  {
    // The intermediate poses have the same orientation than the first pose of the first grinding line
    pose.linear() << grinding_trajectories.front().front().linear();
  }

  // Generate intermediate poses between last extrication pose and first grinding pose
  // of each trajectories
  std::vector<EigenSTL::vector_Affine3d> end_intermediate_poses_trajectories;
  std::vector<EigenSTL::vector_Affine3d>::iterator grinding_it(grinding_trajectories.begin() + 1);
  for (std::vector<EigenSTL::vector_Affine3d>::iterator extrication_traj(extrication_trajectories.begin());
      extrication_traj != extrication_trajectories.end() - 1; extrication_traj++)
  {
    EigenSTL::vector_Affine3d intermediate_poses;
    Eigen::Vector3d start_pose((*extrication_traj).back().translation());
    Eigen::Vector3d end_pose((*grinding_it).front().translation());
    generateIntermediatePoseOnLine(intermediate_poses, start_pose, end_pose, 2);
    for (Eigen::Affine3d &pose : intermediate_poses)
    {
      //The intermediate pose has the same orientation than the first grinding line
      pose.linear() << (*grinding_it).front().linear();
    }
    end_intermediate_poses_trajectories.push_back(intermediate_poses);
    grinding_it++;
  }

  // Generate intermediate poses between last grinding pose and first extrication pose
  // of each trajectories
  std::vector<EigenSTL::vector_Affine3d> start_intermediate_poses_trajectories;
  std::vector<EigenSTL::vector_Affine3d>::iterator extrication_it(extrication_trajectories.begin());
  for (std::vector<EigenSTL::vector_Affine3d>::iterator grinding_traj(grinding_trajectories.begin());
      grinding_traj != grinding_trajectories.end(); grinding_traj++)
  {
    EigenSTL::vector_Affine3d intermediate_poses;
    Eigen::Vector3d start_pose((*grinding_traj).back().translation());
    Eigen::Vector3d end_pose((*extrication_it).front().translation());
    generateIntermediatePoseOnLine(intermediate_poses, start_pose, end_pose, 2);
    for (Eigen::Affine3d &pose : intermediate_poses)
    {
      // The intermediate pose has the same orientation than the first pose of the extrication line
      pose.linear() << (*extrication_it).front().linear();
    }
    start_intermediate_poses_trajectories.push_back(intermediate_poses);
    extrication_it++;
  }

  unsigned index(0);
  if (display_markers)
  {
    for (EigenSTL::vector_Affine3d traj : grinding_trajectories)
    {
      if (index > 11)
        index = 0;
      displayTrajectory(traj, visualToolsColorFromIndex(index++), true);
    }

    index = 0;
    for (EigenSTL::vector_Affine3d traj : extrication_trajectories)
    {
      if (index > 11)
        index = 0;
      displayTrajectory(traj, visualToolsColorFromIndex(index++), true);
    }

    for (EigenSTL::vector_Affine3d traj : end_intermediate_poses_trajectories)
    {
      if (index > 11)
        index = 0;
      displayTrajectory(traj, visualToolsColorFromIndex(index++), true);
    }

    for (EigenSTL::vector_Affine3d traj : start_intermediate_poses_trajectories)
    {
      if (index > 11)
        index = 0;
      displayTrajectory(traj, visualToolsColorFromIndex(index++), true);
    }

    // Display the start pose of the trajectory
    Eigen::Affine3d display_start_pose(start_pose);
    visual_tools_->publishXArrow(display_start_pose, rviz_visual_tools::GREEN, rviz_visual_tools::XXXXSMALL, 0.008);
    visual_tools_->publishZArrow(display_start_pose, rviz_visual_tools::GREEN, rviz_visual_tools::XXXXSMALL, 0.008);
    start_pose.translation() -= 0.01 * display_start_pose.affine().col(2).head<3>();
    visual_tools_->publishText(display_start_pose, "Start pose", rviz_visual_tools::GREEN, rviz_visual_tools::SMALL, false);
    visual_tools_->triggerBatchPublish();

    // Display intermediates poses between trajectory start point and the first grinding point of the first grinding line
    displayTrajectory(first_grinding_line_intermediate_poses, visualToolsColorFromIndex(index++), true);

  }

  // Add intermediates poses to extrication trajectories
  std::vector<EigenSTL::vector_Affine3d>::iterator end_intermediate_poses_it(end_intermediate_poses_trajectories.begin());
  for (std::vector<EigenSTL::vector_Affine3d>::iterator extrication_traj(extrication_trajectories.begin());
      extrication_traj != extrication_trajectories.end() - 1; extrication_traj++)
  {
    (*extrication_traj).insert((*extrication_traj).end(), (*end_intermediate_poses_it).begin(),
                               (*end_intermediate_poses_it).end());
    end_intermediate_poses_it++;
  }

  // Add intermediates poses to extrication trajectories
    std::vector<EigenSTL::vector_Affine3d>::iterator start_intermediate_poses_it(start_intermediate_poses_trajectories.begin());
    for (std::vector<EigenSTL::vector_Affine3d>::iterator extrication_traj(extrication_trajectories.begin());
        extrication_traj != extrication_trajectories.end(); extrication_traj++)
    {
      (*extrication_traj).insert((*extrication_traj).begin(), (*start_intermediate_poses_it).begin(),
                                 (*start_intermediate_poses_it).end());
      start_intermediate_poses_it++;
    }

  // Add one grinding traj, one extrication traj...
  trajectory.clear();
  is_grinding_pose.clear();
  std::vector<EigenSTL::vector_Affine3d>::iterator extrication_iterator(extrication_trajectories.begin());

  trajectory.push_back(start_pose);
  is_grinding_pose.push_back(false);
  for (Eigen::Affine3d &pose : first_grinding_line_intermediate_poses)
  {
    // Add intermediate poses between the start pose of the trajectory and the first grinding pose
    trajectory.push_back(pose);
    is_grinding_pose.push_back(false);
  }

  for (EigenSTL::vector_Affine3d grinding_traj : grinding_trajectories)
  {
    for (Eigen::Affine3d grinding_pose : grinding_traj)
    {
      trajectory.push_back(grinding_pose);
      is_grinding_pose.push_back(true);
    }
    if (extrication_iterator != extrication_trajectories.end())
    {
      for (Eigen::Affine3d extrication_pose : *extrication_iterator)
      {
        trajectory.push_back(extrication_pose);
        is_grinding_pose.push_back(false);
      }
      extrication_iterator++;
    }
  }

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
  vtkIdType min_point_index(0);
  vtkIdType max_point_index(0);
  double min = std::numeric_limits<double>::max();
  double max = std::numeric_limits<double>::min();
  for (vtkIdType index = 0; index < polydata->GetNumberOfPoints(); index++)
  {
    double p[3];
    polydata->GetPoint(index, p);
    Eigen::Vector3d point(p);
    Eigen::Vector3d vector_to_project(point - polydata_center);
    vector_to_project.normalize();
    double res = slicing_orientation.normalized().dot(vector_to_project);

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

  double max_point_coord[3], min_point_coord[3];
  polydata->GetPoint(max_point_index, max_point_coord);
  polydata->GetPoint(min_point_index, min_point_coord);
  Eigen::Vector3d max_point(max_point_coord);
  Eigen::Vector3d min_point(min_point_coord);
  Eigen::Vector3d distance(max_point - min_point);
  double width = grinding_disk_machining_width_ * (1 - (double)covering_percentage_ / 100);
  double line_count = std::floor((distance.norm()) / width);

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
        "BezierGrindingSurfacing::generateRobotPosesAlongStripper: " << vtk_observer_->GetErrorMessage());
    vtk_observer_->Clear();
    return false;
  }

  vtkIdType* indices;
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
          "BezierGrindingSurfacing::generateRobotPosesAlongStripper: " << vtk_observer_->GetErrorMessage());
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
  free(indices);

  filterNeighborPosesTooClose(point_normal_table, 5e-3);

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
      return false;
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
      return false;
    }

    // Keep the last pose in memory.
    // It is used to generate the orientation of the last pose of the line
    last_pose = pose;
    trajectory.push_back(pose);
  }

  return true;
}

void BezierGrindingSurfacing::invertXAxisOfPoses(EigenSTL::vector_Affine3d &line)
{
  // Reverse X and Y vectors for each pose of the line
  // Z stays untouched, this is a PI rotation on the Z axis
  for (Eigen::Affine3d &pose : line)
  {
    pose.linear().col(0) *= -1;
    pose.linear().col(1) *= -1;
  }
}

void BezierGrindingSurfacing::filterNeighborPosesTooClose(BezierPointNormalTable &trajectory,
                                                          const double minimal_distance)
{

  if (trajectory.size() <= 1)
    return;

  // Vector that contain the indices for all points to be removed from the trajectory vector
  std::vector<unsigned> indices_to_be_removed;
  // The offset variable allows to count how much point will be skipped before reaching a point
  // located far enough from the current point. Every point located between the index of the
  // current point and the next point located at index of current point + offset is deleted.
  // The offset is set to 1 at initialisation, so the next point will be immediately the one
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
      // Distance computing between the current point and the designated next point
      double distance = (point - next_point).norm();
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
          // The new distance between the current point and the next point is computed
          distance = (point - next_point).norm();
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

}

bool BezierGrindingSurfacing::filterExtricationTrajectory(const Eigen::Vector3d &first_point,
                                   const Eigen::Vector3d &first_point_normal,
                                   const Eigen::Vector3d &last_point,
                                   const Eigen::Vector3d &last_point_normal,
                                   const Eigen::Vector4d &plane_equation,
                                   EigenSTL::vector_Affine3d &trajectory)
{
  // The extrication filter use a 2D angle comparison method to reject inaccessible points from the extrication trajectory.
  // The filter apply the comparison algorithm on both side of the extrication trajectory :
  // - At the beginning of the trajectory, it takes the normal (Nor) of the last grinding point of the current grinding line (GN)
  // - and make a projection of this vector onto the plane used to slice the mesh and generate the extrication trajectory.
  // - It then computes a vector (AN) between the last grinding point A a extrication point N. Thanks to the projection made before,
  // - the normal (Nor) and the vector (AN) are located into the same 2D plane. An angle computation it's performed.
  // - The sign of the first angle computed is taken as reference. The operation is then repeated with the
  // - following points (N+1, N+2 ...) as long as the current angle computed has the same sign than the reference angle.
  // - A change of sign occur when we reach a point (T) located just above the grinding point, thus
  // - accessible for the robot. Finally all the point located before (T) are removed from the trajectory.
  // - The same process is repeated at the end side of the trajectory, but the normal (Nor) is the normal at the first point of the
  // - next grinding line (GN +1) trajectory.
  // The formula used to project the vector is : A - (A.n)n with A the vector to project and N the normal of the plane
  // The formula used to compute the 2D angle is : arctan(determinant, (dot product))

  if (trajectory.size() <= 2)
  {
    ROS_ERROR_STREAM("BezierGrindingSurfacing::filterExtricationTrajectory: Trajectory is too small!");
    return false;
  }

  // Get the first point of the grinding line
  Eigen::Vector3d line_first_point(first_point);
  // Get the normal of the first point of the grinding line
  Eigen::Vector3d line_first_point_normal(first_point_normal.normalized());
  // Get the last point of the grinding line
  Eigen::Vector3d line_last_point(last_point);
  // Get the normal of the last point of the grinding line
  Eigen::Vector3d line_last_point_normal(last_point_normal.normalized());

  // References vectors being projected onto the plane containing the extrication trajectory
  Eigen::Vector3d line_first_point_normal_projected(Eigen::Vector3d::Identity());
  Eigen::Vector3d line_last_point_normal_projected(Eigen::Vector3d::Identity());

  // Computation of the vectors being projected using : A - (A.n)n with A the vector to be project and n the plane normal
  line_first_point_normal_projected = (line_first_point_normal
          - (line_first_point_normal.dot(plane_equation.head<3>().normalized())) * plane_equation.head<3>().normalized()).normalized();

  line_last_point_normal_projected = (line_last_point_normal
          - (line_last_point_normal.dot(plane_equation.head<3>().normalized())) * plane_equation.head<3>().normalized()).normalized();

  // Iterator containing the index of the first point of the filtered trajectory
  EigenSTL::vector_Affine3d::iterator start_of_filtered_line(trajectory.begin());
  // Iterator containing the index of the last point of the filtered trajectory
  EigenSTL::vector_Affine3d::iterator end_of_filtered_line(trajectory.end() - 1);

  // Boolean allowing to compare the sign of the angles computed
  bool sign = false;

  // We compute the the first angle to get the reference sign
  Eigen::Vector3d vect((trajectory.front().translation() - line_first_point).normalized());
  double dot = line_first_point_normal_projected[0] * line_first_point_normal_projected[1] + vect[0] * vect[1];
  double det = line_first_point_normal_projected[0] * vect[1] - vect[0] * line_first_point_normal_projected[1];
  double angle = atan2(det, dot);
  sign = std::signbit(angle);

  for (EigenSTL::vector_Affine3d::iterator it(trajectory.begin() + 1); it != trajectory.end(); it++)
  {
    Eigen::Vector3d vect(((*it).translation() - line_first_point).normalized());
    double dot = line_first_point_normal_projected[0] * line_first_point_normal_projected[1] + vect[0] * vect[1];
    double det = line_first_point_normal_projected[0] * vect[1] - vect[0] * line_first_point_normal_projected[1];
    double angle = atan2(det, dot);
    if (std::signbit(angle) != sign)
    {
      // The sign has toggled, we save the index of the current point and we stop the process
      start_of_filtered_line = it;
      break;
    }
  }

  // We compute the first angle to get the reference sign
  vect = (trajectory.back().translation() - line_last_point).normalized();
  dot = line_last_point_normal_projected[0] * line_last_point_normal_projected[1] + vect[0] * vect[1];
  det = line_last_point_normal_projected[0] * vect[1] - vect[0] * line_last_point_normal_projected[1];
  angle = atan2(det, dot);
  sign = std::signbit(angle);

  for (EigenSTL::vector_Affine3d::reverse_iterator it(trajectory.rbegin() + 1); it != trajectory.rend(); ++it)
  {
    Eigen::Vector3d vect(((*it).translation() - line_last_point).normalized());
    double dot = line_last_point_normal_projected[0] * line_last_point_normal_projected[1] + vect[0] * vect[1];
    double det = line_last_point_normal_projected[0] * vect[1] - vect[0] * line_last_point_normal_projected[1];
    double angle = atan2(det, dot);
    if (std::signbit(angle) != sign)
    {
      // The sign has toggled, we save the index of the current point and we stop the process
      end_of_filtered_line = it.base() - 1;
      break;
    }
  }

  if (start_of_filtered_line > end_of_filtered_line)
  {
    ROS_ERROR_STREAM("BezierGrindingSurfacing::filterExtricationTrajectory: Bad start/end indices!");
    return false;
  }

  EigenSTL::vector_Affine3d filtered_trajectory;
  while (start_of_filtered_line <= end_of_filtered_line)
  {
    Eigen::Affine3d filtered_pose = *start_of_filtered_line;
    filtered_trajectory.push_back(filtered_pose);
    start_of_filtered_line++;
  }
  // Clear the current trajectory vector
  trajectory.clear();
  // Insert all the filtered points into the trajectory vector
  trajectory.insert(trajectory.begin(), filtered_trajectory.begin(), filtered_trajectory.end());
  return true;
}

bool BezierGrindingSurfacing::harmonizeLineOrientation(EigenSTL::vector_Affine3d &poses_on_line,
                                                       const Eigen::Vector3d &direction_ref)
{
  if (poses_on_line.size() <= 1)
    return false;

  // Compare orientation of lines with reference
  Eigen::Vector3d current_line_orientation(poses_on_line.back().translation() - poses_on_line.front().translation());

  // If dot product > 0 we don't invert the line
  if (direction_ref.dot(current_line_orientation) > 0)
    return false;

  std::reverse(poses_on_line.begin(), poses_on_line.end());

  // We reversed the line order so we need to reverse the axis X/Y of each pose as well
  invertXAxisOfPoses(poses_on_line);
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
