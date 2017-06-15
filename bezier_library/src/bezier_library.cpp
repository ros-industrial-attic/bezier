#include <bezier_library/bezier_library.hpp>

Bezier::Bezier()
{
  vtk_observer_ = vtkSmartPointer<ErrorObserver>::New();
  ROS_INFO_STREAM("Bezier::Bezier: RViz visualization tool is initialized in 'base' "
                  "and the topic name is 'rviz_visual_tools'");
  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link"));
  setDilationParameters(); // Load default dilation parameters
}

void Bezier::waitForRvizVisualToolsSubscriber()
{
  visual_tools_->loadMarkerPub();
  visual_tools_->waitForMarkerPub();
}

bool Bezier::appendInputMesh(const std::string file_absolute_path)
{
  // Determine type of mesh
  std::string file_extension = boost::filesystem::extension(file_absolute_path);

  vtkSmartPointer<vtkAbstractPolyDataReader> reader;
  if (boost::algorithm::iequals(file_extension, ".obj"))
    reader = vtkSmartPointer<vtkOBJReader>::New();
  else if (boost::algorithm::iequals(file_extension, ".ply"))
    reader = vtkSmartPointer<vtkPLYReader>::New();
  else if (boost::algorithm::iequals(file_extension, ".stl"))
    reader = vtkSmartPointer<vtkSTLReader>::New();
  else
    return false;

  reader->SetFileName(file_absolute_path.c_str());
  reader->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
  reader->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
  vtk_observer_->Clear();
  reader->Update();

  if (vtk_observer_->GetWarning())
  {
    ROS_WARN_STREAM(
        "Bezier::appendInputMesh: " << file_absolute_path << std::endl << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }
  if (vtk_observer_->GetError())
  {
    ROS_ERROR_STREAM(
        "Bezier::appendInputMesh: " << file_absolute_path << std::endl << vtk_observer_->GetErrorMessage());
    vtk_observer_->Clear();
    return false;
  }

  vtkSmartPointer<vtkPolyData> polydata;
  polydata = reader->GetOutput();
  input_meshes_.push_back(polydata);
  return true;
}

bool Bezier::saveMesh(const std::string file_absolute_path,
                      const vtkSmartPointer<vtkPolyData> &polydata)
{
  // Determine type of mesh
  std::string file_extension = boost::filesystem::extension(file_absolute_path);

  if (boost::algorithm::iequals(file_extension, ".ply"))
  {
    vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
    writer->SetFileName(file_absolute_path.c_str());
    writer->SetInputData(polydata);
    writer->SetFileTypeToBinary();
    writer->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
    writer->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
    vtk_observer_->Clear();
    writer->Update();
  }
  else if (boost::algorithm::iequals(file_extension, ".stl"))
  {
    vtkSmartPointer<vtkSTLWriter> writer = vtkSmartPointer<vtkSTLWriter>::New();
    writer->SetFileName(file_absolute_path.c_str());
    writer->SetInputData(polydata);
    writer->SetFileTypeToBinary();
    writer->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
    writer->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
    vtk_observer_->Clear();
    writer->Update();
  }
  else
    return false;

  if (vtk_observer_->GetWarning())
  {
    ROS_WARN_STREAM("Bezier::saveMesh: " << file_absolute_path << std::endl << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }
  if (vtk_observer_->GetError())
  {
    ROS_ERROR_STREAM("Bezier::saveMesh: " << file_absolute_path << std::endl << vtk_observer_->GetErrorMessage());
    vtk_observer_->Clear();
    return false;
  }

  return true;
}

void Bezier::displayMesh(const std::shared_ptr<ros::Publisher> &mesh_publisher,
                         const std::string mesh_path,
                         const float r,
                         const float g,
                         const float b,
                         const float a,
                         std::string frame_id)
{
  if (!mesh_publisher)
  {
    ROS_ERROR_STREAM("Bezier::displayMesh: Publisher is not initialized");
    return;
  }

  // Create a mesh marker from ply files
  visualization_msgs::Marker mesh_marker;
  mesh_marker.header.frame_id = frame_id;
  mesh_marker.header.stamp = ros::Time::now();
  mesh_marker.id = 0;
  mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh_marker.mesh_resource = mesh_path;
  mesh_marker.action = visualization_msgs::Marker::ADD;
  mesh_marker.scale.x = mesh_marker.scale.y = mesh_marker.scale.z = 1;
  mesh_marker.color.r = r;
  mesh_marker.color.g = g;
  mesh_marker.color.b = b;
  mesh_marker.color.a = a;
  mesh_marker.lifetime = ros::Duration();

  if (mesh_publisher->getNumSubscribers() < 1)
    ROS_WARN_STREAM("Bezier::displayMesh: There is no subscriber to the \"" << mesh_publisher->getTopic() << "\" marker!");

  mesh_publisher->publish(mesh_marker);
}

void Bezier::displayTrajectory(const EigenSTL::vector_Affine3d &trajectory,
                               rviz_visual_tools::colors color,
                               const bool display_normals,
                               const bool display_labels)
{
  EigenSTL::vector_Vector3d points;
  for(Eigen::Affine3d tmp : trajectory)
    points.push_back(tmp.translation());

  visual_tools_->publishPath(points, color, 0.0005);

  unsigned index(0);
  if (display_normals)
  {
    for (Eigen::Affine3d tmp : trajectory)
    {
      visual_tools_->publishAxis(tmp, 0.003, 0.0005);
      if (display_labels)
      {
        tmp.translation() -= 0.01 * tmp.affine().col(2).head<3>();
        visual_tools_->publishText(tmp, boost::lexical_cast<std::string>(index++), color, rviz_visual_tools::SMALL, false);
      }
    }
  }
  visual_tools_->trigger();
}

bool Bezier::computeNormals(vtkSmartPointer<vtkPolyData> &polydata)
{
  vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
  normals->SetInputData(polydata);
  normals->ComputeCellNormalsOn();
  normals->ComputePointNormalsOn();
  normals->ConsistencyOn(); // Orient all normals in a consistent manner
  normals->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
  normals->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
  vtk_observer_->Clear();
  normals->Update();

  if (vtk_observer_->GetWarning())
  {
    ROS_WARN_STREAM("Bezier::computeNormals: " << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }
  if (vtk_observer_->GetError())
  {
    ROS_ERROR_STREAM("Bezier::computeNormals: " << vtk_observer_->GetErrorMessage());
    vtk_observer_->Clear();
    return false;
  }
  polydata = normals->GetOutput();

  return true;
}

bool Bezier::estimateGlobalMeshNormal(vtkSmartPointer<vtkPolyData> &polydata,
                                      Eigen::Vector3d &mesh_normal,
                                      const unsigned iterations)
{
  pcl::PolygonMesh mesh;
  pcl::VTKUtils::vtk2mesh(polydata, mesh);
  PointCloudT::Ptr cloud(new PointCloudT);
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

  if (cloud->empty())
    return false;

  // Computation of the bounds of the polydata
  double bounds[6];
  polydata->GetBounds(bounds);
  // For the x bound we compute x_size = x_max - x_min
  double x_size = bounds[1] - bounds[0];
  // For the y bound we compute y_size = y_max - y_min
  double y_size = bounds[3] - bounds[2];
  // For the z bound we compute z_size = z_max - z_min
  double z_size = bounds[5] - bounds[4];
  // The threshold is equals to max(x_size,y_size,z_size)
  // That is because the plane model has to fit all points of the inputPolyData
  double threshold = std::max(x_size, y_size);
  threshold = std::max(threshold, z_size);

  // RANSAC method application
  pcl::SACSegmentation<PointT> segmentation;
  segmentation.setInputCloud(cloud);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(threshold);
  segmentation.setMaxIterations(iterations);

  pcl::PointIndices inliers;
  pcl::ModelCoefficients model_coefficients;
  model_coefficients.values.resize(3); // Plane
  segmentation.segment(inliers, model_coefficients);
  mesh_normal[0] = model_coefficients.values[0];
  mesh_normal[1] = model_coefficients.values[1];
  mesh_normal[2] = model_coefficients.values[2];
  mesh_normal.normalize();

  // RViz visual tools
  Eigen::Affine3d pose(Eigen::Affine3d::Identity());
  double centroid[3];
  centroid[0] = polydata->GetCenter()[0];
  centroid[1] = polydata->GetCenter()[1];
  centroid[2] = polydata->GetCenter()[2];
  pose.translation() << centroid[0], centroid[1], centroid[2];
  pose.affine().col(0).head<3>() << mesh_normal;
  pose.affine().col(2).head<3>() << mesh_normal[2], 0, -mesh_normal[0];
  pose.affine().col(1).head<3>() << pose.affine().col(2).head<3>().cross(pose.affine().col(0).head<3>());
  visual_tools_->publishArrow(pose, rviz_visual_tools::RED, rviz_visual_tools::XXSMALL);
  pose.translation() += 0.07 * pose.affine().col(0).head<3>();
  visual_tools_->publishText(pose, "Global normal", rviz_visual_tools::RED, rviz_visual_tools::SMALL, false);
  visual_tools_->trigger();
  return true;
}

bool Bezier::estimateSlicingOrientation(vtkSmartPointer<vtkPolyData> &polydata,
                                        Eigen::Vector3d &mesh_normal,
                                        Eigen::Vector3d &orientation)
{
  // FIXME The slicing orientation generated is not always optimal:
  // We can loop through all orientations (10 deg / 10 deg), find the
  // min/max point and compute the distance between the two.
  // After that, we keep the orientation that has the minimum distance = minimum slice number
  if(!estimateGlobalMeshNormal(polydata, mesh_normal))
    return false;

  if (orientation == Eigen::Vector3d::Zero())
  {
    orientation = Eigen::Vector3d(mesh_normal[2], 0, -mesh_normal[0]);
    orientation.normalize();

    if (mesh_normal.dot(orientation) > 1e-10) // Numerical issues
    {
      ROS_ERROR_STREAM("Bezier::estimateSlicingOrientation: Scalar product is not 0! " << mesh_normal.dot(orientation));
      return false;
    }
  }

  // RViz visual tools
  Eigen::Affine3d pose(Eigen::Affine3d::Identity());
  double centroid[3];
  centroid[0] = polydata->GetCenter()[0];
  centroid[1] = polydata->GetCenter()[1];
  centroid[2] = polydata->GetCenter()[2];
  pose.translation() << centroid[0], centroid[1], centroid[2];
  pose.affine().col(0).head<3>() << orientation;
  pose.affine().col(2).head<3>() << orientation[2], 0, -orientation[0];
  pose.affine().col(1).head<3>() << pose.affine().col(2).head<3>().cross(pose.affine().col(0).head<3>());
  visual_tools_->publishArrow(pose, rviz_visual_tools::BLUE, rviz_visual_tools::XXSMALL);
  pose.translation() += 0.07 * pose.affine().col(0).head<3>();
  visual_tools_->publishText(pose, "Slicing orientation", rviz_visual_tools::BLUE, rviz_visual_tools::SMALL, false);
  visual_tools_->trigger();
  return true;
}

// FIXME estimateSlicingPlanes does not find the exact number of planes
void Bezier::estimateSlicingPlanes(const vtkSmartPointer<vtkPolyData> &polydata,
                                   const Eigen::Vector3d &slicing_orientation,
                                   const Eigen::Vector3d &polydata_center,
                                   const double tool_effective_diameter,
                                   const unsigned covering_percentage,
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
  double width = tool_effective_diameter * (1 - (double)covering_percentage / 100);
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

void Bezier::estimateExtricationSlicingPlane(const Eigen::Vector3d &line_n_last_point,
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

bool Bezier::sliceMeshWithPlane(const vtkSmartPointer<vtkPolyData> &polydata,
                                const Eigen::Vector4d &plane_equation,
                                const Eigen::Vector3d &origin,
                                vtkSmartPointer<vtkStripper> &stripper)
{
  // Parse the coordinates of the origin point into a array of double
  double origin_pt[3];
  origin_pt[0] = origin[0];
  origin_pt[1] = origin[1];
  origin_pt[2] = origin[2];

  // Set the origin point (start point for cut) into the plane cutting function
  vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
  plane->SetOrigin(origin_pt);

  // Set the cutting direction through the three first values contained into plane_equation (Vector4d)
  // These values represent a direction of cut for the cutting plane function
  plane->SetNormal(plane_equation[0], plane_equation[1], plane_equation[2]);

  // Build a vtkCutter used to cut the polyData
  vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
  cutter->SetCutFunction(plane);
  cutter->SetInputData(polydata);
  cutter->SetValue(0, plane_equation[3]); // Plane offset
  cutter->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
  cutter->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
  vtk_observer_->Clear();
  cutter->Update();

  if (vtk_observer_->GetWarning())
  {
    ROS_WARN_STREAM("Bezier::sliceMeshWithPlane: " << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }
  if (vtk_observer_->GetError())
  {
    ROS_ERROR_STREAM("Bezier::sliceMeshWithPlane: " << vtk_observer_->GetErrorMessage());
    vtk_observer_->Clear();
    return false;
  }

  vtkSmartPointer<vtkTriangleFilter> triangle_filter = vtkSmartPointer<vtkTriangleFilter>::New();
  triangle_filter->SetInputConnection(cutter->GetOutputPort());
  triangle_filter->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
  triangle_filter->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
  vtk_observer_->Clear();
  triangle_filter->Update();

  if (vtk_observer_->GetWarning())
  {
    ROS_WARN_STREAM("Bezier::sliceMeshWithPlane: " << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }
  if (vtk_observer_->GetError())
  {
    ROS_ERROR_STREAM("Bezier::sliceMeshWithPlane: " << vtk_observer_->GetErrorMessage());
    vtk_observer_->Clear();
    return false;
  }

  // Generate polylines from cutter
  stripper->SetInputConnection(triangle_filter->GetOutputPort());
  stripper->JoinContiguousSegmentsOn(); //
  stripper->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
  stripper->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
  vtk_observer_->Clear();
  stripper->Update();

  if (vtk_observer_->GetWarning())
  {
    ROS_WARN_STREAM("Bezier::sliceMeshWithPlane: " << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }
  if (vtk_observer_->GetError())
  {
    ROS_ERROR_STREAM("Bezier::sliceMeshWithPlane: " << vtk_observer_->GetErrorMessage());
    vtk_observer_->Clear();
    return false;
  }

  return true;
}

bool Bezier::sliceMeshWithPlanes(const vtkSmartPointer<vtkPolyData> &polydata,
                                 const EigenSTL::vector_Vector4d &plane_equations,
                                 const Eigen::Vector3d &origin,
                                 std::vector<vtkSmartPointer<vtkStripper> > &strippers)
{
  strippers.clear();
  for (EigenSTL::vector_Vector4d::const_iterator it(plane_equations.begin()); it != plane_equations.end(); ++it)
  {
    vtkSmartPointer<vtkStripper> stripper = vtkSmartPointer<vtkStripper>::New();
    if (!sliceMeshWithPlane(polydata, *it, origin, stripper))
      return false;

    if (stripper->GetOutput()->GetNumberOfLines() != 0)
      strippers.push_back(stripper);
  }

  return true;
}

void Bezier::applyLeanAngle(Eigen::Affine3d &pose,
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

bool Bezier::generateRobotPosesAlongStripper(const vtkSmartPointer<vtkStripper> &line,
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
    ROS_ERROR_STREAM("BezierGrindingSurfacing::generateRobotPosesAlongStripper: " << vtk_observer_->GetErrorMessage());
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

void Bezier::invertXAxisOfPoses(EigenSTL::vector_Affine3d &line)
{
  // Reverse X and Y vectors for each pose of the line
  // Z stays untouched, this is a PI rotation on the Z axis
  for (Eigen::Affine3d &pose : line)
  {
    pose.linear().col(0) *= -1;
    pose.linear().col(1) *= -1;
  }
}

void Bezier::filterNeighborPosesTooClose(BezierPointNormalTable &trajectory,
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

bool Bezier::filterExtricationTrajectory(const Eigen::Vector3d &first_point,
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
  line_first_point_normal_projected =
      (line_first_point_normal
          - (line_first_point_normal.dot(plane_equation.head<3>().normalized())) * plane_equation.head<3>().normalized()).normalized();

  line_last_point_normal_projected =
      (line_last_point_normal
          - (line_last_point_normal.dot(plane_equation.head<3>().normalized())) * plane_equation.head<3>().normalized()).normalized();

  // Iterator containing the index of the first point of the filtered trajectory
  EigenSTL::vector_Affine3d::iterator start_of_filtered_line(trajectory.begin());
  // Iterator containing the index of the last point of the filtered trajectory
  EigenSTL::vector_Affine3d::iterator end_of_filtered_line(trajectory.end() - 1);

  // Boolean allowing to compare the sign of the angles computed
  bool sign = false;

  // Computation of the middle index of the trajectory vector
  unsigned middle_index = std::ceil((trajectory.size() / 2));

  // We compute the the first angle to get the reference sign
  Eigen::Vector3d vect((trajectory[middle_index].translation() - line_first_point).normalized());
  double dot = line_first_point_normal_projected[0] * line_first_point_normal_projected[1] + vect[0] * vect[1];
  double det = line_first_point_normal_projected[0] * vect[1] - vect[0] * line_first_point_normal_projected[1];
  double angle = atan2(det, dot);
  sign = std::signbit(angle);

  for (EigenSTL::vector_Affine3d::iterator it(trajectory.begin() + middle_index); it != trajectory.begin(); it--)
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
  vect = (trajectory[middle_index].translation() - line_last_point).normalized();
  dot = line_last_point_normal_projected[0] * line_last_point_normal_projected[1] + vect[0] * vect[1];
  det = line_last_point_normal_projected[0] * vect[1] - vect[0] * line_last_point_normal_projected[1];
  angle = atan2(det, dot);
  sign = std::signbit(angle);

  for (EigenSTL::vector_Affine3d::iterator it(trajectory.begin() + middle_index); it != trajectory.end(); it++)
  {
    Eigen::Vector3d vect(((*it).translation() - line_last_point).normalized());
    double dot = line_last_point_normal_projected[0] * line_last_point_normal_projected[1] + vect[0] * vect[1];
    double det = line_last_point_normal_projected[0] * vect[1] - vect[0] * line_last_point_normal_projected[1];
    double angle = atan2(det, dot);
    if (std::signbit(angle) != sign)
    {
      // The sign has toggled, we save the index of the current point and we stop the process
      end_of_filtered_line = it;
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

bool Bezier::harmonizeLineOrientation(EigenSTL::vector_Affine3d &poses_on_line,
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

void Bezier::setDilationParameters(const double i, const double j, const double k,
                                   const double max_distance)
{
  dilation_sample_dimensions_i_ = i;
  dilation_sample_dimensions_j_ = j;
  dilation_sample_dimensions_k_ = k;
  dilation_maximum_distance_ = max_distance;
}

void Bezier::setDilationParameters(const double * sample_dimensions,
                                   const double max_distance)
{
  dilation_sample_dimensions_i_ = sample_dimensions[0];
  dilation_sample_dimensions_j_ = sample_dimensions[1];
  dilation_sample_dimensions_k_ = sample_dimensions[2];
  dilation_maximum_distance_ = max_distance;
}

bool Bezier::dilate(vtkSmartPointer<vtkPolyData> &polydata,
                    const double radius)
{

  if ((dilation_sample_dimensions_i_ == 50) &&
     (dilation_sample_dimensions_j_ == 50) &&
     (dilation_sample_dimensions_k_ == 50) &&
     (dilation_maximum_distance_ == 0))
  {
    ROS_INFO_STREAM("Bezier::dilate: default parameters will be used to dilate the mesh");
  }

  // Bounding box of the polydata
  double bounds[6];
  polydata->GetBounds(bounds);
  double max_side_length = std::max(bounds[1] - bounds[0], bounds[3] - bounds[2]);
  max_side_length = std::max(max_side_length, bounds[5] - bounds[4]);
  double threshold = radius / max_side_length;

  // Dilation
  // FIXME Make sure these parameters are smart and never need to be tweaked!
  vtkSmartPointer<vtkImplicitModeller> implicit_modeller = vtkSmartPointer<vtkImplicitModeller>::New();
  implicit_modeller->SetProcessModeToPerVoxel();  // Optimize process -> per voxel and not per cell
  implicit_modeller->SetSampleDimensions(dilation_sample_dimensions_i_,
                                         dilation_sample_dimensions_j_,
                                         dilation_sample_dimensions_k_);
  implicit_modeller->SetInputData(polydata);
  implicit_modeller->AdjustBoundsOn();
  implicit_modeller->SetAdjustDistance(threshold);


  if (dilation_maximum_distance_ == 0)
  {
    if (2 * threshold > 1.0)
      implicit_modeller->SetMaximumDistance(1.0);
    else
      implicit_modeller->SetMaximumDistance(2 * threshold); // 2*threshold in order to be sure -> long time but smoothed dilation
  }
  else
  {
    implicit_modeller->SetMaximumDistance(dilation_maximum_distance_);
  }
  implicit_modeller->ComputeModelBounds(polydata);
  implicit_modeller->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
  implicit_modeller->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
  vtk_observer_->Clear();
  implicit_modeller->Update();

  if (vtk_observer_->GetWarning())
  {
    ROS_WARN_STREAM("Bezier::dilate: " << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }
  if (vtk_observer_->GetError())
  {
    ROS_ERROR_STREAM("Bezier::dilate: " << vtk_observer_->GetErrorMessage());
    vtk_observer_->Clear();
    return false;
  }

  vtkSmartPointer<vtkMarchingCubes> surface = vtkSmartPointer<vtkMarchingCubes>::New();
  surface->SetInputConnection(implicit_modeller->GetOutputPort());
  surface->ComputeNormalsOn();
  surface->SetValue(0, radius);
  surface->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
  surface->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
  vtk_observer_->Clear();
  surface->Update();

  if (vtk_observer_->GetWarning())
  {
    ROS_WARN_STREAM("Bezier::dilate: " << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }
  if (vtk_observer_->GetError())
  {
    ROS_ERROR_STREAM("Bezier::dilate: " << vtk_observer_->GetErrorMessage());
    vtk_observer_->Clear();
    return false;
  }

  vtkSmartPointer<vtkPolyData> inversed_dilated_mesh = vtkSmartPointer<vtkPolyData>::New();
  inversed_dilated_mesh = surface->GetOutput();

  vtkSmartPointer<vtkReverseSense> mesh_reverser = vtkSmartPointer<vtkReverseSense>::New();
  mesh_reverser->SetInputData(inversed_dilated_mesh);
  mesh_reverser->SetOutput(polydata);
  mesh_reverser->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
  mesh_reverser->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
  vtk_observer_->Clear();
  mesh_reverser->Update();

  if (vtk_observer_->GetWarning())
  {
    ROS_WARN_STREAM("Bezier::dilate: " << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }
  if (vtk_observer_->GetError())
  {
    ROS_ERROR_STREAM("Bezier::dilate: " << vtk_observer_->GetErrorMessage());
    vtk_observer_->Clear();
    return false;
  }

  return true;
}

bool Bezier::keepUpperPartofDilatedMesh(vtkSmartPointer<vtkPolyData> &base_polydata,
                                        const vtkSmartPointer<vtkPolyData> &dilated_polydata,
                                        vtkSmartPointer<vtkPolyData> &upper_part_dilated_polydata)
{
  // Build a k-d tree
  vtkSmartPointer<vtkKdTreePointLocator> kd_tree = vtkSmartPointer<vtkKdTreePointLocator>::New();
  kd_tree->SetDataSet(base_polydata);
  kd_tree->BuildLocator();

  upper_part_dilated_polydata->DeepCopy(dilated_polydata);

  // Build cell and link in dilated_polydata
  upper_part_dilated_polydata->BuildCells();
  upper_part_dilated_polydata->BuildLinks();

  if (base_polydata->GetPointData()->GetNormals() == 0)
  {
    ROS_WARN_STREAM("Bezier::keepUpperPartofDilatedMesh: Computing normals...");
    computeNormals(base_polydata);
  }

  vtkSmartPointer<vtkFloatArray> point_normal_array(
      vtkFloatArray::SafeDownCast(base_polydata->GetPointData()->GetNormals()));
  if (!point_normal_array)
    return false;

  // For each cell in upper_part_dilated_polydata
  for (vtkIdType index_cell = 0; index_cell < upper_part_dilated_polydata->GetNumberOfCells(); index_cell++)
  {
    // Get cell
    vtkSmartPointer<vtkCell> cell(upper_part_dilated_polydata->GetCell(index_cell));
    // Get center of cell
    double pcoords[3] = {0, 0, 0};
    double *weights = new double[upper_part_dilated_polydata->GetMaxCellSize()];
    int sub_id = cell->GetParametricCenter(pcoords);
    double cell_center[3] = {0, 0, 0};
    cell->EvaluateLocation(sub_id, pcoords, cell_center, weights);
    free(weights);

    // Looking for the 2 closest points of cellCenter
    unsigned number_of_points = 2;
    vtkSmartPointer<vtkIdList> id_closest_points = vtkSmartPointer<vtkIdList>::New();
    kd_tree->FindClosestNPoints(number_of_points, cell_center, id_closest_points);

    std::vector<double> scalar_products_vector;
    // Store variable into vectors
    for(vtkIdType i = 0; i < number_of_points; ++i)
    {
      double closest_point_vector[3];
      Eigen::Vector3d direction_vector;
      double normal[3];

      // Get the closest point i on base_polydata
      base_polydata->GetPoint(id_closest_points->GetId(i), closest_point_vector);

      // Compute vector between cell_center on dilated_polydata and closest_point i on base_polydata
      direction_vector[0] = cell_center[0] - closest_point_vector[0];
      direction_vector[1] = cell_center[1] - closest_point_vector[1];
      direction_vector[2] = cell_center[2] - closest_point_vector[2];

      // Get normal of closest_point i
      point_normal_array->GetTuple(id_closest_points->GetId(i), normal);
      direction_vector.normalize();
      
      Eigen::Vector3d normal_3d(normal);
      scalar_products_vector.push_back(normal_3d.dot(direction_vector));
    }

    bool scalar_product_is_negative = true;
    for(unsigned i = 0; i < scalar_products_vector.size(); ++i)
    {
      if(scalar_products_vector[i] >= 0)
        scalar_product_is_negative = false;
    }

    // Keep the cell only if it belongs to the upper part of the mesh
    if (!vtkMath::IsFinite(cell_center[0]) ||
        !vtkMath::IsFinite(cell_center[1]) ||
        !vtkMath::IsFinite(cell_center[2]) ||
        scalar_product_is_negative)
      upper_part_dilated_polydata->DeleteCell(index_cell);

  }

  upper_part_dilated_polydata->RemoveDeletedCells();
  if (upper_part_dilated_polydata->GetNumberOfCells() == 0)
    return false;

  return true;
}

bool Bezier::removeIsolatedTrianglesFilter(vtkSmartPointer<vtkPolyData> &polydata,
                                           const unsigned minimal_number_of_cells)
{
  vtkSmartPointer<vtkPolyDataConnectivityFilter> connectivityFilter =
      vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
  connectivityFilter->SetExtractionModeToAllRegions();
  connectivityFilter->ColorRegionsOff();
  connectivityFilter->SetInputData(polydata);
  connectivityFilter->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
  connectivityFilter->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
  connectivityFilter->Update();

  if (vtk_observer_->GetWarning())
  {
    ROS_WARN_STREAM("Bezier::removeIsolatedTrianglesFilter: " << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }
  if (vtk_observer_->GetError())
  {
    ROS_ERROR_STREAM("Bezier::removeIsolatedTrianglesFilter: " << vtk_observer_->GetErrorMessage());
    vtk_observer_->Clear();
    return false;
  }

  unsigned regions_number = connectivityFilter->GetNumberOfExtractedRegions();
  if (regions_number == 0)
  {
    ROS_ERROR_STREAM("Bezier::removeIsolatedTrianglesFilter: This mesh is empty !");
    return false;
  }

  connectivityFilter->SetExtractionModeToSpecifiedRegions();
  connectivityFilter->InitializeSpecifiedRegionList();

  vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
  unsigned number_of_regions_added(0);
  for (unsigned index(0); index < regions_number; index++)
  {
    connectivityFilter->AddSpecifiedRegion(index);
    vtk_observer_->Clear();
    vtkSmartPointer<vtkPolyData> region = vtkSmartPointer<vtkPolyData>::New();
    connectivityFilter->SetOutput(region);
    connectivityFilter->Update();

    if (vtk_observer_->GetWarning())
    {
      ROS_WARN_STREAM("Bezier::removeIsolatedTrianglesFilter: " << vtk_observer_->GetWarningMessage());
      vtk_observer_->Clear();
      return false;
    }
    if (vtk_observer_->GetError())
    {
      ROS_ERROR_STREAM("Bezier::removeIsolatedTrianglesFilter: " << vtk_observer_->GetErrorMessage());
      vtk_observer_->Clear();
      return false;
    }

    if (region->GetNumberOfCells() > minimal_number_of_cells)
    {
      number_of_regions_added++;
      appendFilter->AddInputData(region);
    }
    connectivityFilter->DeleteSpecifiedRegion(index);
  }

  if (number_of_regions_added == 0)
  {
    ROS_ERROR_STREAM("Bezier::removeIsolatedTrianglesFilter: The output polydata is empty !");
    return false;
  }
  appendFilter->SetOutput(polydata);
  appendFilter->AddObserver(vtkCommand::ErrorEvent, vtk_observer_);
  appendFilter->AddObserver(vtkCommand::WarningEvent, vtk_observer_);
  vtk_observer_->Clear();
  appendFilter->Update();

  if (vtk_observer_->GetWarning())
  {
    ROS_WARN_STREAM("Bezier::removeIsolatedTrianglesFilter: " << vtk_observer_->GetWarningMessage());
    vtk_observer_->Clear();
    return false;
  }
  if (vtk_observer_->GetError())
  {
    ROS_ERROR_STREAM("Bezier::removeIsolatedTrianglesFilter: " << vtk_observer_->GetErrorMessage());
    vtk_observer_->Clear();
    return false;
  }

  return true;
}

/**
 * Get an rviz_visual_tool color given an index
 * @param[in] index of the color
 * @returns a color enum from rviz_visual_tools corresponding to the index
 */
rviz_visual_tools::colors Bezier::visualToolsColorFromIndex(const unsigned index)
{
  switch (index)
  {
    case 0:
      return rviz_visual_tools::BLUE;
      break;
    case 1:
      return rviz_visual_tools::GREEN;
      break;
    case 2:
      return rviz_visual_tools::RED;
      break;
    case 3:
      return rviz_visual_tools::CYAN;
      break;
    case 4:
      return rviz_visual_tools::MAGENTA;
      break;
    case 5:
      return rviz_visual_tools::BROWN;
      break;
    case 6:
      return rviz_visual_tools::LIME_GREEN;
      break;
    case 7:
      return rviz_visual_tools::ORANGE;
      break;
    case 8:
      return rviz_visual_tools::PINK;
      break;
    case 9:
      return rviz_visual_tools::YELLOW;
      break;
    case 10:
      return rviz_visual_tools::BLACK;
      break;
    case 11:
      return rviz_visual_tools::PURPLE;
      break;
    default:
      return rviz_visual_tools::GREY;
      break;
  }
}
