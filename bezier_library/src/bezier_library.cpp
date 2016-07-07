#include <bezier_library/bezier_library.hpp>

Bezier::Bezier()
{
  vtk_observer_ = vtkSmartPointer<ErrorObserver>::New();
  ROS_INFO_STREAM("Bezier::Bezier: RViz visualization tool is initialized in 'base' "
                  "and the topic name is 'rviz_visual_tools'");
  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link"));
  visual_tools_->enableBatchPublishing(); // Call triggerBatchPublish() to publish everything
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
      visual_tools_->publishXArrow(tmp, color, rviz_visual_tools::XXXXSMALL, 0.008);
      visual_tools_->publishZArrow(tmp, color, rviz_visual_tools::XXXXSMALL, 0.008);
      if (display_labels)
      {
        tmp.translation() -= 0.01 * tmp.affine().col(2).head<3>();
        visual_tools_->publishText(tmp, boost::lexical_cast<std::string>(index++), color, rviz_visual_tools::SMALL, false);
      }
    }
  }
  visual_tools_->triggerBatchPublish();
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
  visual_tools_->triggerBatchPublish();
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
  estimateGlobalMeshNormal(polydata, mesh_normal);
  orientation = Eigen::Vector3d(mesh_normal[2], 0, -mesh_normal[0]);
  orientation.normalize();

  if (mesh_normal.dot(orientation) > 1e-10) // Numerical issues
  {
    ROS_ERROR_STREAM("Bezier::estimateSlicingOrientation: Scalar product is not 0! " << mesh_normal.dot(orientation));
    return false;
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
  visual_tools_->triggerBatchPublish();
  return true;
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

bool Bezier::dilate(vtkSmartPointer<vtkPolyData> &polydata,
                    const double radius)
{
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
  implicit_modeller->SetSampleDimensions(50, 50, 50);
  implicit_modeller->SetInputData(polydata);
  implicit_modeller->AdjustBoundsOn();
  implicit_modeller->SetAdjustDistance(threshold);
  if (2 * threshold > 1.0)
    implicit_modeller->SetMaximumDistance(1.0);
  else
    implicit_modeller->SetMaximumDistance(2 * threshold); // 2*threshold in order to be sure -> long time but smoothed dilation

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
