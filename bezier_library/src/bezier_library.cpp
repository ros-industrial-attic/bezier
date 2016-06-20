#include "bezier_library/bezier_library.hpp"

Bezier::Bezier() :
        lean_angle_axis_(std::string()),
        angle_value_(0.0),
        maximum_depth_of_path_(0.05),
        working_line_width_(0.02),
        covering_percentage_(0.50),
        extrication_coefficient_(0.5),
        extrication_frequency_(1),
        mesh_normal_vector_(Eigen::Vector3d::Identity()),
        slicing_dir_(Eigen::Vector3d::Identity()),
        number_of_normal_markers_published_(0),
        use_translation_mode_ (false)
{
  inputPolyData_ = vtkSmartPointer<vtkPolyData>::New();
  defectPolyData_ = vtkSmartPointer<vtkPolyData>::New();
  ROS_WARN_STREAM("Bezier::Bezier: Default constructor called, RViz visualization tool is initialized in 'base_link' "
                "and the topic name is 'rviz_visual_tools'");
  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_link", "rviz_visual_tools"));
  visual_tools_->setLifetime(0);
  visual_tools_->enableBatchPublishing();
}

Bezier::Bezier(const std::string filename_inputMesh,
               const std::string filename_defectMesh,
               const std::string rviz_fixed_frame,
               const std::string rviz_topic_name,
               const std::string lean_angle_axis,
               const double angle_value,
               const double maximum_depth_of_path,
               const double working_line_width,
               const double covering_percentage,
               const int extrication_coefficient,
               const int extrication_frequency,
               const bool surfacing_mode,
               const bool use_translation_mode) :
    lean_angle_axis_(lean_angle_axis),
    angle_value_(angle_value),
    maximum_depth_of_path_(maximum_depth_of_path),
    working_line_width_(working_line_width),
    covering_percentage_(covering_percentage),
    extrication_coefficient_(extrication_coefficient),
    extrication_frequency_(extrication_frequency),
    mesh_normal_vector_(Eigen::Vector3d::Identity()),
    slicing_dir_(Eigen::Vector3d::Identity()),
    number_of_normal_markers_published_(0),
    surfacing_mode_(surfacing_mode),
    use_translation_mode_(use_translation_mode)
{
  inputPolyData_ = vtkSmartPointer<vtkPolyData>::New();
  if (!loadPLYPolydata(filename_inputMesh, inputPolyData_))
    ROS_ERROR_STREAM("Bezier::Bezier: Can't load input mesh: " << filename_inputMesh);

  if(!surfacing_mode_)
  {
    //No need to load the defect polyData when using surface mode
    defectPolyData_ = vtkSmartPointer<vtkPolyData>::New();
    if (!loadPLYPolydata(filename_defectMesh, defectPolyData_))
      ROS_ERROR_STREAM("Bezier::Bezier: Can't load defect mesh: " << filename_defectMesh);
  }

  visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(rviz_fixed_frame, rviz_topic_name));
  visual_tools_->setLifetime(0);
  visual_tools_->enableBatchPublishing();

}

Bezier::~Bezier()
{
}

/** @brief Structure used to reorder lines (strippers) */
struct lineOrganizerStruct
{
  // Sort struct have to know its containing objects
  Bezier* bezier_object;
  lineOrganizerStruct(Bezier* bezier_object2) :
      bezier_object(bezier_object2)
  {
  }
  ;

  // This is our sort function: use dot products to determine line position
  bool operator()(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > &line_a,
                  const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > &line_b)
  {
    Eigen::Vector3d vector_dir = bezier_object->getSlicingDirection();
    float dist_a = vector_dir.dot(line_a[0].first);
    float dist_b = vector_dir.dot(line_b[0].first);
    return dist_a < dist_b;
  }
};

void Bezier::printBezierParameters(void)
{
  ROS_INFO_STREAM("Bezier parameters" << std::endl <<
                  "maximum_depth_of_path (in centimeters) : " << maximum_depth_of_path_*100 << std::endl <<
                  "Effector diameter (in centimeters) : " << working_line_width_*100 << std::endl <<
                  "covering_percentage (in %) : "<< covering_percentage_*100 << "/100");
}

void Bezier::setTranslationMode(const bool translation_mode)
{
  use_translation_mode_ = translation_mode;
}

bool Bezier::getTranslationMode()
{
  return use_translation_mode_;
}

bool Bezier::getSurfacing() const
{
  return surfacing_mode_;
}

//////////////////// PRIVATE FUNCTIONS ////////////////////
Eigen::Vector3d Bezier::getSlicingDirection()
{
  return slicing_dir_;
}

bool Bezier::loadPLYPolydata(const std::string meshname,
                             vtkSmartPointer<vtkPolyData> &poly_data)
{
  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName(meshname.c_str());
  reader->Update();
  poly_data = reader->GetOutput();
  return true;
}

bool Bezier::savePLYPolyData(const std::string meshname,
                             const vtkSmartPointer<vtkPolyData> poly_data)
{
  vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
  plyWriter->SetFileName(meshname.c_str());
  plyWriter->SetInputData(poly_data);
  plyWriter->Update();
  if (!plyWriter->Write())
    return false;
  return true;
}

// FIXME Dilation problem : when depth is to high, dilated mesh has unexpected holes
// These holes are problematic. In fact, when cutting process is called on dilated mesh, slices are divided in some parts due to these holes
// and this affects the path generation, especially for extrication trajectory. We have to find a solution, perhaps find best parameters
// in order to resolve this problem.
bool Bezier::dilation(const double depth, vtkSmartPointer<vtkPolyData> &dilated_polydata)
{
  // Get maximum length of the sides
  double bounds[6];
  inputPolyData_->GetBounds(bounds);
  double max_side_length = std::max(bounds[1] - bounds[0], bounds[3] - bounds[2]);
  max_side_length = std::max(max_side_length, bounds[5] - bounds[4]);
  double threshold = depth / max_side_length;

  // Dilation
  vtkSmartPointer<vtkImplicitModeller> implicitModeller = vtkSmartPointer<vtkImplicitModeller>::New();
  implicitModeller->SetProcessModeToPerVoxel();  // Optimize process -> per voxel and not per cell
  implicitModeller->SetSampleDimensions(50, 50, 50);
#if VTK_MAJOR_VERSION <= 5
  implicitModeller->SetInput(inputPolyData_);
#else
  implicitModeller->SetInputData(inputPolyData_);
#endif
  implicitModeller->AdjustBoundsOn();
  implicitModeller->SetAdjustDistance(threshold);  // Adjust by 10%
  if (2 * threshold > 1.0)
    implicitModeller->SetMaximumDistance(1.0);
  else
    implicitModeller->SetMaximumDistance(2 * threshold); // 2*threshold in order to be sure -> long time but smoothed dilation
  implicitModeller->ComputeModelBounds(inputPolyData_);
  implicitModeller->Update();

  vtkSmartPointer<vtkMarchingCubes> surface = vtkSmartPointer<vtkMarchingCubes>::New();
  surface->SetInputConnection(implicitModeller->GetOutputPort());
  surface->ComputeNormalsOn();
  surface->SetValue(0, depth);
  surface->Update();
  vtkSmartPointer<vtkPolyData> dilated_tmp = vtkSmartPointer<vtkPolyData>::New();
  dilated_tmp = surface->GetOutput();

  // Keep only the upper part of the dilation:
  // morphological dilation is usually used on volumes, not on surfaces so we have to adapt result  // Build a Kdtree

  // Build a Kdtree
  vtkSmartPointer<vtkKdTreePointLocator> kDTree = vtkSmartPointer<vtkKdTreePointLocator>::New();
  kDTree->SetDataSet(inputPolyData_);
  kDTree->BuildLocator();
  //Build cell and link in dilated_poly_data
  dilated_tmp->BuildCells();
  dilated_tmp->BuildLinks();
  // Get normal tab
  // if there are no normals in poly data, they are computed
  if (inputPolyData_->GetPointData()->GetNormals() == 0)
  {
    ROS_WARN_STREAM("Bezier::dilation: No normals in inputPolyData_. Computing normals...");
    generatePointNormals(inputPolyData_);
  }

  vtkFloatArray *PointNormalArray = vtkFloatArray::SafeDownCast(inputPolyData_->GetPointData()->GetNormals());
  if (!PointNormalArray)
    return false;
  // For each cell in dilated_polydata
  for (vtkIdType index_cell = 0; index_cell < (dilated_tmp->GetNumberOfCells()); index_cell++)
  {
    // Get cell
    vtkCell* cell = dilated_tmp->GetCell(index_cell);
    // Get center of cell
    double pcoords[3] = {0, 0, 0};
    double *weights = new double[dilated_tmp->GetMaxCellSize()];
    int subId = cell->GetParametricCenter(pcoords);
    double cellCenter[3] = {0, 0, 0};
    cell->EvaluateLocation(subId, pcoords, cellCenter, weights);
    // Get closest point (in input_polydata)
    vtkIdType iD = kDTree->FindClosestPoint(cellCenter);
    double closestPoint[3];
    inputPolyData_->GetPoint(iD, closestPoint);
    // Get direction vector
    Eigen::Vector3d direction_vector = Eigen::Vector3d(cellCenter[0] - closestPoint[0],
                                                       cellCenter[1] - closestPoint[1],
                                                       cellCenter[2] - closestPoint[2]);

    // Get closest point normal
    double normal[3];
    PointNormalArray->GetTuple(iD, normal);
    Eigen::Vector3d normal_vector(normal[0], normal[1], normal[2]);
    direction_vector.normalize();
    normal_vector.normalize();

    // Keep the cell only if it belongs to the upper part of the mesh
    if (!vtkMath::IsFinite(cellCenter[0]) ||
        !vtkMath::IsFinite(cellCenter[1]) ||
        !vtkMath::IsFinite(cellCenter[2])
        || normal_vector.dot(direction_vector) <= 0)
      dilated_tmp->DeleteCell(index_cell);
  }

  dilated_tmp->RemoveDeletedCells();
  if (dilated_tmp->GetNumberOfCells() == 0)
    return false;

  vtkSmartPointer<vtkReverseSense> mesh_reverser = vtkSmartPointer<vtkReverseSense>::New();
  mesh_reverser->SetInputData(dilated_tmp);
  mesh_reverser->SetOutput(dilated_polydata);
  mesh_reverser->Update();
  return true;
}

bool Bezier::translation(const double depth,
                         const vtkSmartPointer<vtkPolyData> poly_data,
                         vtkSmartPointer<vtkPolyData> &translation_poly_data)
{
  vtkSmartPointer<vtkTransform> translation = vtkSmartPointer<vtkTransform>::New();
  Eigen::Vector3d translate_vector = depth * mesh_normal_vector_;
  double translate_tab[3] = {translate_vector[0], translate_vector[1], translate_vector[2]};
  translation->Translate(translate_tab[0], translate_tab[1], translate_tab[2]);
  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetInputData(poly_data);
  transformFilter->SetTransform(translation);
  transformFilter->Update();
  translation_poly_data = transformFilter->GetOutput();
  //check depth
  vtkSmartPointer<vtkDistancePolyDataFilter> distanceFilter = vtkSmartPointer<vtkDistancePolyDataFilter>::New();
  distanceFilter->SetInputData(1, poly_data);
  distanceFilter->SetInputData(0, translation_poly_data);
  distanceFilter->Update();
  double distance = depth - distanceFilter->GetOutput()->GetPointData()->GetScalars()->GetRange()[0];
  if (distance <= 0)
  {
    ROS_ERROR_STREAM("Bezier::translation: Wrong translation! " << distance << " <= 0");
    return false;
  }
  return true;
}

bool Bezier::defectIntersectionOptimisation(vtkSmartPointer<vtkPolyData> &poly_data)
{
  bool intersection_flag = false;
  // Build a Kdtree on defect
  vtkSmartPointer<vtkKdTreePointLocator> kDTreeDefect = vtkSmartPointer<vtkKdTreePointLocator>::New();
  kDTreeDefect->SetDataSet(defectPolyData_);
  kDTreeDefect->BuildLocator();

  // if there are no normals in poly data, they are computed
  if (defectPolyData_->GetPointData()->GetNormals() == 0)
  {
    ROS_WARN_STREAM("Bezier::defectIntersectionOptimisation: No normals in inputPolyData_. Computing normals...");
    generatePointNormals(defectPolyData_);
  }

  vtkFloatArray *defectPointNormalArray = vtkFloatArray::SafeDownCast(defectPolyData_->GetPointData()->GetNormals());
  // For each cell in dilate polydata
  for (vtkIdType index_cell = 0; index_cell < (poly_data->GetNumberOfCells()); index_cell++)
  {
    // Get cell
    vtkCell* cell = poly_data->GetCell(index_cell);
    // Get points of cell
    vtkPoints * pts = cell->GetPoints();
    // Variable test use to know points position
    bool inside = false;
    for (int index_pt = 0; index_pt < pts->GetNumberOfPoints(); index_pt++)
    {
      // Get point
      double pt[3];
      pts->GetPoint(index_pt, pt);
      // Get closest point (in defautPolyData)
      vtkIdType iD = kDTreeDefect->FindClosestPoint(pt);
      double closestPoint[3];
      defectPolyData_->GetPoint(iD, closestPoint);
      // Get direction vector
      Eigen::Vector3d direction_vector = Eigen::Vector3d(closestPoint[0] - pt[0],
                                                         closestPoint[1] - pt[1],
                                                         closestPoint[2] - pt[2]);
      // Get closest point normal
      double normal[3];
      defectPointNormalArray->GetTuple(iD, normal);
      Eigen::Vector3d normal_vector(normal[0], normal[1], normal[2]);
      // Normalize vectors
      direction_vector.normalize();
      normal_vector.normalize();
      // Test in order to save or remove cell
      if (vtkMath::IsFinite(pt[0]) &&
          vtkMath::IsFinite(pt[1]) &&
          vtkMath::IsFinite(pt[2])
          && normal_vector.dot(direction_vector) > 0.1) //FIXME threshold use for corner (90° angle) : It resolves problem with neighbor and dot product
      {
        inside = true;
        intersection_flag = true;
        break;
      }
    }
    if (!inside)
      poly_data->DeleteCell(index_cell);

  }
  // Remove all deleted cells (outside cells)
  poly_data->RemoveDeletedCells();
  return intersection_flag;
}

bool Bezier::generateCellNormals(vtkSmartPointer<vtkPolyData> &poly_data)
{
  vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
  normals->SetInputData(poly_data);
  normals->ComputeCellNormalsOn();
  normals->ComputePointNormalsOff();
  normals->ConsistencyOn();
  normals->AutoOrientNormalsOn();
  normals->Update();
  poly_data = normals->GetOutput();
  vtkFloatArray* CellNormalArray = vtkFloatArray::SafeDownCast(poly_data->GetCellData()->GetArray("Normals"));

  if (!CellNormalArray)
    return false;

  return true;
}

bool Bezier::generatePointNormals(vtkSmartPointer<vtkPolyData> &poly_data)
{
  vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New();
  normals->SetInputData(poly_data);
  normals->ComputeCellNormalsOff();
  normals->ComputePointNormalsOn();
  normals->Update();
  poly_data = normals->GetOutput();
  vtkFloatArray *PointNormalArray = vtkFloatArray::SafeDownCast(poly_data->GetPointData()->GetNormals());

  if (!PointNormalArray)
    return false;

  return true;
}

void Bezier::ransacPlaneSegmentation()
{
  //Get polydata point cloud (PCL)
  pcl::PolygonMesh mesh;
  pcl::VTKUtils::vtk2mesh(inputPolyData_, mesh);
  PointCloudT::Ptr input_cloud(new PointCloudT);
  pcl::fromPCLPointCloud2(mesh.cloud, *input_cloud);

  // Get polydata dimensions
  double x_size = inputPolyData_->GetBounds()[1] - inputPolyData_->GetBounds()[0];
  double y_size = inputPolyData_->GetBounds()[3] - inputPolyData_->GetBounds()[2];
  double z_size = inputPolyData_->GetBounds()[5] - inputPolyData_->GetBounds()[4];

  // Apply RANSAC
  pcl::SACSegmentation<PointT> seg;
  pcl::ModelCoefficients model_coefficients;
  seg.setInputCloud(input_cloud);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  model_coefficients.values.resize(3);  //Plane
  //@note : Threshold = max(x_size,y_size,z_size) cos plane model has to fit all points of inputPolyData
  double threshold = std::max(x_size, y_size);
  threshold = std::max(threshold, z_size);
  seg.setDistanceThreshold(threshold);
  seg.setMaxIterations(2000);  //@note 2000 (high value) has been set in order to get good result
  pcl::PointIndices inliers;
  seg.segment(inliers, model_coefficients);

  // Set Mesh normal vector
  mesh_normal_vector_ = Eigen::Vector3d(model_coefficients.values[0],
                                        model_coefficients.values[1],
                                        model_coefficients.values[2]);
}

void Bezier::generateSlicingDirection()
{
  // Find two simple orthogonal vectors to mesh_normal
  Eigen::Vector3d x_vector = Eigen::Vector3d(mesh_normal_vector_[2], 0, -mesh_normal_vector_[0]);
  //Eigen::Vector3d y_vector = Eigen::Vector3d(0, mesh_normal_vector_[2], -mesh_normal_vector_[1]);
  // By default, we chose x_vector. But another vector in this plan could be chosen.
  x_vector.normalize();
  slicing_dir_ = x_vector;
}

unsigned int Bezier::getRealSliceNumber(vtkSmartPointer<vtkStripper> stripper, const Eigen::Vector3d &vector_dir)
{
  //vtkIdType numberOfLines = stripper->GetOutput()->GetNumberOfLines();
  vtkPoints *points = stripper->GetOutput()->GetPoints();
  vtkCellArray *cells = stripper->GetOutput()->GetLines();
  vtkIdType *indices;
  vtkIdType numberOfPoints;
  unsigned int lineCount = 0;
  std::vector<double> dot_vector;
  // For all lines
  for (cells->InitTraversal(); cells->GetNextCell(numberOfPoints, indices); lineCount++)
  {
    double point[3];
    int indice = ceil(numberOfPoints / 2);  // Get a point in line : we have chosen center point
    if (numberOfPoints <= 1)  // Yet, if number of points is to small, get 0 as indice.
      indice = 0;
    // Get point
    points->GetPoint(indices[indice], point);
    // Scalar product between vector_dir and point[0] vector
    Eigen::Vector3d point_vector(point[0], point[1], point[2]);
    double dot = vector_dir.dot(point_vector);
    // For each scalar products, push back to a vector
    dot_vector.push_back(dot);
  }
  if (dot_vector.size() == 0)  // If vector size is null return 0
    return 0;
  // Else sort vector
  std::sort(dot_vector.begin(), dot_vector.end());
  // Remove duplicated value or too close values
  unsigned int index = 0;
  double value = (working_line_width_ * (1 - covering_percentage_)) / (2 * 10); //2 to get radius, 10 to get 10% of virtual radius as threshold
  while (index < (dot_vector.size() - 1))
  {
    if (std::abs(dot_vector[index] - dot_vector[index + 1]) < value)
    {
      dot_vector.erase(dot_vector.begin() + index + 1);
    }
    // Witout threshold :
    /*if(dot_vector[index] == dot_vector[index + 1])
     dot_vector.erase(dot_vector.begin() + index + 1);*/
    else
      index++;
  }
  // Return size of vector (equal real number of lines)
  return dot_vector.size();
}

bool Bezier::cutMesh(const vtkSmartPointer<vtkPolyData> poly_data,
                     const Eigen::Vector3d &cut_dir,
                     const unsigned int line_number_expected,
                     vtkSmartPointer<vtkStripper> &stripper)
{
  // Get info about polyData : center point & bounds
  double minBound[3];
  minBound[0] = poly_data->GetBounds()[0];
  minBound[1] = poly_data->GetBounds()[2];
  minBound[2] = poly_data->GetBounds()[4];

  double maxBound[3];
  maxBound[0] = poly_data->GetBounds()[1];
  maxBound[1] = poly_data->GetBounds()[3];
  maxBound[2] = poly_data->GetBounds()[5];

  double distanceMinMax = sqrt(vtkMath::Distance2BetweenPoints(minBound, maxBound));

  double width = working_line_width_ * (1 - covering_percentage_);
  double line_number = std::floor((distanceMinMax) / width);

  // Create a plane to cut mesh
  vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
  plane->SetOrigin(poly_data->GetCenter());
  plane->SetNormal(cut_dir.normalized()[0], cut_dir.normalized()[1], cut_dir.normalized()[2]);
  // Create a cutter
  vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
  cutter->SetCutFunction(plane);
#if VTK_MAJOR_VERSION <= 5
  cutter->SetInput(poly_data);
#else
  cutter->SetInputData(poly_data);
#endif
  cutter->Update();

  // In the case where line_number_expected is 1, the call of the function SetValue is automatically done
  // at the creation of the cutter, so we don't have to call it again, because it will create an other
  // cut which is not expected
  if (line_number_expected == 1)
  {
    line_number = line_number_expected;
  }
  // This loop will create cuts from center to maximum bound of poly_data, with width like offset between two cuts
  // and will do the same from center to minimum bound of poly_data
  else
  {
    for (unsigned int i = 0; i < (line_number / 2); ++i)
    {
      cutter->SetValue(i + 1, width * (i + 1));
      cutter->SetValue(i + 1 + line_number / 2, -width * (i + 1));
    }
  }

  cutter->Update();
  // VTK triangle filter used for stripper
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
  triangleFilter->SetInputConnection(cutter->GetOutputPort());
  triangleFilter->Update();
  // VTK Stripper used to generate polylines from cutter
  stripper->SetInputConnection(triangleFilter->GetOutputPort());
  stripper->Update();
  return true;
}

bool Bezier::generateRobotPoses(const Eigen::Vector3d &point,
                                const Eigen::Vector3d &point_next,
                                const Eigen::Vector3d &normal,
                                Eigen::Affine3d &pose)
{
  Eigen::Vector3d normal_x;
  Eigen::Vector3d normal_y;
  Eigen::Vector3d normal_z(normal);
  normal_x = point_next - point;  // Next point direction
  if (normal_x == Eigen::Vector3d::Zero())
  {
    ROS_ERROR_STREAM("Bezier::generateRobotPoses: X normal = 0, mesh is too dense or there are duplicate points in the line!");
    return false;
  }
  normal_y = normal_z.cross(normal_x);

  normal_x.normalize();
  normal_y.normalize();
  normal_z.normalize();

  // Check if pose has NAN values
  if (!vtkMath::IsFinite((float)normal_y[0]) ||
      !vtkMath::IsFinite((float)normal_y[1]) ||
      !vtkMath::IsFinite((float)normal_y[2]))
    return false;
  // Else generate matrix (pose)
  // Translation
  pose.translation() << point;
  // Rotation
  pose.linear().col(0) << normal_x;
  pose.linear().col(1) << normal_y;
  pose.linear().col(2) << normal_z;

  return (applyLeanAngle(pose, lean_angle_axis_, angle_value_));
}

void Bezier::harmonizeLinesOrientation(std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > &lines)
{
  // Get vector reference
  Eigen::Vector3d reference = slicing_dir_.cross(mesh_normal_vector_);
  reference.normalize();

  // Compare orientation of lines with reference
  for (unsigned int line_index = 0; line_index < lines.size(); line_index++)
  {
    // Get line orientation
    int point_number = lines[line_index].size();
    Eigen::Vector3d vector_orientation = lines[line_index][point_number - 1].first - lines[line_index][0].first;
    vector_orientation.normalize();
    // Compare (Check orientation)
    if (reference.dot(vector_orientation) < 0)  // Dot product<0 so, vectors have opposite orientation
      std::reverse(lines[line_index].begin(), lines[line_index].end());  // Change orientation of line
  }
}

void Bezier::removeNearNeighborPoints(std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > &lines)
{
  // For each points
  for (unsigned int index_line = 0; index_line < lines.size(); index_line++)
  {
    for (unsigned int index_point = 0; index_point < (lines[index_line].size() - 1); index_point++)
    {
      Eigen::Vector3d point_vector = lines[index_line][index_point].first;  // Get point position
      double point[3] = {point_vector[0], point_vector[1], point_vector[2]};

      Eigen::Vector3d next_point_vector = lines[index_line][index_point + 1].first;  // Get next point position
      double next_point[3] = {next_point_vector[0], next_point_vector[1], next_point_vector[2]};

      if (sqrt(vtkMath::Distance2BetweenPoints(point, next_point)) < 0.001)
      {  // FIXME Arbitrary value
        if (index_point < (lines[index_line].size() - 2))
          lines[index_line].erase(lines[index_line].begin() + index_point + 1);
        else
          lines[index_line].erase(lines[index_line].begin() + index_point);
      }
    }
  }
}

bool Bezier::generateStripperOnSurface(const vtkSmartPointer<vtkPolyData> PolyData,
                                       std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > &lines)
{
  // Cut mesh
  vtkSmartPointer<vtkStripper> stripper = vtkSmartPointer<vtkStripper>::New();
  stripper->SetJoinContiguousSegments(true);
  cutMesh(PolyData, slicing_dir_, 0, stripper);  // Fill this stripper with cutMesh function

  // FIXME At the moment we use vector to reorganize data from stripper
  //       In the future, we must change that in order to use stripper only

  // Get data from stripper
  //vtkIdType numberOfLines = stripper->GetOutput()->GetNumberOfLines();
  vtkPoints *points = stripper->GetOutput()->GetPoints();
  vtkCellArray *cells = stripper->GetOutput()->GetLines();
  vtkIdType *indices;
  vtkIdType numberOfPoints;
  unsigned int lineCount = 0;

  // Get normal array of stripper
  vtkFloatArray *PointNormalArray = vtkFloatArray::SafeDownCast(stripper->GetOutput()->GetPointData()->GetNormals());
  if (!PointNormalArray)
    return false;
  for (cells->InitTraversal(); cells->GetNextCell(numberOfPoints, indices); lineCount++)  // For each line : iterator
  {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > line; // line.first : point position, line.second : point normal
    for (vtkIdType i = 0; i < numberOfPoints; i++)
    {
      //Get points
      double point[3];
      points->GetPoint(indices[i], point);
      Eigen::Vector3d point_vector(point[0], point[1], point[2]);

      //Get Z normal
      double normal[3];
      PointNormalArray->GetTuple(indices[i], normal);
      Eigen::Vector3d normal_vector(normal[0], normal[1], normal[2]);

      if (use_translation_mode_)
        normal_vector *= -1;
      else
      {
        if (PolyData == inputPolyData_)
          normal_vector *= -1;
      }
      line.push_back(std::make_pair(point_vector, normal_vector));
    }
    lines.push_back(line);
  }
  // Sort vector : re order lines
  std::sort(lines.begin(), lines.end(), lineOrganizerStruct(this));
  // Check line orientation
  harmonizeLinesOrientation(lines);
  // Remove too closed points
  removeNearNeighborPoints(lines);
  return true;
}

int Bezier::seekClosestLine(
    const Eigen::Vector3d &point_vector,
    const std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > &extrication_lines)
{
  int index_of_closest_line(0);
  double distance(std::numeric_limits<double>::max());
  for (size_t index_line = 0; index_line < extrication_lines.size(); index_line++)
  {
    double number_of_point = extrication_lines[index_line].size();
    Eigen::Vector3d end_extrication_point_vector = extrication_lines[index_line][number_of_point - 1].first;
    //get distance
    double point[3] = {point_vector[0], point_vector[1], point_vector[2]};
    double extrication_point[3] = {end_extrication_point_vector[0],
                                   end_extrication_point_vector[1],
                                   end_extrication_point_vector[2]};
    if (distance > vtkMath::Distance2BetweenPoints(point, extrication_point))
    {
      distance = vtkMath::Distance2BetweenPoints(point, extrication_point);
      index_of_closest_line = index_line;
    }
  }
  return index_of_closest_line;
}

// FIXME Combine seekClosestPoint and seekClosestExtricationPassPoint in one function
int Bezier::seekClosestPoint(const Eigen::Vector3d &point_vector,
                             const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > &extrication_line)
{
  int index(0);
  double distance(std::numeric_limits<double>::max());
  for (size_t index_point = 0; index_point < extrication_line.size(); index_point++)
  {
    Eigen::Vector3d extrication_point_vector = extrication_line[index_point].first;
    double point[3] = {point_vector[0], point_vector[1], point_vector[2]};
    double extrication_point[3] =
        {extrication_point_vector[0], extrication_point_vector[1], extrication_point_vector[2]};
    if (distance > vtkMath::Distance2BetweenPoints(point, extrication_point))
    {
      distance = vtkMath::Distance2BetweenPoints(point, extrication_point);
      index = index_point;
    }
  }
  return index;
}

int Bezier::seekClosestExtricationPassPoint(
    const Eigen::Vector3d &point_vector,
    const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &extrication_poses)
{
  int index(0);
  double distance(std::numeric_limits<double>::max());
  for (size_t index_point = 0; index_point < extrication_poses.size(); index_point++)
  {
    Eigen::Vector3d closest_point_vector = extrication_poses[index_point].translation();
    double point[3] = {point_vector[0], point_vector[1], point_vector[2]};
    double closest_point[3] = {closest_point_vector[0], closest_point_vector[1], closest_point_vector[2]};
    if (distance > vtkMath::Distance2BetweenPoints(point, closest_point))
    {
      distance = vtkMath::Distance2BetweenPoints(point, closest_point);
      index = index_point;
    }
  }
  return index;
}

bool Bezier::saveDilatedMeshes(const std::string path)
{
  if (dilationPolyDataVector_.empty())
    return false;

  for (size_t i = 0; i < dilationPolyDataVector_.size(); ++i)
  {
    std::string number(boost::lexical_cast<std::string>(i));
    std::string file(path + "/mesh_" + number + ".ply");
    if (!savePLYPolyData(file, dilationPolyDataVector_[i]))
      return false;
    ROS_INFO_STREAM("Bezier::saveDilatedMeshes: " << file << " saved successfully");
  }
  return true;
}

bool Bezier::applyLeanAngle(Eigen::Affine3d &pose,
                            const std::string lean_angle_axis,
                            const double angle_value)
{
  // Check if axis of rotation given is known
  if (lean_angle_axis != "x" && lean_angle_axis != "y" && lean_angle_axis != "z")
  {
    ROS_ERROR_STREAM("Bezier::applyLeanAngle: Lean angle direction is not correct. "
                     "Trajectory will be computed with no angle");
    return false;
  }

  // Will provide effector angle with axis of rotation and angle value (in radians).
  if (lean_angle_axis == "x")
    pose.rotate(Eigen::AngleAxisd(angle_value, Eigen::Vector3d::UnitX()));
  else if (lean_angle_axis == "y")
    pose.rotate(Eigen::AngleAxisd(angle_value, Eigen::Vector3d::UnitY()));
  else
    pose.rotate(Eigen::AngleAxisd(angle_value, Eigen::Vector3d::UnitZ()));

  return true;
}

//////////////////// PUBLIC MEMBERS ////////////////////
bool Bezier::generateTrajectory(
    std::vector<Eigen::Affine3d,
    Eigen::aligned_allocator<Eigen::Affine3d> > &way_points_vector,
    std::vector<bool> &color_vector,
    std::vector<int> &index_vector)
{
  way_points_vector.clear();
  color_vector.clear();
  generatePointNormals(inputPolyData_);
  // Generate global mesh normal
  ransacPlaneSegmentation();
  // Find cut direction thanks to the mesh normal
  generateSlicingDirection();
  // Dilate mesh to generate different passes
  if (use_translation_mode_)
    ROS_INFO_STREAM("Bezier::generateTrajectory: Start translation");
  else
  {
    if (surfacing_mode_)
      ROS_INFO_STREAM("Bezier::generateTrajectory: Start surfacing");
    else
      ROS_INFO_STREAM("Bezier::generateTrajectory: Start dilation");
  }
  bool intersection_flag = true;  // Flag variable : dilate while dilated mesh intersect defect mesh
  double depth = 0;  // Depth between input mesh and dilated mesh
  bool surfacing_mode_interrupt = false;
  while (intersection_flag && !surfacing_mode_interrupt)
  {
    vtkSmartPointer<vtkPolyData> expanded_polydata = vtkSmartPointer<vtkPolyData>::New();
    bool flag_expansion(false);
    if (use_translation_mode_)
      flag_expansion = translation(depth, inputPolyData_, expanded_polydata);
    else
    {
      if (surfacing_mode_)
      {
        // We Interrupt the while loop after the first pass in order to make only a surface grinding
        // In this case, the dilation will not be perform. The if(depth == 0) condition is executed before
        // exiting the loop
        surfacing_mode_interrupt = true;
      }

      // Add inputPolyData_ as first expanded_polydata, in order to compute intersection between input and defect
      if (depth == 0)
      {
        expanded_polydata->DeepCopy(inputPolyData_);
        // if expanded_polydata have not normals, they are computed
        if (expanded_polydata->GetPointData()->GetNormals() == 0)
        {
          generatePointNormals(expanded_polydata);
        }
        // Reverse normals sense
        vtkSmartPointer<vtkReverseSense> normalReverser = vtkSmartPointer<vtkReverseSense>::New();
        normalReverser->SetInputData(expanded_polydata);
        normalReverser->ReverseNormalsOn();
        normalReverser->Update();
        expanded_polydata = normalReverser->GetOutput();
        flag_expansion = true;
      }
      else
      {
        flag_expansion = dilation(depth, expanded_polydata);
      }
    }
    vtkSmartPointer<vtkPolyData> temp_polydata = vtkSmartPointer<vtkPolyData>::New(); //Ignore collision in translation process fixme
    temp_polydata->ShallowCopy(expanded_polydata);
    if (!surfacing_mode_)
    {
      if (flag_expansion && defectIntersectionOptimisation(expanded_polydata)
          && expanded_polydata->GetNumberOfCells() > 10) // FIXME Check intersection between new dilated mesh and defect
      {
        if (use_translation_mode_)
          dilationPolyDataVector_.push_back(temp_polydata);  // If intersection, consider dilated mesh as a pass
        else
          dilationPolyDataVector_.push_back(expanded_polydata);

        ROS_INFO_STREAM("Bezier::generateTrajectory: New pass generated");
      }
      else
        intersection_flag = false;  // No intersection : end of dilation
    }
    else if (expanded_polydata->GetNumberOfCells() > 10 && surfacing_mode_)
    {
      ROS_INFO_STREAM("Bezier::generateTrajectory: New surface pass generated");
      intersection_flag = false;
      dilationPolyDataVector_.push_back(expanded_polydata);
    }
    else
    {
      ROS_WARN_STREAM("Bezier::generateTrajectory: Number of cells isn't sufficient");
      intersection_flag = false;
    }
    depth += maximum_depth_of_path_;
  }

  if (use_translation_mode_)
    ROS_INFO_STREAM("Bezier::generateTrajectory: Translation process done");
  else
  {
    if (!surfacing_mode_)
      ROS_INFO_STREAM("Bezier::generateTrajectory: Dilation process done");
    else
      ROS_INFO_STREAM("Bezier::generateTrajectory: Surfacing process done");
  }
  // Reverse pass vector: grind from upper (last) pass
  std::reverse(dilationPolyDataVector_.begin(), dilationPolyDataVector_.end());
  // Generate extrication mesh data
  vtkSmartPointer<vtkPolyData> extrication_poly_data = vtkSmartPointer<vtkPolyData>::New();
  std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > extrication_lines;
  index_vector.push_back(way_points_vector.size() - 1);  // Push back index of last pose in pass
  // Generate trajectory
  for (unsigned int polydata_index = 0; polydata_index < dilationPolyDataVector_.size(); polydata_index++)
  {  // For each polydata (passes)
     // Generate extrication mesh
    double dist_to_extrication_mesh(0);
    if (polydata_index % extrication_frequency_ == 0)
    {
      if (use_translation_mode_)
      {
        dist_to_extrication_mesh = extrication_coefficient_ * maximum_depth_of_path_;
        translation(dist_to_extrication_mesh, dilationPolyDataVector_[polydata_index], extrication_poly_data);
      }
      else
      {
        //-> dilated_depth = extrication_coefficient+numberOfPolydataDilated-1-n*frequency)*maximum_depth_of_path
        double dilated_depth(
            (extrication_coefficient_ + dilationPolyDataVector_.size() - 1 - polydata_index) * maximum_depth_of_path_);
        dilation(dilated_depth, extrication_poly_data);
        //dilatation(extrication_coefficient_*maximum_depth_of_path_, dilationPolyDataVector_[polydata_index], extrication_poly_data);
        //double dist_to_extrication_mesh((extrication_coefficient_ + polydata_index) * maximum_depth_of_path_); //distance between dilationPolyDataVector_[index_polydata] and extrication polydata
      }
      generateStripperOnSurface(extrication_poly_data, extrication_lines);
    }
    else
      dist_to_extrication_mesh += maximum_depth_of_path_;
    // Generate trajectory on mesh (polydata)
    std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > lines;
    generateStripperOnSurface(dilationPolyDataVector_[polydata_index], lines);
    for (unsigned int index_line = 0; index_line < lines.size(); index_line++)
    {
      // Variable use to store pose : used for extrication paths
      Eigen::Affine3d start_pose(Eigen::Affine3d::Identity());  // Start line pose
      Eigen::Affine3d end_pose(Eigen::Affine3d::Identity());  // End line pose

      // Generate poses on a line
      for (unsigned int index_point = 0; index_point < lines[index_line].size(); index_point++) // For each point
      {
        if (lines[index_line].size() < 2)
        {
          ROS_WARN_STREAM("Bezier::generateTrajectory: Line path is too small (number of points on the line < 2)");
          break;
        }
        // Get points, normal and generate pose
        Eigen::Vector3d point, next_point, normal;
        Eigen::Affine3d pose(Eigen::Affine3d::Identity());
        bool flag_is_finite = true;  //check generated pose for NAN value

        if (index_point < (lines[index_line].size() - 1))
        {
          point = lines[index_line][index_point].first;
          next_point = lines[index_line][index_point + 1].first;
          normal = lines[index_line][index_point].second;
          flag_is_finite = generateRobotPoses(point, next_point, normal, pose);
        }
        else
        {
          point = lines[index_line][index_point - 1].first;
          next_point = lines[index_line][index_point].first;
          normal = lines[index_line][index_point - 1].second;
          flag_is_finite = generateRobotPoses(point, next_point, normal, pose);
          pose.translation() << next_point;
        }

        if (index_point == 0)
        {  // First point in line
          start_pose = pose;  // Save start pose
          way_points_vector.push_back(pose);  // Add pose with false color flag (out of line)
          color_vector.push_back(false);
        }

        if (flag_is_finite)
        {
          // Push robot pose into vector
          way_points_vector.push_back(pose);
          color_vector.push_back(true);
        }
        else
        {
          ROS_ERROR_STREAM("Bezier::generateTrajectory: The pose generated is not correct! Aborting generation.");
          return false;
        }

        if (index_point == (lines[index_line].size() - 1))
        {  // Last point in line
          end_pose = pose;
          way_points_vector.push_back(pose);  // Add pose with false color flag (out of line)
          color_vector.push_back(false);
        }
      }
      // End of line: Generate extrication to the next line
      if (index_line == (lines.size() - 1))  // No simple extrication for the last line of mesh
        break;
      Eigen::Vector3d end_point(end_pose.translation() + dist_to_extrication_mesh * end_pose.linear().col(0));
      Eigen::Vector3d dilated_end_point(end_pose.translation() - dist_to_extrication_mesh * end_pose.linear().col(2));
      Eigen::Vector3d dilated_start_point(start_pose.translation()
                                          - dist_to_extrication_mesh * start_pose.linear().col(2));
      // Seek closest line in extrication lines
      int index_of_closest_line = seekClosestLine(end_point, extrication_lines);
      // Seek for dilated_end_point neighbor in extrication line
      int index_of_closest_end_point;
      // Seek for dilated_start_point neighbor in extrication line
      int index_of_closest_start_point;

      if (use_translation_mode_)
      {
        index_of_closest_end_point = lines[index_line].size() - 1;
        index_of_closest_start_point = 0;
      }
      else
      {
        index_of_closest_end_point = seekClosestPoint(dilated_end_point, extrication_lines[index_of_closest_line]);
        index_of_closest_start_point = seekClosestPoint(dilated_start_point, extrication_lines[index_of_closest_line]);
      }

      // Get vector between these indices
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > extrication_line(
          extrication_lines[index_of_closest_line].begin() + index_of_closest_start_point,
          extrication_lines[index_of_closest_line].begin() + index_of_closest_end_point);

      // Generate pose on this vector (line)
      Eigen::Affine3d pose(start_pose);
      Eigen::Vector3d point(Eigen::Vector3d::Identity());
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > extrication_poses;
      for (unsigned int index_point = 0; index_point < extrication_line.size(); index_point++)
      {
        if (index_point == 0 || index_point == (extrication_line.size() - 1) || index_point % 5 == 0)
        {
          point = extrication_line[index_point].first;
          pose.translation() << point[0], point[1], point[2];
          extrication_poses.push_back(pose);
          color_vector.push_back(false);
        }
      }

      // Reverse extrication pose
      std::reverse(extrication_poses.begin(), extrication_poses.end());
      way_points_vector.insert(way_points_vector.end(), extrication_poses.begin(), extrication_poses.end());
    }

    //////////// EXTRICATION FROM LAST LINE TO FIRST ONE ////////////
    Eigen::Vector3d start_point_pass(lines[0][0].first);
    Eigen::Vector3d start_normal_pass(lines[0][0].second);
    Eigen::Vector3d end_point_pass(lines[lines.size() - 1][lines[lines.size() - 1].size() - 1].first);
    Eigen::Vector3d end_normal_pass(lines[lines.size() - 1][lines[lines.size() - 1].size() - 1].second);
    // Get vector from last point of last line and first point of first line
    Eigen::Vector3d extrication_pass_dir(end_point_pass - start_point_pass);
    extrication_pass_dir.normalize();
    // Get his orthogonal vector to use vtkCutter
    // First step : take his projection on the Ransac plan model
    Eigen::Vector3d extrication_cut_dir(
        extrication_pass_dir - (extrication_pass_dir.dot(mesh_normal_vector_)) * mesh_normal_vector_);
    // Second step : cross product with mesh normal
    extrication_cut_dir = extrication_cut_dir.cross(mesh_normal_vector_);
    extrication_cut_dir.normalize();
    // Cut this dilated mesh to determine extrication pass trajectory
    vtkSmartPointer<vtkStripper> extrication_stripper = vtkSmartPointer<vtkStripper>::New();
    extrication_stripper->SetJoinContiguousSegments(true);

    cutMesh(extrication_poly_data, extrication_cut_dir, 1, extrication_stripper);
    // Get last pose
    Eigen::Affine3d extrication_pose(Eigen::Affine3d::Identity());
    extrication_pose = way_points_vector.back();
    // Check stripper orientation
    vtkPoints *points = extrication_stripper->GetOutput()->GetPoints();
    vtkCellArray *cells = extrication_stripper->GetOutput()->GetLines();
    vtkIdType *indices;
    vtkIdType numberOfPoints;
    vtkIdType lastnumberOfPoints(0);  //FIXME Alternative solution used to face hole problems in dilated mesh
    unsigned int lineCount(0);

    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > extrication_poses;
    Eigen::Vector3d orientation = Eigen::Vector3d::Identity();

    for (cells->InitTraversal(); cells->GetNextCell(numberOfPoints, indices); lineCount++)
    {  // In case where more than one slice has been cut
      if (numberOfPoints > lastnumberOfPoints)
      {  // Get the max length stripper
        extrication_poses.clear();
        for (vtkIdType i = 0; i < numberOfPoints; i++)
        {
          double point[3];
          points->GetPoint(indices[i], point);
          extrication_pose.translation() << point[0], point[1], point[2];
          extrication_poses.push_back(extrication_pose);
        }
      }
    }

    orientation = extrication_pose.translation() - extrication_poses[0].translation();

    // Check orientation
    if (orientation.dot(extrication_pass_dir) > 0)
      std::reverse(extrication_poses.begin(), extrication_poses.end());

    // Seek for closest end pass point index
    unsigned int index_start_extrication_path;
    // Seek for closest start pass point index
    unsigned int index_end_extrication_path;

    if (use_translation_mode_)
    {
      index_start_extrication_path = 0;
      index_end_extrication_path = extrication_poses.size() - 1;
    }
    else
    {
      index_start_extrication_path = seekClosestExtricationPassPoint(
          end_point_pass - dist_to_extrication_mesh * end_normal_pass, extrication_poses);
      //seek for closest end pass point index
      index_end_extrication_path = seekClosestExtricationPassPoint(
          start_point_pass - dist_to_extrication_mesh * start_normal_pass, extrication_poses);
    }

    // Get indices of close points
    way_points_vector.insert(way_points_vector.end(), extrication_poses.begin() + index_start_extrication_path,
                             extrication_poses.begin() + index_end_extrication_path);

    for (size_t i = 0; i < (index_end_extrication_path - index_start_extrication_path); i++)
    {
      color_vector.push_back(false);
    }

    index_vector.push_back(way_points_vector.size() - 1);  // Push back index of last pose in pass
  }
  return true;
}

bool Bezier::rvizDisplayNormals(
    const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &way_points_vector,
    const std::vector<bool> points_color_viz,
    const ros::Publisher &normal_publisher,
    const Display disp,
    const unsigned int factor)
{

  if (way_points_vector.size() != points_color_viz.size())
  {
    ROS_ERROR_STREAM("Bezier::rvizDisplayNormals: Path vector and boolean vector have different sizes");
    return false;
  }

  int in_factor = factor;
  if (in_factor == 0)
    in_factor = 1;

  visual_tools_->waitForSubscriber(normal_publisher, 0.5, true);

  double length = 0.01;
  for (unsigned int i = 0; i < way_points_vector.size(); i++)
  {
    if (i % in_factor != 0)
      continue;

    if (disp == GRINDING && points_color_viz[i])
    {
      visual_tools_->publishZArrow(way_points_vector[i], rviz_visual_tools::GREEN, rviz_visual_tools::XSMALL, length, i);
    }
    else if (disp == EXTRICATION && !points_color_viz[i])
    {
      visual_tools_->publishZArrow(way_points_vector[i], rviz_visual_tools::RED, rviz_visual_tools::XSMALL, length, i);
    }
    else if (disp == ALL)
    {
      visual_tools_->publishZArrow(way_points_vector[i],
                                   (points_color_viz[i] ? rviz_visual_tools::GREEN : rviz_visual_tools::RED),
                                   rviz_visual_tools::XSMALL, length, i);
    }
    else
      return false;
  }

  visual_tools_->triggerBatchPublish();
  return true;
}

bool Bezier::rvizDisplayAxes(
    const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &way_points_vector,
    const std::vector<bool> points_color_viz,
    const ros::Publisher &axes_publisher,
    const Display disp,
    const unsigned int factor,
    const double axes_length,
    const double axes_radius)
{

  if (way_points_vector.size() != points_color_viz.size())
  {
    ROS_ERROR_STREAM("Bezier::rvizDisplayAxes: Path vector and boolean vector have different sizes");
    return false;
  }

  int in_factor = factor;
  if (in_factor == 0)
    in_factor = 1;

  visual_tools_->waitForSubscriber(axes_publisher, 0.5, true);

  for (unsigned int i = 0; i < way_points_vector.size(); i++)
  {
    if (i % in_factor != 0)
      continue;

    if (disp == GRINDING && points_color_viz[i])
    {
      visual_tools_->publishAxis(way_points_vector[i], axes_length, axes_radius);
    }
    else if (disp == EXTRICATION && !points_color_viz[i])
    {
      visual_tools_->publishAxis(way_points_vector[i], axes_length, axes_radius);
    }
    else if (disp == ALL)
    {
      visual_tools_->publishAxis(way_points_vector[i], axes_length, axes_radius);
    }
    else
      return false;
  }

  visual_tools_->triggerBatchPublish();
  return true;
}

void Bezier::rvizRemoveAllMarkers(void)
{
  visual_tools_->deleteAllMarkers();
}

void Bezier::displayTrajectory(
    const std::vector<Eigen::Affine3d,
    Eigen::aligned_allocator<Eigen::Affine3d> > &way_points_vector,
    const std::vector<bool> points_color_viz,
    const ros::Publisher &trajectory_publisher)
{
  // Rviz cannot display more than 8192, else the program will crash
  if (way_points_vector.size() >= 8192)
  {
    ROS_ERROR_STREAM("Bezier::displayTrajectory: RViz cannot display more than 8192 points, aborting publish");
    return;
  }

  //check possible error
  if (way_points_vector.size() != points_color_viz.size())
  {
    ROS_ERROR_STREAM("Bezier::displayTrajectory: Path vector and bool vector have different sizes");
  }
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.scale.x = 0.001;  // Set scale of our new marker : Diameter in our case.
  // Set marker orientation. Here, there is no rotation : (v1,v2,v3)=(0,0,0) angle=0
  marker.pose.orientation.x = 0.0;      // v1* sin(angle/2)
  marker.pose.orientation.y = 0.0;      // v2* sin(angle/2)
  marker.pose.orientation.z = 0.0;      // v3* sin(angle/2)
  marker.pose.orientation.w = 1.0;      // cos(angle/2)

  // Set the pose and color of marker from parameter[in]
  geometry_msgs::Point p;  // Temporary point
  std_msgs::ColorRGBA color;  // Temporary color
  for (unsigned int k = 1; k < way_points_vector.size(); k++)
  {
    // For each index of trajectory vector, store coordinates in a temporary point p;
    p.x = way_points_vector[k].translation()[0];
    p.y = way_points_vector[k].translation()[1];
    p.z = way_points_vector[k].translation()[2];
    // For each index of boolean vector, check membership of point;
    if (points_color_viz[k] == true)
    {
      color.r = 0.0f;
      color.g = 1.0f;  // If points is from grindstone path : Color is green
      color.b = 0.0f;
      color.a = 1.0;
    }
    else
    {
      color.r = 1.0f;  // If points is from extrication path : Color is red
      color.g = 0.0f;
      color.b = 0.0f;
      color.a = 1.0;
    }
    marker.colors.push_back(color);     // Add color c to LINE_STRIP to set color segment
    marker.points.push_back(p);         // Add point p to LINE_STRIP
  }
  while (trajectory_publisher.getNumSubscribers() < 1)
  {
    ROS_WARN_ONCE("Bezier::displayTrajectory: Please create a subscriber to the trajectory marker");
    sleep(1);
  }
  trajectory_publisher.publish(marker);
}

void Bezier::displayMesh(const ros::Publisher &mesh_publisher,
                         const std::string mesh_path,
                         const float r,
                         const float g,
                         const float b,
                         const float a)
{
  // Create a mesh marker from ply files
  visualization_msgs::Marker mesh_marker;
  mesh_marker.header.frame_id = "/base_link";
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

  while (mesh_publisher.getNumSubscribers() < 1)
  {
    ROS_WARN_ONCE("Bezier::displayMesh: Please create a subscriber to the mesh marker");
    sleep(1);
  }
  mesh_publisher.publish(mesh_marker);
}
