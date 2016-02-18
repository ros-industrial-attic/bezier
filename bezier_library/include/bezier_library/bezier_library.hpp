#ifndef BEZIER_LIBRARY_HPP
#define BEZIER_LIBRARY_HPP

#include <string>

// PCL headers
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

// Eigen headers
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/StdVector>
#include <Eigen/Dense>

// VTK headers
#include <vtkVersion.h>
#include <vtkMath.h>
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkTriangleFilter.h>
#include <vtkStripper.h>
#include <vtkCellData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkGlyph3D.h>
#include <vtkImplicitModeller.h>
#include <vtkMarchingCubes.h>
#include <vtkKdTreePointLocator.h>
#include <vtkReverseSense.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkDistancePolyDataFilter.h>

// ROS headers
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/lexical_cast.hpp>

/**
 * @file bezier_library.hpp
 * @brief Library used to generate 3D paths (robot poses) from PLY files.
 * @author Francois Lasson, Kévin Bolloré - Institut Maupertuis (France)
 * @date Project started in February 2015
 */

/** @mainpage Bézier library
 * Please read [ros-industrial-consortium/bezier](https://github.com/ros-industrial-consortium/bezier#description)
 */

/** @brief PointT is a pcl::PointXYZRGBA */
typedef pcl::PointXYZRGBA PointT;
/** @brief PointCloudT is a PointT cloud */
typedef pcl::PointCloud<PointT> PointCloudT;

/** @brief Bezier class */
class Bezier
{
public:

  /** @brief Default Constructor */
  Bezier();

  /** @brief Initialized constructor
   *  @param[in] filename_inputMesh filename of input poly data (input mesh)
   *  @param[in] filename_defectMesh filename of defect poly data (defect mesh)
   *  @param[in] maximum_depth_of_path maximum grinding depth (in meters)
   *  @param[in] effector_diameter diameter of effector (in meters)
   *  @param[in] covering_percentage Percentage of covering (decimal value)
   *  @param[in] extrication_coefficient extrication depth equal of the percentage of maximum_depth_of_path (coefficient)
   *  @param[in] extrication_frequency new extrication mesh generated each 1/extrication_frequency times
   *  @param[in] use_translation_mode see @ref use_translation_mode_
   */
  Bezier(std::string filename_inputMesh,
         std::string filename_defectMesh,
         double maximum_depth_of_path,
         double effector_diameter,
         double covering_percentage,
         int extrication_coefficient,
         int extrication_frequency,
         bool use_translation_mode = false);

  ~Bezier();

  /** @brief Generate 3D paths thanks the the input parameters
   *  @param[out] way_points_vector in order to save robot poses
   *  @param[out] color_vector in order to know if pose is an extrication pose or not (Use to display path)
   *  @param[out] index_vector in order to know index of start and end pose in each pass (Use to display path)
   *  @return True if successful, false otherwise
   */
  bool
  generateTrajectory(std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &way_points_vector,
                     std::vector<bool> &color_vector,
                     std::vector<int> &index_vector);

  /** @brief Allow to save all dilated vtkPolyData
   *  @param[in] path is the path of folder where we want to save the vtkPolyData
   *  @return True if successful, false otherwise
   **/
  bool
  saveDilatedMeshes(std::string path);

  /** @brief Retrieve the slicing direction vector
   *  @return @ref slicing_dir_
   **/
  Eigen::Vector3d
  getSlicingDirection();

  /** @brief Allow to display normals in RVIZ
   *  @param[in] way_points_vector vector containing robot poses
   *  @param[in] points_color_viz vector of booleans useful to distinguish machining / extrication path
   *  @param[out] normal_publisher publisher used to display normals in RVIZ
   *  @note Boolean vector and trajectory vector must be the same size
   **/
  void
  displayNormal(std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector,
                std::vector<bool> points_color_viz,
                ros::Publisher &normal_publisher);

  /** @brief Generate a trajectory (LINE_STRIP) marker and publish it
   *  @param[in] way_points_vector 3D trajectory vector (containing poses)
   *  @param[in] points_color_viz vector of booleans useful to distinguish machining / extrication path
   *  @param[out] trajectory_publisher reference to a ROS publisher used to display trajectory marker
   *  @note Boolean vector and trajectory vector must be the same size
   */
  void
  displayTrajectory(std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector,
                    std::vector<bool> points_color_viz,
                    ros::Publisher &trajectory_publisher);

  /** @brief Allow to display a mesh in RVIZ
   *  @param[out] mesh_publisher publisher used to display the mesh in RVIZ
   *  @param[in] mesh_path path of the mesh
   *  @param[in] r red component of the RGB rate
   *  @param[in] g green component of the RGB rate
   *  @param[in] b blue component of the RGB rate
   **/
  void
  displayMesh(ros::Publisher &mesh_publisher,
              std::string mesh_path,
              float r = 0.6, float g = 0.6, float b = 0.6);

  /** @brief Function used to display some Bezier library's parameters (effector diameter, grind depth and covering_percentage) */
  void
  printBezierParameters();

  /** @brief Use translation mode (see @ref use_translation_mode_)
   *  @param[in] use_translation
   */
  void
  setTranslationMode(bool use_translation);

  /** @brief Get @ref use_translation_mode_
   *  @return True if using translation mode, false otherwise
   */
  bool
  getTranslationMode();

private:
  /** @brief Input mesh */
  vtkSmartPointer<vtkPolyData> inputPolyData_;

  /** @brief Defect mesh */
  vtkSmartPointer<vtkPolyData> defectPolyData_;

  /** @brief Vector containing several dilated meshes */
  std::vector<vtkSmartPointer<vtkPolyData> > dilationPolyDataVector_;

  /** @brief Grinding depth (in meters) */
  double maximum_depth_of_path_;

  /** @brief Diameter of effector (in meters) */
  double effector_diameter_;

  /** @brief Percentage of covering (decimal value) */
  double covering_percentage_;

  /** @brief Extrication depth equal of how many @ref maximum_depth_of_path_ (coefficient) should be used to generate extrication paths */
  int extrication_coefficient_;

  /** @brief New extrication mesh generated each 1/extrication_frequency times */
  int extrication_frequency_;

  /** @brief Mesh global normal vector */
  Eigen::Vector3d mesh_normal_vector_;

  /** @brief Vector direction for the slicing operation */
  Eigen::Vector3d slicing_dir_;

  /** @brief Stores the last number of normal markers published. Useful to get them deleted */
  unsigned int number_of_normal_markers_published_;

  /** @brief If true, uses a translation expansion instead of a dilation. False by default */
  bool use_translation_mode_;

  /** @brief This function uses vtkImplicitModeller in order to dilate the input vtkPolyData surface
   *  @param[in] depth depth for grind process (pass depth)
   *  @param[out] dilated_polydata output data
   *  @return True if successful, false otherwise
   *  @bug Dilation problem happens when depth is too high, dilated mesh has unexpected holes\n
   *  These holes are problematic. In fact, when cutting process is called on dilated mesh, slices are divided in some parts due to these holes
   *  and this affects the path generation, especially for extrication trajectory\n
   *  We have to find a solution, perhaps find best parameters in order to resolve this problem
   */
  bool
  dilation(double depth,
             vtkSmartPointer<vtkPolyData> &dilated_polydata);

  /**@brief This function used vtkImplicitModeller in order to translate the @ref inputPolyData_ surface
   * @param[in] depth depth for grind process (pass depth)
   * @param[in] poly_data Poly data we would like to translate
   * @param[out] translation_poly_data output of input poly data (@ref inputPolyData_) translation
   * @return boolean flag reflects the function proceedings
   */
  bool
  translation(double depth,
              vtkSmartPointer<vtkPolyData> poly_data,
              vtkSmartPointer<vtkPolyData> &translation_poly_data);

  /** @brief This function allows to optimize path generation. When passes are generated (dilation), we make an intersection between the
   * dilated mesh / defect mesh in order to only save useful part of mesh
   *  @param[in, out] poly_data mesh generated after dilation and before intersection
   *  @return True if successful, false otherwise
   * @bug Sometimes, unexpected parts of the mesh are saved
   */
  bool
  defectIntersectionOptimisation(vtkSmartPointer<vtkPolyData> &poly_data);

  /** @brief Determine the normals of the cells in a mesh
   *  @param[in, out] poly_data vtkPolyData in which normals are computed
   *  @return True if successful, false otherwise
   */
  bool
  generateCellNormals(vtkSmartPointer<vtkPolyData> &poly_data);

  /** @brief Determine the normals of the points in a mesh
   *  @param[in, out] poly_data vtkPolyData in which normals are computed
   *  @return True if successful, false otherwise
   */
  bool
  generatePointNormals(vtkSmartPointer<vtkPolyData> &poly_data);

  /** @brief Allows to load a PLY file into a vtkPolyData
   *  @param[in] filename file path
   *  @param[out] poly_Data vtkPolyData loaded
   *  @return True if successful, false otherwise
   */
  bool
  loadPLYPolydata(std::string filename, vtkSmartPointer<vtkPolyData> &poly_Data);

  /** @brief Allows to save a PLY file from a vtkPolyData object
   *  @param[in] filename file path
   *  @param[in] poly_data vtkPolyData to be saved
   *  @return True if successful, false otherwise
   */
  bool
  savePLYPolyData(std::string filename, vtkSmartPointer<vtkPolyData> poly_data);

  /** @brief Uses PCL RANSAC to segment a plane into @ref inputPolyData_ and fills @ref mesh_normal_vector_
   *  @note The segmented plane represents a global mesh orientation
   */
  void
  ransacPlaneSegmentation();

  /** @brief Computes a slicing vector direction thanks to the @ref mesh_normal_vector_
   * In fact, in this process we would like a @ref slicing_dir_ orthogonal to the mesh normal so the slicing
   * direction is a vector that belongs to the plane model estimated
   */
  void
  generateSlicingDirection();

  /** @brief Compute scalar product between @ref slicing_dir_ and all points of the vtkPolyData point cloud
   *  to to determine the number of lines that should be sliced into the mesh
   *  @param[in] poly_data mesh to be sliced
   *  @return Number of lines that should be sliced in order to respect the covering_percentage constraint
   */
  unsigned int
  determineSliceNumberExpected(vtkSmartPointer<vtkPolyData> poly_data);

  /** @brief Allows to compute the real number of slices because if there are holes in the vtkPolyData,
   *  the line number returned by VTK is not the one expected
   *  @param stripper slice cut without organization
   *  @param vector_dir cut direction
   *  @return Real line number
   */
  unsigned int
  getRealSliceNumber(vtkSmartPointer<vtkStripper> stripper,
                     Eigen::Vector3d vector_dir);

  /** @brief Slices a vtkPolyData, the result is a series of lines stored into a vtkStripper
   *  @param[in] PolyData vtkPolyData represents the mesh to be cut
   *  @param[in] line_number_expected Number of slices expected
   *  @param[in] cut_dir Slicing direction
   *  @param[out] stripper Output lines
   *  @return True if successful, false otherwise
   */
  bool
  cutMesh(vtkSmartPointer<vtkPolyData> PolyData,
          Eigen::Vector3d cut_dir, unsigned int line_number_expected,
          vtkSmartPointer<vtkStripper> &stripper);

  /** @brief This function checks lines orientation and compares them with a reference vector (cut_direction.dot(mesh_normal))
   *  It reverts the lines so that they all have the same direction
   *  @param[in, out] lines is a vector containing several lines. Each line is a vector of pairs (first point position, second z normal)
   */
  void
  harmonizeLinesOrientation(std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > &lines);

  /** @brief Removes points that are very close in a point cloud
   *  Points that are too close can lead to numerical errors when trying to normalize vectors (division by 0)
   *  @param[in, out] lines lines is a vector containing several lines. Each line is a vector of pairs (first point position, second z normal)
   *  @note The threshold distance is fixed to 0.001
   */
  void
  removeNearNeighborPoints(std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > &lines);

  /** @brief Determines the tool orientation thanks to the input data in order to generate a robot pose
   *  @param[in] point current robot position
   *  @param[in] point_next next robot position
   *  @param[in] normal current robot position point normal (z normal)
   *  @param[out] pose a robot pose
   *  @return True if successful, false otherwise
   */
  bool
  generateRobotPoses(Eigen::Vector3d point,
                     Eigen::Vector3d point_next,
                     Eigen::Vector3d normal,
                     Eigen::Affine3d &pose);

  /** @brief Generates 3D robot trajectories on a vtkPolyData
   *  @param[in] PolyData mesh to generate trajectories one
   *  @param[out] lines a vector containing several lines. Each line is a vector of pairs (first point position, second z normal)
   *  @return True if successful, false otherwise
   */
  bool
  generateStripperOnSurface(vtkSmartPointer<vtkPolyData> PolyData,
                            std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > &lines);

  /** @brief Function used to generate extrication paths between two lines. It allows to find the closest extrication line of a point
   *  @param[in] point_vector vector of point position
   *  @param[in] extrication_lines a vector containing several lines. Each line is a vector of pairs (first point position, second z normal)\n
   *                                  extrication_lines are lines generated on dilated (extrication) mesh
   *  @return Value equal to index of closest line in extrication_lines vector.
   */
  int
  seekClosestLine(Eigen::Vector3d point_vector,
                  std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > extrication_lines);

  /** @brief Function used for extrication path. Extrication between two lines. It allows to find the closest extrication point of a point in a line.
   *  @param[in] point_vector Eigen vector of point position
   *  @param[in] extrication_line Line is a vector of pairs (first point position, second z normal)\n
   *                                  extrication_line are the closest line on the dilated (extrication) mesh
   *  @return Value equal to index of closest point in extrication_line vector
   */
  int
  seekClosestPoint(Eigen::Vector3d point_vector,
                   std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > extrication_line);

  /** @brief Function used for extrication path. Extrication between two passes. It allows to find the closest extrication point of a point in the last extrication line
   *  @param[in] point_vector Eigen vector of point position
   *  @param[in] extrication_poses Vector containing all poses generated in the stripper between a pass i and a pass i+1.
   *  @return Value equal to index of closest point in extrication_poses vector.
   */
  int
  seekClosestExtricationPassPoint(
      Eigen::Vector3d point_vector,
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > extrication_poses);
};

#endif
