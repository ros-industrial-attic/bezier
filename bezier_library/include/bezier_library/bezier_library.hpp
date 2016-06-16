#ifndef BEZIER_LIBRARY_HPP
#define BEZIER_LIBRARY_HPP

// PCL headers
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

// VTK headers
#include <vtkMath.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkTriangleFilter.h>
#include <vtkStripper.h>
#include <vtkCellData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
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

//RViz headers
#include "rviz_visual_tools/rviz_visual_tools.h"
#include "moveit_visual_tools/moveit_visual_tools.h"

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


/** \brief Set of display options. */
enum Display
{
  GRINDING,     /**< display items related to the grinding path */
  EXTRICATION,  /**< display items related to the extrication path */
  ALL           /**< display items related both to the grinding and extrication path */
};

/** @brief Bezier class */
class Bezier
{
public:

  /** @brief Default Constructor */
  Bezier();

  /** @brief Initialized constructor
   *  @param[in] filename_inputMesh filename of input poly data (input mesh)
   *  @param[in] filename_defectMesh filename of defect poly data (defect mesh)
   *  @param[in] rviz_base name of the base in which objects will be published in RViz
   *  @param[in] rviz_topic_name name of the topic for objects publication in RViz
   *  @param[in] lean_angle_axis rotation axis for the lean angle
   *  @param[in] angle_value rotation value (in radians) around lean_angle_axis
   *  @param[in] maximum_depth_of_path maximum grinding depth (in meters)
   *  @param[in] working_line_width width of the tool which is used to cut (in meters)
   *  @param[in] covering_percentage Percentage of covering (decimal value)
   *  @param[in] extrication_coefficient extrication depth equal of the percentage of maximum_depth_of_path (coefficient)
   *  @param[in] extrication_frequency new extrication mesh generated each 1/extrication_frequency times
   *  @param[in] use_translation_mode see @ref use_translation_mode_
   */
  Bezier(const std::string filename_inputMesh,
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
         const bool use_translation_mode);

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
  saveDilatedMeshes(const std::string path);

  /** @brief Retrieve the slicing direction vector
   *  @return @ref slicing_dir_
   **/
  Eigen::Vector3d
  getSlicingDirection();

  /** @brief Allow to display normals in RViz
   *  @param[in] way_points_vector vector containing robot poses
   *  @param[in] points_color_viz vector of booleans useful to distinguish machining / extrication path
   *  @param[in] name of the publisher
   *  @param[in] axes_publisher publisher used to display axes in RViz
   *  @param[in] factor division coefficient to display an axe each factor time (ignored if equals to 0)
   *  @param[in] axes_length length of the axes to be published
   *  @param[in] axes_radius radius of the axes to be published
   *  @param[in] disp select where the axes must be displayed : GRINDING for grind path, EXTRICATION for extrication pass
   *             ALL for both
   *  @return True if the displayed has been done correctly, False otherwise
   *  @note Boolean vector and trajectory vector must be the same size
   *  @note The markers can be removed through the function rvizRemoveAllMarkers()
   **/
  bool
  rvizDisplayAxes(const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &way_points_vector,
                  const std::vector<bool> points_color_viz,
                  const ros::Publisher &axes_publisher,
                  const Display disp = ALL,
                  const unsigned int factor = 1,
                  const double axes_length = 0.01,
                  const double axes_radius = 0.0005);

  /** @brief Allow to display normals in RViz
   *  @param[in] way_points_vector vector containing robot poses
   *  @param[in] Color of the normals
   *  @param[in] name of the publisher
   *  @param[in] normal_publisher publisher used to display normals in RViz
   *  @param[in] factor division coefficient to display an normal each factor time (ignored if equals to 0)
   *  @param[in] disp select where the normals must be displayed : GRINDING for grind path, EXTRICATION for extrication pass
   *             ALL for both
   *  @return True if the displayed has been done correctly, False otherwise
   *  @note Boolean vector and trajectory vector must be the same size
   *  @note The markers can be removed through the function rvizRemoveAllMarkers()
   **/
  bool
  rvizDisplayNormals(const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &way_points_vector,
                     const std::vector<bool> points_color_viz,
                     const ros::Publisher &normal_publisher,
                     const Display disp = ALL,
                     const unsigned int factor = 1);

  /** @brief Remove all markers in RViz
   **/
  void rvizRemoveAllMarkers(void);

  /** @brief Generate a trajectory (LINE_STRIP) marker and publish it
   *  @param[in] way_points_vector 3D trajectory vector (containing poses)
   *  @param[in] points_color_viz vector of booleans useful to distinguish machining / extrication path
   *  @param[out] trajectory_publisher reference to a ROS publisher used to display trajectory marker
   *  @note Boolean vector and trajectory vector must be the same size
   */
  void
  displayTrajectory(const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &way_points_vector,
                    const std::vector<bool> points_color_viz,
                    const ros::Publisher &trajectory_publisher);

  /** @brief Allow to display a mesh in RViz
   *  @param[out] mesh_publisher publisher used to display the mesh in RViz
   *  @param[in] mesh_path path of the mesh
   *  @param[in] r red component of the RGB rate
   *  @param[in] g green component of the RGB rate
   *  @param[in] b blue component of the RGB rate
   *  @param[in] a transparency component
   **/
  void
  displayMesh(const ros::Publisher &mesh_publisher,
              const std::string mesh_path,
              const float r = 0.6,
              const float g = 0.6,
              const float b = 0.6,
              const float a = 1.0);

  /** @brief Function used to display some Bezier library's parameters (effector diameter, grind depth and covering_percentage) */
  void
  printBezierParameters();

  /** @brief Use translation mode (see @ref use_translation_mode_)
   *  @param[in] use_translation
   */
  void
  setTranslationMode(const bool use_translation);

  /** @brief Get @ref use_translation_mode_
   *  @return True if using translation mode, false otherwise
   */
  bool
  getTranslationMode();

  /** @brief Activate the surfacing mode
   */
  void setSurfacingOn();

  /** @brief Deactivate the surfacing mode
   */
  void setSurfacingOff();

  /** @brief Set the surfacing mode
   *  @param[in] surface_mode
   */
  void setSurfacing(const bool surfacing_mode);

  /** @brief Get the surfacing mode
   * @return surfacing_mode
   */
  bool getSurfacing() const;

private:
  /** @brief Input mesh */
  vtkSmartPointer<vtkPolyData> inputPolyData_;

  /** @brief Defect mesh */
  vtkSmartPointer<vtkPolyData> defectPolyData_;

  /** @brief Vector containing several dilated meshes */
  std::vector<vtkSmartPointer<vtkPolyData> > dilationPolyDataVector_;

  /** @brief Visualization tool handling display of item in RViz */
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  /** @brief Lean angle axis (string) */
  std::string lean_angle_axis_;

  /** @brief angle_value_ */
  double angle_value_;

  /** @brief Grinding depth (in meters) */
  double maximum_depth_of_path_;

  /** @brief width of working tool (in meters) */
  double working_line_width_;

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

  /** @brief If true, No dilation will be done, we only make a surfacing process onto the scanned mesh*/
  bool surfacing_;

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
  dilation(const double depth, vtkSmartPointer<vtkPolyData> &dilated_polydata);

  /**@brief This function used vtkImplicitModeller in order to translate the @ref inputPolyData_ surface
   * @param[in] depth depth for grind process (pass depth)
   * @param[in] poly_data Poly data we would like to translate
   * @param[out] translation_poly_data output of input poly data (@ref inputPolyData_) translation
   * @return boolean flag reflects the function proceedings
   */
  bool
  translation(const double depth, const vtkSmartPointer<vtkPolyData> poly_data,
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
   *  @param[in] meshname file path
   *  @param[out] poly_Data vtkPolyData loaded
   *  @return True if successful, false otherwise
   */
  bool
  loadPLYPolydata(const std::string meshname,
                  vtkSmartPointer<vtkPolyData> &poly_Data);

  /** @brief Allows to save a PLY file from a vtkPolyData object
   *  @param[in] meshname file path
   *  @param[in] poly_data vtkPolyData to be saved
   *  @return True if successful, false otherwise
   */
  bool
  savePLYPolyData(const std::string meshname,
                  const vtkSmartPointer<vtkPolyData> poly_data);

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

  /** @brief Allows to compute the real number of slices because if there are holes in the vtkPolyData,
   *  the line number returned by VTK is not the one expected
   *  @param stripper slice cut without organization
   *  @param vector_dir cut direction
   *  @return Real line number
   */
  unsigned int
  getRealSliceNumber(vtkSmartPointer<vtkStripper> stripper,
                     const Eigen::Vector3d &vector_dir);

  /** @brief Slices a vtkPolyData, the result is a series of lines stored into a vtkStripper
   *  @param[in] PolyData vtkPolyData represents the mesh to be cut
   *  @param[in] line_number_expected Number of slices expected
   *  @param[in] cut_dir Slicing direction
   *  @param[out] stripper Output lines
   *  @return True if successful, false otherwise
   */
  bool
  cutMesh(const vtkSmartPointer<vtkPolyData> PolyData,
          const Eigen::Vector3d &cut_dir,
          const unsigned int line_number_expected,
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
  generateRobotPoses(const Eigen::Vector3d &point,
                     const Eigen::Vector3d &point_next,
                     const Eigen::Vector3d &normal,
                     Eigen::Affine3d &pose);

  /** @brief Generates 3D robot trajectories on a vtkPolyData
   *  @param[in] PolyData mesh to generate trajectories one
   *  @param[out] lines a vector containing several lines. Each line is a vector of pairs (first point position, second z normal)
   *  @return True if successful, false otherwise
   */
  bool
  generateStripperOnSurface(const vtkSmartPointer<vtkPolyData> PolyData,
                            std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > &lines);

  /** @brief Function used to generate extrication paths between two lines. It allows to find the closest extrication line of a point
   *  @param[in] point_vector vector of point position
   *  @param[in] extrication_lines a vector containing several lines. Each line is a vector of pairs (first point position, second z normal)\n
   *                                  extrication_lines are lines generated on dilated (extrication) mesh
   *  @return Value equal to index of closest line in extrication_lines vector.
   */
  int
  seekClosestLine(const Eigen::Vector3d &point_vector,
                  const std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > &extrication_lines);

  /** @brief Function used for extrication path. Extrication between two lines. It allows to find the closest extrication point of a point in a line.
   *  @param[in] point_vector Eigen vector of point position
   *  @param[in] extrication_line Line is a vector of pairs (first point position, second z normal)\n
   *                                  extrication_line are the closest line on the dilated (extrication) mesh
   *  @return Value equal to index of closest point in extrication_line vector
   */
  int
  seekClosestPoint(const Eigen::Vector3d &point_vector,
                   const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > &extrication_line);

  /** @brief Function used for extrication path. Extrication between two passes. It allows to find the closest extrication point of a point in the last extrication line
   *  @param[in] point_vector Eigen vector of point position
   *  @param[in] extrication_poses Vector containing all poses generated in the stripper between a pass i and a pass i+1.
   *  @return Value equal to index of closest point in extrication_poses vector.
   */
  int
  seekClosestExtricationPassPoint(const Eigen::Vector3d &point_vector,
                                  const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &extrication_poses);

  /** @brief Function used to provide a lean angle for effector
   * @param[in, out] pose rotation matrix modified with the lean angle
   * @param[in] lean_angle_axis axis of rotation
   * @param[in] angle_value value of angle of rotation (radians)
   * @return True if rotation is successful, false otherwise
   */
  bool
  applyLeanAngle(Eigen::Affine3d &pose,
                 const std::string lean_angle_axis,
                 const double angle_value);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
