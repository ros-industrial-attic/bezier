#ifndef BEZIER_HPP
#define BEZIER_HPP

// STL
#include <map>
#include <string>
#include <vector>

// Boost
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include <eigen_stl_containers/eigen_stl_vector_container.h>

// VTK
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkSTLReader.h>
#include <vtkOBJExporter.h>
#include <vtkPLYWriter.h>
#include <vtkSTLWriter.h>
#include <vtkPolyDataNormals.h>
#include <vtkImplicitModeller.h>
#include <vtkMarchingCubes.h>
#include <vtkStripper.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkTriangleFilter.h>
#include <vtkMath.h>
#include <vtkFloatArray.h>
#include <vtkKdTreePointLocator.h>
#include <vtkReverseSense.h>
#include <vtkPointData.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkAppendPolyData.h>
#include <bezier/error_observer.hpp>

// PCL
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/segmentation/sac_segmentation.h>
#pragma GCC diagnostic pop
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/PolygonMesh.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "append_bezier_exception.hpp"

/**
 * Bezier abstract class
 */
class Bezier
{
public:
  /**
   * Convenience typedef
   */
  typedef pcl::PointXYZ PointT;

  /**
   * Convenience typedef
   */
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> PointNormal;
  typedef std::vector<PointNormal> BezierPointNormalTable;

  enum AXIS_OF_ROTATION
  {
    X,
    Y,
    Z
  };

protected:

public:
  Bezier();
  virtual ~Bezier()
  {
  }

  /**
   * @returns the name of the current Bezier object
   */
  std::string
  virtual
  name() = 0;

  /**
   * Generate a trajectory using the input mesh and parameters
   * @param[out] trajectory
   * @param[out] is_grinding_pose whether the pose is a grinding or extrication one
   * @param[in] display_markers
   * @return An empty string on success, an error string otherwise
   * @note trajectory and is_grinding_pose are always the same size
   */
  std::string virtual generateTrajectory(EigenSTL::vector_Isometry3d &trajectory,
                                         std::vector<bool> &is_grinding_pose,
                                         bool display_markers) = 0;

  /**
   * Blocking call to make sure there is at least one subscriber to the RViz visual tool publisher
   */
  void waitForRvizVisualToolsSubscriber();

  /**
   * @brief This function allows to specify the parameters of the dilation process
   * @param i sample dimension i
   * @param j sample dimension j
   * @param k sample dimension k
   * @param max_distance is the distance away from surface of input geometry to sample
   */
  void setDilationParameters(const double i = 50, const double j = 50, const double k = 50,
                             const double max_distance = 0);

  /**
   * @brief This function allows to specify the parameters of the dilation process
   * @param sample_dimensions is an array containing the sample dimensions i, j and k
   * @param max_distance is the distance away from surface of input geometry to sample
   */
  void setDilationParameters(const double * sample_dimensions,
                             const double max_distance = 0);

protected:
  /**
   * VTK error observer to manage failures when loading files etc.
   */
  vtkSmartPointer<ErrorObserver> vtk_observer_;

  /**
   * A visual tool object to allow publishing markers (text, trajectories etc.)
   */
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  /**
   * Meshes provided to Bezier
   */
  std::vector<vtkSmartPointer<vtkPolyData> > input_meshes_;

  /**
   * Check if the parameters provided to Bezier are correct
   */
  std::string virtual validateParameters() = 0;

  /**
   * Sample dimension i parameter of the dilation
   */
  double dilation_sample_dimensions_i_;

  /**
   * Sample dimension j parameter of the dilation
   */
  double dilation_sample_dimensions_j_;

  /**
   * Sample dimension k parameter of the dilation
   */
  double dilation_sample_dimensions_k_;

  /**
   * Maximum distance parameter of the dilation
   */
  double dilation_maximum_distance_;

  /**
   * Append a OBJ, PLY or STL mesh into the vector of meshes input_meshes_
   * @param[in] file_absolute_path
   * @return True if successful, false otherwise
   */
  bool appendInputMesh(const std::string file_absolute_path);

  /**
   * Save a mesh file on the disk, supported formats are PLY and STL
   * @param[in] file_absolute_path
   * @param[in] polydata the mesh to be saved
   * @return True if successful, false otherwise
   */
  bool saveMesh(const std::string file_absolute_path,
                const vtkSmartPointer<vtkPolyData> &polydata);

  /**
   * Publish a PLY mesh on a ROS topic
   * @param[in] mesh_publisher
   * @param[in] mesh_path
   * @param[in] red
   * @param[in] green
   * @param[in] blue
   * @param[in] alpha
   * @param[in] frame_id
   */
  void displayMesh(const std::shared_ptr<ros::Publisher> &mesh_publisher,
                   const std::string mesh_path,
                   const float red,
                   const float green,
                   const float blue,
                   const float alpha = 1.0,
                   std::string frame_id = "/base_link");

  /**
   * Display a colored trajectory
   * @param[in] trajectory
   * @param color
   * @param display_normals display Z normal
   * @param display_labels display pose ID
   */
  void displayTrajectory(const EigenSTL::vector_Isometry3d &trajectory,
                         rviz_visual_tools::colors color,
                         const bool display_normals = false,
                         const bool display_labels = false);

  /**
   * Estimate a global normal for a mesh
   * @param[in] polydata
   * @param[out] mesh_normal
   * @param[in] iterations RANSAC iterations
   * @return True if successful, false otherwise
   */
  bool estimateGlobalMeshNormal(vtkSmartPointer<vtkPolyData> &polydata,
                                Eigen::Vector3d &mesh_normal,
                                const unsigned iterations = 2000);

  /**
   * Automatically computes a cutting orientation vector given a polydata.\n
   * The orientation computed is normalized
   * @param[in] polydata
   * @param[out] mesh_orientation The global mesh orientation
   * @param[out] orientation The slicing orientation
   * @return True if successful, false otherwise
   */
  bool estimateSlicingOrientation(vtkSmartPointer<vtkPolyData> &polydata,
                                  Eigen::Vector3d &mesh_orientation,
                                  Eigen::Vector3d &orientation);

  /**
   * Computes all the plane equations we need to slice the whole part given the polydata and the covering percentage.\n
   * This function computes the bound of the polydata and determines the number of slices using covering_percentage_\n
   * The planes equations are ordered from one side to another (plane @c n+1 is a neighbor of plane @c n)
   * @param[in] polydata
   * @param[in] slicing_orientation
   * @param[in] polydata_center the origin used to compute all the planes
   * @param[in] tool_effective_diameter is the width of the effective part of the tool
   * @param[in] covering_percentage is the covering percentage between two successive path
   * @param[out] planes_equations the equations of the planes
   */
  void estimateSlicingPlanes(const vtkSmartPointer<vtkPolyData> &polydata,
                                     const Eigen::Vector3d &slicing_orientation,
                                     const Eigen::Vector3d &polydata_center,
                                     const double tool_effective_diameter,
                                     const unsigned covering_percentage,
                                     EigenSTL::vector_Vector4d &planes_equations);

  /**
   * Estimate a slicing plane between line N and line N+1
   * @param[in] line_n_last_point
   * @param[in] line_n1_first_point
   * @param[in] global_mesh_normal
   * @param[out] plane_equation_normal
   * @param[out] plane_origin
   */
  void estimateExtricationSlicingPlane(const Eigen::Vector3d &line_n_last_point,
                                       const Eigen::Vector3d &line_n1_first_point,
                                       const Eigen::Vector3d &global_mesh_normal,
                                       Eigen::Vector4d &plane_equation_normal,
                                       Eigen::Vector3d &plane_origin);

  /**
   * Apply a lean angle to a pose
   * @param pose to be modified
   * @param lean_angle_axis
   * @param angle_value in radians
   */
  void applyLeanAngle(Eigen::Isometry3d &pose,
                      const AXIS_OF_ROTATION lean_angle_axis,
                      const double angle_value);

  /**
   * Generate robot poses along a line. Uses the stripper normal to generate the Z axis.
   * The X axis is generated from the current point to the next point.
   * @param[in] line
   * @param[out] trajectory
   * @return True if successful, false otherwise
   */
  bool generateRobotPosesAlongStripper(const vtkSmartPointer<vtkStripper> &line,
                                       EigenSTL::vector_Isometry3d &trajectory);

  /**
   * Invert the X axis orientation by inverting the X and Y vectors of the pose
   * @param[in] line to be modified
   */
  void invertXAxisOfPoses(EigenSTL::vector_Isometry3d &line);

  /**
   * Filters points that are too close from each others
   * @param line the line to be filtered, it contains the point and the normal
   * @param minimal_distance the minimal accepted distance between two consecutive points
   */
  void filterNeighborPosesTooClose(BezierPointNormalTable &line,
                                   const double minimal_distance);

  /**
   * Filter extrications lines to remove the unwanted beginning and end points
   * @param[in] first_point is the last point of the grinding line N
   * @param[in] first_point_normal is the normal of the last point of the grinding line N
   * @param[in] last_point is the first point of the grinding line N + 1
   * @param[in] last_point_normal is the normal of the first point of the grinding line N + 1
   * @param[in] plane_equation is the equation of the plane used to slice the mesh thus generating the extrication trajectory
   * @param[in,out] trajectory the trajectory to be filtered
   * @return True if successful, false otherwise
   */
  bool filterExtricationTrajectory(const Eigen::Vector3d &first_point,
                                   const Eigen::Vector3d &first_point_normal,
                                   const Eigen::Vector3d &last_point,
                                   const Eigen::Vector3d &last_point_normal,
                                   const Eigen::Vector4d &plane_equation,
                                   EigenSTL::vector_Isometry3d &trajectory);

  /**
   * Harmonize grinding/extrication lines orientation.
   * @param[in,out] poses is a vector of poses on a line
   * @param[in] direction_ref is direction vector reference
   * @return True if line was reversed, false otherwise
   */
  bool harmonizeLineOrientation(EigenSTL::vector_Isometry3d &poses,
                                const Eigen::Vector3d &direction_ref);

  /**
   * Slice a mesh with a plane and get the output stripper
   * @param[in] polydata
   * @param[in] plane_equation
   * @param[in] origin
   * @param[out] stripper
   * @return True if successful, false otherwise
   * @warning Returns true even if the stripper is empty!
   */
  bool sliceMeshWithPlane(const vtkSmartPointer<vtkPolyData> &polydata,
                          const Eigen::Vector4d &plane_equation,
                          const Eigen::Vector3d &origin,
                          vtkSmartPointer<vtkStripper> &stripper);

  /**
   * Slice a mesh with multiple planes and fill a vector of strippers
   * @param[in] polydata
   * @param[in] plane_equations
   * @param[in] origin
   * @param[out] strippers
   * @return True if successful, false otherwise
   * @warning Empty strippers are not pushed into the vector!
   *
   */
  bool sliceMeshWithPlanes(const vtkSmartPointer<vtkPolyData> &polydata,
                           const EigenSTL::vector_Vector4d &plane_equations,
                           const Eigen::Vector3d &origin,
                           std::vector<vtkSmartPointer<vtkStripper> > &strippers);

  /**
   * Compute points and cells normals of a polydata
   * @param[in] polydata
   * @return True if successful, false otherwise
   */
  bool computeNormals(vtkSmartPointer<vtkPolyData> &polydata);

  /** @brief This function uses vtkImplicitModeller in order to dilate the polydata surface/mesh
   *  @param[in,out] polydata the mesh to be dilated
   *  @param[in] radius of the dilation
   *  @return True if successful, false otherwise
   *  @bug Dilation problem happens when the radius is too high, the dilated mesh has unexpected holes.
   */
  bool
  dilate(vtkSmartPointer<vtkPolyData> &polydata,
         const double radius);

  /**
   * This removes all points "below" the dilated mesh using the base mesh.\n
   * During the dilation of a surface the mesh is expanded across all orientations, this means the
   * result is a closed mesh. In our process we only are interested in the upper part of the mesh.\n
   * A k-d tree is built and scalar products allows to determine if the current cell (= face) is
   * "above" or "below" the mesh.
   * @param[in] base_polydata_ the normals are computed in the polydata
   * @param[in] dilated_polydata the mesh to be processed
   * @param[out] upper_part_dilated_polydata the resulting mesh
   * @return True if successful, false otherwise
   * @note This probably should be implemented in VTK rather than filtered afterwards!
   */
  bool keepUpperPartofDilatedMesh(vtkSmartPointer<vtkPolyData> &base_polydata_,
                                  const vtkSmartPointer<vtkPolyData> &dilated_polydata,
                                  vtkSmartPointer<vtkPolyData> &upper_part_dilated_polydata);

  /**
   * This function clean a polydata by deleting all set of triangles which have a total number of cells
   * inferior to a minimum number specified by parameter
   * @param [in,out] polydata is the mesh to be cleaned
   * @param [in] minimal_number_of_cells a set of triangles has to match to pass the filter
   * @return True if successful, false otherwise
   */
  bool removeIsolatedTrianglesFilter(vtkSmartPointer<vtkPolyData> &polydata, const unsigned minimal_number_of_cells);

  /**
   * Get an rviz_visual_tool color given an index
   * @param[in] index of the color
   * @returns a color enum from rviz_visual_tools corresponding to the index
   */
  rviz_visual_tools::colors visualToolsColorFromIndex(const unsigned index);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif

