#ifndef BEZIER_GRINDING_SURFACING_HPP
#define BEZIER_GRINDING_SURFACING_HPP

#include <bezier_library/bezier_library.hpp>
#include <ros/package.h>
#include <memory>

/**
 * Bezier grinding surfacing class
 */
class BezierGrindingSurfacing : public Bezier
{
public:
  typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> PointNormal;
  typedef std::vector<PointNormal> BezierPointNormalTable;

  enum INPUT_MESHES
  {
    SURFACE_MESH
  };

  enum AXIS_OF_ROTATION
  {
    X,
    Y,
    Z
  };

  BezierGrindingSurfacing(const std::string input_mesh,
                          const double grinding_disk_machining_width,
                          const unsigned covering_percentage,
                          const double extrication_radius,
                          const double lean_angle = 0.0,
                          const AXIS_OF_ROTATION axis_of_rotation = AXIS_OF_ROTATION::Y,
                          const Eigen::Vector3d &slicing_orientation = Eigen::Vector3d::Zero());

  BezierGrindingSurfacing(const std::string input_mesh);

  virtual ~BezierGrindingSurfacing()
  {
  }

  std::string name()
  {
    return "BezierGrindingSurfacing";
  }

  /**
   * Allows to publish the input mesh and the dilated mesh
   * @param input_mesh_publisher
   * @param dilated_mesh_publisher
   * @note If not called, no mesh will be displayed
   */
  void setMeshesPublishers(std::shared_ptr<ros::Publisher> &input_mesh_publisher,
                           std::shared_ptr<ros::Publisher> &dilated_mesh_publisher);

  std::string generateTrajectory(EigenSTL::vector_Affine3d &trajectory,
                                 std::vector<bool> &is_grinding_pose,
                                 const bool display_markers = true);

  std::string generateTrajectory(EigenSTL::vector_Affine3d &trajectory,
                                 std::vector<bool> &is_grinding_pose,
                                 const double grinding_disk_machining_width,
                                 const unsigned covering_percentage,
                                 const double extrication_radius,
                                 const double lean_angle = 0.0,
                                 const AXIS_OF_ROTATION axis_of_rotation = AXIS_OF_ROTATION::Y,
                                 const Eigen::Vector3d &slicing_orientation = Eigen::Vector3d::Zero(),
                                 const bool display_markers = true)
  {
    grinding_disk_machining_width_ = grinding_disk_machining_width;
    covering_percentage_ = covering_percentage;
    extrication_radius_ = extrication_radius;
    axis_of_rotation_ = axis_of_rotation;
    lean_angle_ = lean_angle;
    setSlicingOrientation(slicing_orientation);
    return (generateTrajectory(trajectory, is_grinding_pose, display_markers));
  }

  /**
   * Force the cutting plane orientation, when set the automatic cutting orientation estimation is skipped.\n
   * The orientation provided is normalized.
   * @param[in] cutting_plane_normal
   * @note To restore the automatic estimation behavior, use setAutomaticCuttingOrientationEstimation
   */
  void setSlicingOrientation(const Eigen::Vector3d &cutting_plane_normal);

  /**
   * Force the cutting plane orientation automatic estimation
   */
  void setAutomaticSlicingOrientationEstimation()
  {
    setSlicingOrientation(Eigen::Vector3d::Zero());
  }

  /**
   * Computes all the plane equations we need to slice the whole part given the polydata and the covering percentage.\n
   * This function computes the bound of the polydata and determines the number of slices using covering_percentage_\n
   * The planes equations are ordered from one side to another (plane @c n+1 is a neighbor of plane @c n)
   * @param[in] polydata
   * @param[in] slicing_orientation
   * @param[in] polydata_center the origin used to compute all the planes
   * @param[out] planes_equations the equations of the planes
   */
  void estimateGrindingSlicingPlanes(const vtkSmartPointer<vtkPolyData> &polydata,
                                     const Eigen::Vector3d &slicing_orientation,
                                     const Eigen::Vector3d &polydata_center,
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
  void applyLeanAngle(Eigen::Affine3d &pose,
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
                                       EigenSTL::vector_Affine3d &trajectory);

  /**
   * Invert the X axis orientation by inverting the X and Y vectors of the pose
   * @param[in] line to be modified
   */
  void invertXAxisOfPoses(EigenSTL::vector_Affine3d &line);

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
                                   EigenSTL::vector_Affine3d &trajectory);

  /**
   * Harmonize grinding/extrication lines orientation.
   * @param[in,out] poses is a vector of poses on a line
   * @param[in] direction_ref is direction vector reference
   * @return True if line was reversed, false otherwise
   */
  bool harmonizeLineOrientation(EigenSTL::vector_Affine3d &poses,
                                const Eigen::Vector3d &direction_ref);

  /**
   * Generate a set of points on a line between a start and a end point
   * @param[out] poses is a vector of poses generated on a line
   * @param[in] start_point is the start point of the line
   * @param[in] end_point is the end point of the line
   * @param[in] number_of_poses is the number of poses to be generated
   */
  void generateIntermediatePoseOnLine(EigenSTL::vector_Affine3d &poses,
                                      const Eigen::Vector3d &start_point,
                                      const Eigen::Vector3d &end_point,
                                      const unsigned number_of_poses);

private:
  std::string
  validateParameters();

  // Parameters
  double grinding_disk_machining_width_;
  unsigned covering_percentage_;
  double extrication_radius_;
  double lean_angle_;
  AXIS_OF_ROTATION axis_of_rotation_;

  // Internals
  Eigen::Vector3d slicing_orientation_;
  std::string input_mesh_absolute_path_;
  std::shared_ptr<ros::Publisher> input_mesh_pub_;
  std::shared_ptr<ros::Publisher> dilated_mesh_pub_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif

