#ifndef BEZIER_GRINDING_SURFACING_HPP
#define BEZIER_GRINDING_SURFACING_HPP

#include <bezier_library/bezier_library.hpp>
#include <bezier_library/bezier_params_observer.hpp>
#include <ros/package.h>
#include <memory>

/**
 * Bezier grinding surfacing class
 */
class BezierGrindingSurfacing : public Bezier
{

public:

  enum INPUT_MESHES
    {
      SURFACE_MESH
    };

  BezierGrindingSurfacing(const std::string input_mesh,
                          const double grinding_disk_machining_width,
                          const unsigned covering_percentage,
                          const double extrication_radius,
                          const double lean_angle = 0.0,
                          const Bezier::AXIS_OF_ROTATION axis_of_rotation = Bezier::AXIS_OF_ROTATION::Y,
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

  std::string generateTrajectory(EigenSTL::vector_Isometry3d &trajectory,
                                 std::vector<bool> &is_grinding_pose,
                                 const bool display_markers = true);

  std::string generateTrajectory(EigenSTL::vector_Isometry3d &trajectory,
                                 std::vector<bool> &is_grinding_pose,
                                 const double grinding_disk_machining_width,
                                 const unsigned covering_percentage,
                                 const double extrication_radius,
                                 const double lean_angle = 0.0,
                                 const Bezier::AXIS_OF_ROTATION axis_of_rotation = Bezier::AXIS_OF_ROTATION::Y,
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
   * Generate a set of points on a line between a start and a end point
   * @param[out] poses is a vector of poses generated on a line
   * @param[in] start_point is the start point of the line
   * @param[in] end_point is the end point of the line
   * @param[in] number_of_poses is the number of poses to be generated
   */
  void generateIntermediatePoseOnLine(EigenSTL::vector_Isometry3d &poses,
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
  Bezier::AXIS_OF_ROTATION axis_of_rotation_;

  // Internals
  std::shared_ptr<BezierParamsObserver> params_observer_;
  Eigen::Vector3d slicing_orientation_;
  Eigen::Vector3d input_mesh_centroid_;
  std::string input_mesh_absolute_path_;
  std::shared_ptr<ros::Publisher> input_mesh_pub_;
  std::shared_ptr<ros::Publisher> dilated_mesh_pub_;

  // PolyData
  vtkSmartPointer<vtkPolyData> dilated_mesh;
  vtkSmartPointer<vtkPolyData> extrication_mesh;

  // Vector of generated poses (trajectories)
  std::vector<EigenSTL::vector_Isometry3d> grinding_trajectories;
  std::vector<EigenSTL::vector_Isometry3d> extrication_trajectories;
  std::vector<EigenSTL::vector_Isometry3d> start_intermediate_poses_trajectories;
  std::vector<EigenSTL::vector_Isometry3d> end_intermediate_poses_trajectories;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif

