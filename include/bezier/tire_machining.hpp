#ifndef BEZIER_TIRE_MACHINING_HPP
#define BEZIER_TIRE_MACHINING_HPP

#include <bezier/bezier_library.hpp>
#include <ros/package.h>
#include <memory>

/**
 * Bezier tire machining class
 */
class BezierTireMachining : public Bezier
{

public:

  enum INPUT_MESHES
  {
    TIRE_PATCH_MESH
  };

  BezierTireMachining(const std::string input_mesh,
                      const double tool_length,
                      const double tool_radius,
                      const unsigned covering_percentage,
                      const double lean_angle,
                      const AXIS_OF_ROTATION axis_of_rotation = Bezier::AXIS_OF_ROTATION::Y,
                      const Eigen::Vector3d &slicing_orientation = Eigen::Vector3d::Zero());

  BezierTireMachining(const std::string input_mesh);

  virtual ~BezierTireMachining()
  {
  }

  std::string name()
  {
    return "BezierTireMachining";
  }

  void setMeshesPublishers(std::shared_ptr<ros::Publisher> &input_mesh_publisher,
                           std::shared_ptr<ros::Publisher> &dilated_mesh_publisher);

  std::string generateTrajectory(EigenSTL::vector_Isometry3d &trajectory,
                                 std::vector<bool> &is_grinding_pose,
                                 const bool display_markers = true);

  std::string generateTrajectory(EigenSTL::vector_Isometry3d &trajectory,
                                 std::vector<bool> &is_grinding_pose,
                                 const double tool_length,
                                 const double tool_radius,
                                 const unsigned covering_percentage,
                                 const double lean_angle = 0.0,
                                 const Bezier::AXIS_OF_ROTATION axis_of_rotation = Bezier::AXIS_OF_ROTATION::Y,
                                 const Eigen::Vector3d &slicing_orientation = Eigen::Vector3d::Zero(),
                                 const bool display_markers = true)
  {
    tool_length_ = tool_length;
    tool_radius_ = tool_radius;
    covering_percentage_ = covering_percentage;
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

private:
  std::string
  validateParameters();

  // Parameters
  double tool_length_;
  double tool_radius_;
  unsigned covering_percentage_;
  double lean_angle_;
  Bezier::AXIS_OF_ROTATION axis_of_rotation_;

  // Internals
  Eigen::Vector3d slicing_orientation_;
  std::string input_mesh_absolute_path_;
  std::shared_ptr<ros::Publisher> input_mesh_pub_;
  std::shared_ptr<ros::Publisher> dilated_mesh_pub_;

  vtkSmartPointer<vtkPolyData> dilated_mesh;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

#endif
