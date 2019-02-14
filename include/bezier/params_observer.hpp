#ifndef BEZIER_PARAMS_OBSERVER_HPP
#define BEZIER_PARAMS_OBSERVER_HPP

#include <bezier/bezier_library.hpp>

/**
 * Bezier parameters observer class
 * This class allows to reduce computations done by the bezier library.
 * It achieves that by comparing parameters of the library each time a trajectory generation
 * is required. If the trajectory haven't change between two computation, it informs the trajectory
 * generation function so the concerned operation is skipped.
 */

class BezierParamsObserver
{

public:

  BezierParamsObserver()
  {
    // When a BezierParamObserver object is instantiated, all the values are set to their maximum
    // numeric limits. This make sure that all the computations are done the first time the observer is requested.
    // After a first computation is performed, the observer will keep in memory those first parameters and
    // will start comparing them with their new counterpart when other computations are performed.
    setCurrentParams(std::numeric_limits<double>::max(), std::numeric_limits<unsigned>::max(),
                     std::numeric_limits<double>::max(), Eigen::Vector3d::Zero(), std::numeric_limits<double>::max(),
                     std::numeric_limits<int>::max());
  }

  virtual ~BezierParamsObserver()
  {
  }

  std::string name()
  {
    return "BezierParamsObserver";
  }

  void inline setCurrentGrindingDiskWidth(const double current_grinding_disk_machining_width)
  {
    grinding_disk_machining_width_.first = grinding_disk_machining_width_.second;
    grinding_disk_machining_width_.second = current_grinding_disk_machining_width;
  }

  void inline setCurrentCoveringPercentage(const unsigned current_covering_percentage)
  {
    covering_percentage_.first = covering_percentage_.second;
    covering_percentage_.second = current_covering_percentage;
  }

  void inline setCurrentExtricationRadius(const double current_extrication_radius)
  {
    extrication_radius_.first = extrication_radius_.second;
    extrication_radius_.second = current_extrication_radius;
  }

  void inline setCurrentSlicingDirection(const Eigen::Vector3d current_slicing_orientation)
  {
    slicing_orientation_.first = slicing_orientation_.second;
    slicing_orientation_.second = current_slicing_orientation;
  }

  void inline setCurrentLeanAngle(const double lean_angle)
  {
    lean_angle_.first = lean_angle_.second;
    lean_angle_.second = lean_angle;
  }

  void inline setCurrentAxisOfRotation(const int axis_of_rotation)
  {
    axis_of_rotation_.first = axis_of_rotation_.second;
    axis_of_rotation_.second = axis_of_rotation;
  }

  void setCurrentParams(const double grinding_disk_machining_width, const unsigned covering_percentage,
                        const double extrication_radius, const Eigen::Vector3d slicing_orientation,
                        const double lean_angle, const int axis_of_rotation)
  {
    setCurrentGrindingDiskWidth(grinding_disk_machining_width);
    setCurrentCoveringPercentage(covering_percentage);
    setCurrentExtricationRadius(extrication_radius);
    setCurrentSlicingDirection(slicing_orientation);
    setCurrentLeanAngle(lean_angle);
    setCurrentAxisOfRotation(axis_of_rotation);
  }

  bool inline mustComputeDilation()
  {
    return extrication_radius_.first != extrication_radius_.second;
  }

  bool inline mustGenerateGrindingTrajectories()
  {
    return grinding_disk_machining_width_.first != grinding_disk_machining_width_.second
        || covering_percentage_.first != covering_percentage_.second
        || slicing_orientation_.first != slicing_orientation_.second || lean_angle_.first != lean_angle_.second
        || axis_of_rotation_.first != axis_of_rotation_.second;
  }

  bool inline mustGenerateExtricationTrajectories()
  {
    return mustComputeDilation() || mustGenerateGrindingTrajectories();
  }

  bool inline mustRecomputeTrajectoryVector()
  {
    return mustGenerateExtricationTrajectories() || mustGenerateGrindingTrajectories();
  }

private:
  // A std::pair is used to store a parameter to test. The first value of the pair
  // is the parameter used for the last computation and the second value is the
  // parameter for the current computation.
  std::pair<double, double> grinding_disk_machining_width_;
  std::pair<unsigned, unsigned> covering_percentage_;
  std::pair<double, double> extrication_radius_;
  std::pair<Eigen::Vector3d, Eigen::Vector3d> slicing_orientation_;
  std::pair<double, double> lean_angle_;
  std::pair<int, int> axis_of_rotation_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
