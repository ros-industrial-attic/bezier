#include "bezier_library/bezier_painting.hpp"

BezierPainting::BezierPainting(const std::string input_mesh,
                               const double painting_cone_width,
                               const unsigned covering_percentage,
                               const double extrication_radius,
                               const double lean_angle,
                               const AXIS_OF_ROTATION axis_of_rotation,
                               const Eigen::Vector3d &slicing_orientation) :
        Bezier(),
        painting_cone_width_(painting_cone_width),
        covering_percentage_(covering_percentage),
        extrication_radius_(extrication_radius),
        lean_angle_(lean_angle),
        axis_of_rotation_(axis_of_rotation),
        slicing_orientation_(slicing_orientation)
{
  if(!appendInputMesh(input_mesh))
  {
    AppendBezierException load_mesh_exception("BezierLibrary::appendInputMesh: Cannot load input mesh");
    throw load_mesh_exception;
  }
  input_mesh_absolute_path_ = input_mesh;
}

BezierPainting::BezierPainting(const std::string input_mesh) :
    BezierPainting(input_mesh, 0, 0, 0, 0, Y, Eigen::Vector3d::Identity())
{
}

void BezierPainting::setMeshesPublishers(std::shared_ptr<ros::Publisher> &input_mesh_publisher,
                                         std::shared_ptr<ros::Publisher> &dilated_mesh_publisher)
{
  input_mesh_pub_ = input_mesh_publisher;
  dilated_mesh_pub_ = dilated_mesh_publisher;

  if (input_mesh_pub_)
    displayMesh(input_mesh_pub_, std::string("file://" + input_mesh_absolute_path_), 0.3, 0.2, 0.2);
}

std::string BezierPainting::generateTrajectory(EigenSTL::vector_Affine3d &trajectory,
                                               std::vector<bool> &is_grinding_pose,
                                               const bool display_markers)
{
  visual_tools_->deleteAllMarkers();
  std::string error_string;

  // Make sure all parameters allows to generate a trajectory
  error_string = validateParameters();
  if (!error_string.empty())
    return error_string;

  // Estimate a slicing orientation if it was not provided
  // Otherwise, use the last/provided slicing orientation
  Eigen::Vector3d global_mesh_normal;
  if (!estimateSlicingOrientation(input_meshes_[PAINTING_MESH], global_mesh_normal, slicing_orientation_))
    return "Error estimating slicing orientation and global mesh normal";

  // Useful if using a scan mesh
  /*if (!removeIsolatedTrianglesFilter(input_meshes_[PAINTING_MESH], 20))
   return "Failed to filter isolated triangles in painting mesh";*/

  // Compute normals
  if (!computeNormals(input_meshes_[PAINTING_MESH]))
    return "Error computing normals in surface mesh";

  // Compute the plane equations to slice the mesh
  EigenSTL::vector_Vector4d grinding_planes_equations;
  Eigen::Vector3d input_mesh_centroid(input_meshes_[PAINTING_MESH]->GetCenter());
  estimateGrindingSlicingPlanes(input_meshes_[PAINTING_MESH], slicing_orientation_, input_mesh_centroid,
                                painting_cone_width_, covering_percentage_, grinding_planes_equations);

  // Generate all grinding trajectories
  std::vector<vtkSmartPointer<vtkStripper> > grinding_strippers;
  if (!sliceMeshWithPlanes(input_meshes_[PAINTING_MESH], grinding_planes_equations, input_mesh_centroid,
                           grinding_strippers))
    return "Could not slice polydata for grinding trajectories";

  for (vtkSmartPointer<vtkStripper> stripper : grinding_strippers)
  {
    if (stripper->GetOutput()->GetNumberOfLines() > 1)
    {
      ROS_ERROR_STREAM(
          "BezierPainting::generateTrajectory: Grinding stripper has " << stripper->GetOutput()->GetNumberOfLines() << " lines");
      // FIXME Handle this case!
      //return "Grinding stripper has more than 1 line (mesh has a hole). Not implemented yet!";
    }
  }

  // Compute the vector used to harmonize all the trajectories directions and display it
  Eigen::Vector3d direction_reference(slicing_orientation_.cross(global_mesh_normal));
  Eigen::Affine3d pose_dir_reference(Eigen::Affine3d::Identity());
  double centroid[3];
  centroid[0] = input_meshes_[PAINTING_MESH]->GetCenter()[0];
  centroid[1] = input_meshes_[PAINTING_MESH]->GetCenter()[1];
  centroid[2] = input_meshes_[PAINTING_MESH]->GetCenter()[2];
  pose_dir_reference.translation() << centroid[0], centroid[1], centroid[2];
  pose_dir_reference.affine().col(0).head<3>() << direction_reference;
  pose_dir_reference.affine().col(2).head<3>() << direction_reference[2], 0, -direction_reference[0];
  pose_dir_reference.affine().col(1).head<3>()
      << pose_dir_reference.affine().col(2).head<3>().cross(pose_dir_reference.affine().col(0).head<3>());
  visual_tools_->publishArrow(pose_dir_reference, rviz_visual_tools::GREEN, rviz_visual_tools::XXSMALL);
  pose_dir_reference.translation() += 0.07 * pose_dir_reference.affine().col(0).head<3>();
  visual_tools_->publishText(pose_dir_reference, "Direction reference", rviz_visual_tools::GREEN,
                             rviz_visual_tools::SMALL, false);

  std::vector<EigenSTL::vector_Affine3d> grinding_trajectories;
  for (std::vector<vtkSmartPointer<vtkStripper> >::iterator it(grinding_strippers.begin());
      it != grinding_strippers.end(); ++it)
  {
    EigenSTL::vector_Affine3d traj;
    if (!generateRobotPosesAlongStripper(*it, traj))
    {
      ROS_WARN_STREAM("BezierPainting::generateTrajectory: Could not generate robot poses for grinding trajectory");
      continue;
    }

    if (harmonizeLineOrientation(traj, direction_reference))
      ROS_INFO_STREAM("BezierPainting::generateTrajectory: Grinding line reversed");

    grinding_trajectories.push_back(traj);
  }

  trajectory.clear();
  is_grinding_pose.clear();
  is_grinding_pose.push_back(false);

  unsigned reverse_index(0);
  for (EigenSTL::vector_Affine3d & grinding_traj : grinding_trajectories)
  {
    // Zig Zag trajectory
    if (reverse_index % 2 != 0)
    {
      std::reverse(grinding_traj.begin(), grinding_traj.end());
      invertXAxisOfPoses(grinding_traj);
    }
    reverse_index++;
    for (Eigen::Affine3d grinding_pose : grinding_traj)
    {
      trajectory.push_back(grinding_pose);
      is_grinding_pose.push_back(true);
    }
  }

  unsigned index(0);
  if (display_markers)
  {
    for (EigenSTL::vector_Affine3d traj : grinding_trajectories)
    {
      if (index > 11)
        index = 0;
      displayTrajectory(traj, visualToolsColorFromIndex(index++), true);
    }
  }

  return "";
}

void BezierPainting::setSlicingOrientation(const Eigen::Vector3d &cutting_plane_normal)
{
  slicing_orientation_ = cutting_plane_normal;

  if (slicing_orientation_ != Eigen::Vector3d::Zero())
    slicing_orientation_.normalize();
}

std::string BezierPainting::validateParameters()
{
  if (painting_cone_width_ <= 0.0)
    return "Painting cone diameter must be > 0";

  if (covering_percentage_ > 99)
    return "Covering percentage must be lower than 100%";
  return "";
}
