#include "bezier/tire_machining.hpp"
#include <vtkCleanPolyData.h>
#include <vtkWarpVector.h>

BezierTireMachining::BezierTireMachining(const std::string input_mesh,
                                         const double tool_length,
                                         const double tool_radius,
                                         const unsigned covering_percentage,
                                         const double lean_angle,
                                         const AXIS_OF_ROTATION axis_of_rotation,
                                         const Eigen::Vector3d &slicing_orientation) :
        Bezier(),
        tool_length_(tool_length),
        tool_radius_(tool_radius),
        covering_percentage_(covering_percentage),
        lean_angle_(lean_angle),
        axis_of_rotation_(axis_of_rotation),
        slicing_orientation_(slicing_orientation)
{
  if (!appendInputMesh(input_mesh))
  {
    AppendBezierException load_mesh_exception("BezierLibrary::appendInputMesh: Cannot load input mesh");
    throw load_mesh_exception;
  }
  input_mesh_absolute_path_ = input_mesh;
}

BezierTireMachining::BezierTireMachining(const std::string input_mesh) :
        BezierTireMachining(input_mesh, 0, 0, 0, M_PI, Z, Eigen::Vector3d::Identity())
{
}

void BezierTireMachining::setMeshesPublishers(std::shared_ptr<ros::Publisher> &input_mesh_publisher,
                                              std::shared_ptr<ros::Publisher> &dilated_mesh_publisher)
{
  input_mesh_pub_ = input_mesh_publisher;
  dilated_mesh_pub_ = dilated_mesh_publisher;

  if (input_mesh_pub_)
    displayMesh(input_mesh_pub_, std::string("file://" + input_mesh_absolute_path_), 0.3, 0.2, 0.2);
}

std::string BezierTireMachining::generateTrajectory(EigenSTL::vector_Isometry3d &trajectory,
                                                    std::vector<bool> &is_machining_pose,
                                                    const bool display_markers)
{
  visual_tools_->deleteAllMarkers();
  std::string error_string;

  // Make sure all parameters allows to generate a trajectory
  error_string = validateParameters();
  if (!error_string.empty())
    return error_string;

  dilated_mesh = vtkSmartPointer<vtkPolyData>::New();
  dilated_mesh->DeepCopy(input_meshes_[TIRE_PATCH_MESH]);

  // Estimate a slicing orientation if it was not provided
  // Otherwise, use the last/provided slicing orientation
  Eigen::Vector3d global_mesh_normal;
  if (!estimateSlicingOrientation(dilated_mesh, global_mesh_normal, slicing_orientation_))
    return "Error estimating slicing orientation and global mesh normal";

  vtkSmartPointer<vtkCleanPolyData> clean =
      vtkSmartPointer<vtkCleanPolyData>::New();
  clean->SetInputData(dilated_mesh);

  // Generate normals
  vtkSmartPointer<vtkPolyDataNormals> normals =
      vtkSmartPointer<vtkPolyDataNormals>::New();
  normals->SetInputConnection(clean->GetOutputPort());
  normals->SplittingOff();

  // Warp using the normals
  vtkSmartPointer<vtkWarpVector> warp =
      vtkSmartPointer<vtkWarpVector>::New();
  warp->SetInputConnection(normals->GetOutputPort());
  warp->SetInputArrayToProcess(0, 0, 0,
                               vtkDataObject::FIELD_ASSOCIATION_POINTS,
                               vtkDataSetAttributes::NORMALS);
  warp->SetScaleFactor(tool_radius_); // 5 cm
  warp->Update();

  dilated_mesh = warp->GetPolyDataOutput();

  std::string mesh_path(ros::package::getPath("michelin_tire_machining_path_planning") + "/meshes/dilated_mesh.ply");
  if (!saveMesh(mesh_path, dilated_mesh))
  {
    ROS_WARN_STREAM(
        "BezierTireMachining::generateTrajectory: Could not save the mesh, aborting visualization of the dilated mesh!");
  }

  if (dilated_mesh_pub_)
    displayMesh(dilated_mesh_pub_, std::string("file://" + mesh_path), 0.1, 0.1, 0.1, 0.5);

  if (!computeNormals(dilated_mesh))
    return "Error computing normals in dilated mesh";

  if (!computeNormals(input_meshes_[TIRE_PATCH_MESH]))
    return "Error computing normals in tire patch mesh";

  // Compute the plane equations to slice the mesh
  EigenSTL::vector_Vector4d machining_planes_equations;
  Eigen::Vector3d input_mesh_centroid(input_meshes_[TIRE_PATCH_MESH]->GetCenter());
  estimateSlicingPlanes(dilated_mesh, slicing_orientation_, input_mesh_centroid,
                        tool_length_,
                        covering_percentage_, machining_planes_equations);

  // Generate all machining trajectories
  std::vector<vtkSmartPointer<vtkStripper> > machining_strippers;
  if (!sliceMeshWithPlanes(dilated_mesh, machining_planes_equations, input_mesh_centroid,
                           machining_strippers))
    return "Could not slice polydata for machining trajectories";

  // Compute the vector used to harmonize all the trajectories directions and display it
  Eigen::Vector3d direction_reference(slicing_orientation_.cross(global_mesh_normal));
  if (direction_reference.x() > 0)
    direction_reference = -direction_reference;

  Eigen::Isometry3d pose_dir_reference(Eigen::Isometry3d::Identity());
  double centroid[3];
  centroid[0] = dilated_mesh->GetCenter()[0];
  centroid[1] = dilated_mesh->GetCenter()[1];
  centroid[2] = dilated_mesh->GetCenter()[2];
  pose_dir_reference.translation() << centroid[0], centroid[1], centroid[2];
  pose_dir_reference.affine().col(0).head<3>() << direction_reference;
  pose_dir_reference.affine().col(2).head<3>() << direction_reference[2], 0, -direction_reference[0];
  pose_dir_reference.affine().col(1).head<3>()
  << pose_dir_reference.affine().col(2).head<3>().cross(pose_dir_reference.affine().col(0).head<3>());
  visual_tools_->publishArrow(pose_dir_reference, rviz_visual_tools::GREEN, rviz_visual_tools::XXSMALL);
  pose_dir_reference.translation() += 0.07 * pose_dir_reference.affine().col(0).head<3>();
  visual_tools_->publishText(pose_dir_reference, "Direction reference", rviz_visual_tools::GREEN,
                             rviz_visual_tools::SMALL,
                             false);

  std::vector<EigenSTL::vector_Isometry3d> machining_trajectories;
  for (std::vector<vtkSmartPointer<vtkStripper> >::iterator it(machining_strippers.begin());
      it != machining_strippers.end(); ++it)
  {
      if (it->Get()->GetOutput()->GetNumberOfLines() > 1)
      {
        ROS_ERROR_STREAM(
            "BezierTireMachining::generateTrajectory: Machining stripper has " << it->Get()->GetOutput()->GetNumberOfLines() << " lines");
        // FIXME Handle this case!
        //return "Machining stripper has more than 1 line (mesh has a hole). Not implemented yet!";
        continue;
      }

    EigenSTL::vector_Isometry3d traj;
    if (!generateRobotPosesAlongStripper(*it, traj))
    {
      ROS_WARN_STREAM(
          "BezierTireMachining::generateTrajectory: Could not generate robot poses for machining trajectory");
      continue;
    }

    // Reset orientation
    for (auto &pose : traj)
    {
      Eigen::Matrix3d rot;
      rot << 0, -1, 0,
          -1, 0, 0,
          0, 0, -1;

      Eigen::Vector3d direction;
      if (axis_of_rotation_ == X)
        direction = Eigen::Vector3d::UnitX();
      else if (axis_of_rotation_ == Y)
        direction = Eigen::Vector3d::UnitY();
      else
        direction = Eigen::Vector3d::UnitZ();
      pose.linear() = rot * Eigen::AngleAxisd(lean_angle_, direction);

      // Lean forward
      pose.linear() = pose.linear() * Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY());
    }

    harmonizeLineOrientation(traj, direction_reference);
    //ROS_INFO_STREAM("BezierTireMachining::generateTrajectory: Machining line reversed");

    machining_trajectories.push_back(traj);
  }

  trajectory.clear();
  is_machining_pose.clear();
  is_machining_pose.push_back(false);

  unsigned reverse_index(0);
  for (EigenSTL::vector_Isometry3d & machining_traj : machining_trajectories)
  {
    // Zig Zag trajectory
    if (reverse_index % 2 != 0)
    {
      std::reverse(machining_traj.begin(), machining_traj.end());
      // Do NOT modify orientation!
    }
    reverse_index++;
    for (Eigen::Isometry3d machining_pose : machining_traj)
    {
      trajectory.push_back(machining_pose);
      is_machining_pose.push_back(true);
    }
  }

  unsigned index(0);
  if (display_markers)
  {
    for (EigenSTL::vector_Isometry3d traj : machining_trajectories)
    {
      if (index > 11)
        index = 0;
      displayTrajectory(traj, visualToolsColorFromIndex(index++), true);
    }
  }

  return "";
}

void BezierTireMachining::setSlicingOrientation(const Eigen::Vector3d &cutting_plane_normal)
{
  slicing_orientation_ = cutting_plane_normal;

  if (slicing_orientation_ != Eigen::Vector3d::Zero())
    slicing_orientation_.normalize();
}

std::string
BezierTireMachining::validateParameters()
{
  if (tool_length_ <= 0.0)
    return "Tool length must be > 0";

  if (tool_radius_ <= 0.0)
    return "Tool radius must be > 0";

  if (covering_percentage_ > 99)
    return "Covering percentage must be lower than 100%";
  return "";
}
