#include "bezier_library/bezier_grinding_surfacing.hpp"
#include "bezier_library/bezier_params_observer.hpp"

BezierGrindingSurfacing::BezierGrindingSurfacing(const std::string input_mesh,
                                                 const double grinding_disk_machining_width,
                                                 const unsigned covering_percentage,
                                                 const double extrication_radius,
                                                 const double lean_angle,
                                                 const AXIS_OF_ROTATION axis_of_rotation,
                                                 const Eigen::Vector3d &slicing_orientation) :
        Bezier(),
        grinding_disk_machining_width_(grinding_disk_machining_width),
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
  double * centroid = input_meshes_[SURFACE_MESH]->GetCenter();
  input_mesh_centroid_ << centroid[0], centroid[1], centroid[2];
  params_observer_.reset(new BezierParamsObserver());
}

BezierGrindingSurfacing::BezierGrindingSurfacing(const std::string input_mesh) :
        BezierGrindingSurfacing(input_mesh, 0, 0, 0, 0, Y, Eigen::Vector3d::Identity())
{
}

void BezierGrindingSurfacing::setMeshesPublishers(std::shared_ptr<ros::Publisher> &input_mesh_publisher,
                                                  std::shared_ptr<ros::Publisher> &dilated_mesh_publisher)
{
  input_mesh_pub_ = input_mesh_publisher;
  dilated_mesh_pub_ = dilated_mesh_publisher;

  if (input_mesh_pub_)
    displayMesh(input_mesh_pub_, std::string("file://"+input_mesh_absolute_path_), 0.3, 0.2, 0.2);
}

std::string BezierGrindingSurfacing::generateTrajectory(EigenSTL::vector_Isometry3d &trajectory,
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
  if (!estimateSlicingOrientation(input_meshes_[SURFACE_MESH], global_mesh_normal, slicing_orientation_))
    return "Error estimating slicing orientation and global mesh normal";

  params_observer_->setCurrentParams(grinding_disk_machining_width_, covering_percentage_, extrication_radius_,
                                    slicing_orientation_, lean_angle_, axis_of_rotation_);

  if (params_observer_->mustComputeDilation())
  {
    ROS_INFO_STREAM("BezierGrindingSurfacing::generateTrajectory: Dilating mesh");
    // Dilate the mesh using the extrication radius
    dilated_mesh = vtkSmartPointer<vtkPolyData>::New();
    dilated_mesh->DeepCopy(input_meshes_[SURFACE_MESH]);
    if (!dilate(dilated_mesh, extrication_radius_))
      return "Error dilating mesh";

    extrication_mesh = vtkSmartPointer<vtkPolyData>::New();
    if (!keepUpperPartofDilatedMesh(input_meshes_[SURFACE_MESH], dilated_mesh, extrication_mesh))
      return "Failed to extract upper part of dilated mesh";

    if (!removeIsolatedTrianglesFilter(extrication_mesh, 20))
      return "Failed to filter isolated triangles in extrication mesh";

    if (!computeNormals(extrication_mesh))
      return "Error computing normals in extrication mesh";
  }
  else
  {
    ROS_INFO_STREAM("BezierGrindingSurfacing::Parameters haven't changed, no need to dilate the mesh");
  }

  // Compute normals
  if (!computeNormals(input_meshes_[SURFACE_MESH]))
    return "Error computing normals in surface mesh";

  if (display_markers)
  {
    std::string mesh_path(ros::package::getPath("bezier_library") + "/meshes/dilated_mesh.stl");

    if (saveMesh(mesh_path, dilated_mesh))
    {
      if (dilated_mesh_pub_)
        displayMesh(dilated_mesh_pub_, std::string("file://" + mesh_path), 0.1, 0.1, 0.1, 0.5);
    }
    else
      ROS_WARN_STREAM(
          "BezierGrindingSurfacing::generateTrajectory: Could not save the mesh, aborting visualization of the dilated mesh!");
  }

  if (display_markers)
  {
    std::string mesh_path(ros::package::getPath("bezier_library") + "/meshes/extrication_mesh.stl");

    if (saveMesh(mesh_path, extrication_mesh))
    {
      if (dilated_mesh_pub_)
        displayMesh(dilated_mesh_pub_, std::string("file://" + mesh_path), 0.1, 0.1, 0.1, 0.5);
    }
    else
      ROS_WARN_STREAM(
          "BezierGrindingSurfacing::generateTrajectory: Could not save the mesh, aborting visualization of the extrication mesh!");
  }

  // Compute the vector used to harmonize all the trajectories directions and display it
  Eigen::Vector3d direction_reference(slicing_orientation_.cross(global_mesh_normal));
  Eigen::Isometry3d pose_dir_reference(Eigen::Isometry3d::Identity());
  double centroid[3];
  centroid[0] = input_meshes_[SURFACE_MESH]->GetCenter()[0];
  centroid[1] = input_meshes_[SURFACE_MESH]->GetCenter()[1];
  centroid[2] = input_meshes_[SURFACE_MESH]->GetCenter()[2];
  pose_dir_reference.translation() << centroid[0], centroid[1], centroid[2];
  pose_dir_reference.affine().col(0).head<3>() << direction_reference;
  pose_dir_reference.affine().col(2).head<3>() << direction_reference[2], 0, -direction_reference[0];
  pose_dir_reference.affine().col(1).head<3>() << pose_dir_reference.affine().col(2).head<3>().cross(pose_dir_reference.affine().col(0).head<3>());
  visual_tools_->publishArrow(pose_dir_reference, rviz_visual_tools::GREEN, rviz_visual_tools::XXSMALL);
  pose_dir_reference.translation() += 0.07 * pose_dir_reference.affine().col(0).head<3>();
  visual_tools_->publishText(pose_dir_reference, "Direction reference", rviz_visual_tools::GREEN, rviz_visual_tools::SMALL, false);

  if (params_observer_->mustGenerateGrindingTrajectories())
  {
    ROS_INFO_STREAM("BezierGrindingSurfacing::generateTrajectory: Generating grinding trajectories");
    // Compute the plane equations to slice the mesh
    EigenSTL::vector_Vector4d grinding_planes_equations;
    estimateSlicingPlanes(input_meshes_[SURFACE_MESH], slicing_orientation_, input_mesh_centroid_,
                                  grinding_disk_machining_width_, covering_percentage_, grinding_planes_equations);

    // Generate all grinding trajectories
    std::vector<vtkSmartPointer<vtkStripper> > grinding_strippers;
    if (!sliceMeshWithPlanes(input_meshes_[SURFACE_MESH], grinding_planes_equations, input_mesh_centroid_, grinding_strippers))
      return "Could not slice polydata for grinding trajectories";

    for (vtkSmartPointer<vtkStripper> stripper : grinding_strippers)
    {
      if (stripper->GetOutput()->GetNumberOfLines() > 1)
      {
        ROS_ERROR_STREAM(
            "BezierGrindingSurfacing::generateTrajectory: Grinding stripper has " << stripper->GetOutput()->GetNumberOfLines() << " lines");
        // FIXME Handle this case!
        return "Grinding stripper has more than 1 line (mesh has a hole). Not implemented yet!";
      }
    }

    grinding_trajectories.clear();
    for (std::vector<vtkSmartPointer<vtkStripper> >::iterator it(grinding_strippers.begin());
        it != grinding_strippers.end(); ++it)
    {
      EigenSTL::vector_Isometry3d traj;
      if (!generateRobotPosesAlongStripper(*it, traj))
      {
        ROS_WARN_STREAM(
            "BezierGrindingSurfacing::generateTrajectory: Could not generate robot poses for grinding trajectory");
        continue;
      }

      if (harmonizeLineOrientation(traj, direction_reference))
        ROS_INFO_STREAM("BezierGrindingSurfacing::generateTrajectory: Grinding line reversed");

      grinding_trajectories.push_back(traj);
    }
  }
  else
  {
    ROS_INFO_STREAM("BezierGrindingSurfacing::generateTrajectory: parameters haven't changed, skipping grinding trajectories generation");
  }

  if (params_observer_->mustGenerateExtricationTrajectories())
  {
    ROS_INFO_STREAM("BezierGrindingSurfacing::generateTrajectory: Generating extrication trajectories");
    // Generate all extrication trajectories
    EigenSTL::vector_Vector4d extrication_planes_equations;
    EigenSTL::vector_Vector3d extrication_planes_origins;
    // Compute extrication direction vector orientation
    EigenSTL::vector_Vector3d extrication_direction_vector;

    for (std::vector<EigenSTL::vector_Isometry3d>::iterator it(grinding_trajectories.begin());
        it != grinding_trajectories.end() - 1; ++it)
    {
      // Last point of grinding N line
      Eigen::Vector3d line_n_last_point((*it).back().translation());
      // First point of grinding N+1 line
      Eigen::Vector3d line_n1_first_point((*(it + 1)).front().translation());

      extrication_direction_vector.push_back(line_n1_first_point - line_n_last_point);

      Eigen::Vector4d plane_equation;
      Eigen::Vector3d plane_origin;

      EigenSTL::vector_Isometry3d traj;
      estimateExtricationSlicingPlane(line_n_last_point, line_n1_first_point, global_mesh_normal, plane_equation,
                                      plane_origin);

      extrication_planes_equations.push_back(plane_equation);
      extrication_planes_origins.push_back(plane_origin);
    }

    if (extrication_planes_equations.size() != extrication_planes_origins.size()
        || extrication_planes_equations.size() != extrication_direction_vector.size())
      return "Error: extrication plane equations/origins size are different!";

    std::vector<vtkSmartPointer<vtkStripper> > extrication_strippers;
    for (unsigned i = 0; i < extrication_planes_equations.size(); ++i)
    {
      vtkSmartPointer<vtkStripper> stripper = vtkSmartPointer<vtkStripper>::New();
      sliceMeshWithPlane(extrication_mesh, extrication_planes_equations[i], extrication_planes_origins[i], stripper);

      if (stripper->GetOutput()->GetNumberOfLines() > 1)
      {
        ROS_ERROR_STREAM(
            "BezierGrindingSurfacing::generateTrajectory: Extrication stripper has " << stripper->GetOutput()->GetNumberOfLines() << " lines");
        // FIXME Handle this case!
        //return "Extrication stripper has more than 1 line (mesh has a hole). Not implemented yet!";
      }

      extrication_strippers.push_back(stripper);
    }

    extrication_trajectories.clear();
    for (std::vector<vtkSmartPointer<vtkStripper> >::iterator it(extrication_strippers.begin());
        it != extrication_strippers.end(); ++it)
    {
      EigenSTL::vector_Isometry3d traj;
      if (!generateRobotPosesAlongStripper(*it, traj))
        return "Could not generate robot poses for extrication trajectory";

      // We generated the pose from N to N+1 but when extricating the grinder
      // the orientation should stay the same as the one when grinding.
      // So we revert the X axis to travel backwards:
      invertXAxisOfPoses(traj);

      extrication_trajectories.push_back(traj);
    }

    // Iterator allowing to move through the grinding trajectories
    std::vector<EigenSTL::vector_Isometry3d>::iterator grinding_iterator(grinding_trajectories.begin());

    for (unsigned i = 0; i < extrication_trajectories.size(); ++i)
    {
      if (harmonizeLineOrientation(extrication_trajectories[i], -direction_reference))
        ROS_INFO_STREAM("BezierGrindingSurfacing::generateTrajectory: Extrication line reversed");

      // Last point and normal of the grinding line N
      Eigen::Vector3d first_point((*grinding_iterator).back().translation());
      Eigen::Vector3d first_point_normal((*grinding_iterator).back().linear().col(2));
      first_point_normal *= -1;
      // Move the iterator to the next grinding line
      grinding_iterator++;

      // Last point and normal of the grinding line N + 1
      Eigen::Vector3d last_point((*grinding_iterator).front().translation());
      Eigen::Vector3d last_point_normal((*grinding_iterator).front().linear().col(2));
      last_point_normal *= -1;

      // Application of the extrication filter on the current extrication line
      if (!filterExtricationTrajectory(first_point, first_point_normal, last_point, last_point_normal,
                                       extrication_planes_equations[i], extrication_trajectories[i]))
      {
        return "Could not filter extrication trajectory";
      }
    }

    // Generate last extrication trajectory from last grinding point to first grinding point
    if (grinding_trajectories.empty())
      return "Grinding trajectories are empty, could not generate last extrication trajectory";

    if (grinding_trajectories.front().empty() || grinding_trajectories.back().empty())
      return "First or last grinding trajectory is empty, could not generate last extrication trajectory";

    Eigen::Vector3d grinding_first_point(grinding_trajectories.front().front().translation());
    Eigen::Vector3d grinding_last_point(grinding_trajectories.back().back().translation());

    Eigen::Vector4d last_extrication_plane_eq;
    Eigen::Vector3d last_extrication_plane_origin;
    estimateExtricationSlicingPlane(grinding_last_point, grinding_first_point, global_mesh_normal,
                                    last_extrication_plane_eq, last_extrication_plane_origin);

    vtkSmartPointer<vtkStripper> last_extrication_stripper = vtkSmartPointer<vtkStripper>::New();
    if (!sliceMeshWithPlane(extrication_mesh, last_extrication_plane_eq, last_extrication_plane_origin,
                            last_extrication_stripper))
      return "Could not slice mesh for last extrication trajectory";

    if (last_extrication_stripper->GetOutput()->GetNumberOfLines() > 1)
    {
      ROS_ERROR_STREAM(
          "BezierGrindingSurfacing::generateTrajectory: Extrication stripper has " << last_extrication_stripper->GetOutput()->GetNumberOfLines() << " lines");
      // FIXME Handle this case!
      //return "Extrication stripper has more than 1 line (mesh has a hole). Not implemented yet!";
    }

    EigenSTL::vector_Isometry3d last_extrication_traj;
    if (!generateRobotPosesAlongStripper(last_extrication_stripper, last_extrication_traj))
      return "Could not generate robot poses for last extrication trajectory";

    const Eigen::Vector3d last_extrication_orientation_reference(grinding_first_point - grinding_last_point);
    if (last_extrication_orientation_reference == Eigen::Vector3d::Zero())
      return "Could not harmonize last extrication trajectory, reference vector is Zero !";

    if (harmonizeLineOrientation(last_extrication_traj, last_extrication_orientation_reference))
      ROS_INFO_STREAM("BezierGrindingSurfacing::generateTrajectory: Last extrication line reversed");

    // Last point and normal of the last grinding line
    Eigen::Vector3d first_point(grinding_trajectories.back().back().translation());
    Eigen::Vector3d first_point_normal(grinding_trajectories.back().back().linear().col(2));
    first_point_normal *= -1;

    // First point and normal of the first grinding line
    Eigen::Vector3d last_point(grinding_trajectories.front().front().translation());
    Eigen::Vector3d last_point_normal(grinding_trajectories.front().front().linear().col(2));
    last_point_normal *= -1;

    // Application of the extrication filter on the last extrication line
    if (!filterExtricationTrajectory(first_point, first_point_normal, last_point, last_point_normal,
                                     last_extrication_plane_eq, last_extrication_traj))
      return "Could not filter last extrication trajectory";

    invertXAxisOfPoses(last_extrication_traj);
    extrication_trajectories.push_back(last_extrication_traj);

  }
  else
  {
    ROS_INFO_STREAM("BezierGrindingSurfacing::generateTrajectory: parameters haven't changed, skipping extrication trajectories generation");
  }

  if (params_observer_->mustGenerateGrindingTrajectories())
  {
    // Application of the grinding lean angle
    for (EigenSTL::vector_Isometry3d &poses : grinding_trajectories)
    {
      for (Eigen::Isometry3d &pose : poses)
        applyLeanAngle(pose, axis_of_rotation_, lean_angle_);
    }
  }

  // The first pose of the trajectory is a point located above the first grinding pose.
  // We use the last pose of the last extrication trajectory which is located above the
  // first pose of the first grinding line. This point is stored as the first pose of the trajectory.
  Eigen::Isometry3d start_pose(extrication_trajectories.back().back());
  // We keep the same orientation than the one of the first grinding pose of the first grinding line
  start_pose.linear() << grinding_trajectories.front().front().linear();

  // Generate Intermediate poses between the start pose of the trajectory and the first grinding line
  EigenSTL::vector_Isometry3d first_grinding_line_intermediate_poses;
  Eigen::Vector3d end_pose(grinding_trajectories.front().front().translation());
  generateIntermediatePoseOnLine(first_grinding_line_intermediate_poses, start_pose.translation(), end_pose, 2);
  for (Eigen::Isometry3d &pose : first_grinding_line_intermediate_poses)
  {
    // The intermediate poses have the same orientation than the first pose of the first grinding line
    pose.linear() << grinding_trajectories.front().front().linear();
  }

  if (params_observer_->mustGenerateGrindingTrajectories() || params_observer_->mustGenerateExtricationTrajectories())
  {
    // Generate intermediate poses between last extrication pose and first grinding pose
    // of each trajectories
    end_intermediate_poses_trajectories.clear();
    std::vector<EigenSTL::vector_Isometry3d>::iterator grinding_it(grinding_trajectories.begin() + 1);
    for (std::vector<EigenSTL::vector_Isometry3d>::iterator extrication_traj(extrication_trajectories.begin());
        extrication_traj != extrication_trajectories.end() - 1; extrication_traj++)
    {
      EigenSTL::vector_Isometry3d intermediate_poses;
      Eigen::Vector3d start_pose((*extrication_traj).back().translation());
      Eigen::Vector3d end_pose((*grinding_it).front().translation());
      generateIntermediatePoseOnLine(intermediate_poses, start_pose, end_pose, 2);
      for (Eigen::Isometry3d &pose : intermediate_poses)
      {
        //The intermediate pose has the same orientation than the first grinding line
        pose.linear() << (*grinding_it).front().linear();
      }
      end_intermediate_poses_trajectories.push_back(intermediate_poses);
      grinding_it++;
    }

    // Generate intermediate poses between last grinding pose and first extrication pose
    // of each trajectories
    start_intermediate_poses_trajectories.clear();
    std::vector<EigenSTL::vector_Isometry3d>::iterator extrication_it(extrication_trajectories.begin());
    for (std::vector<EigenSTL::vector_Isometry3d>::iterator grinding_traj(grinding_trajectories.begin());
        grinding_traj != grinding_trajectories.end(); grinding_traj++)
    {
      EigenSTL::vector_Isometry3d intermediate_poses;
      Eigen::Vector3d start_pose((*grinding_traj).back().translation());
      Eigen::Vector3d end_pose((*extrication_it).front().translation());
      generateIntermediatePoseOnLine(intermediate_poses, start_pose, end_pose, 2);
      for (Eigen::Isometry3d &pose : intermediate_poses)
      {
        // The intermediate pose has the same orientation than the first pose of the extrication line
        pose.linear() << (*extrication_it).front().linear();
      }
      start_intermediate_poses_trajectories.push_back(intermediate_poses);
      extrication_it++;
    }

    // Add intermediates poses to extrication trajectories
    std::vector<EigenSTL::vector_Isometry3d>::iterator end_intermediate_poses_it(
        end_intermediate_poses_trajectories.begin());
    for (std::vector<EigenSTL::vector_Isometry3d>::iterator extrication_traj(extrication_trajectories.begin());
        extrication_traj != extrication_trajectories.end() - 1; extrication_traj++)
    {
      (*extrication_traj).insert((*extrication_traj).end(), (*end_intermediate_poses_it).begin(),
                                 (*end_intermediate_poses_it).end());
      end_intermediate_poses_it++;
    }

    // Add intermediates poses to extrication trajectories
    std::vector<EigenSTL::vector_Isometry3d>::iterator start_intermediate_poses_it(
        start_intermediate_poses_trajectories.begin());
    for (std::vector<EigenSTL::vector_Isometry3d>::iterator extrication_traj(extrication_trajectories.begin());
        extrication_traj != extrication_trajectories.end(); extrication_traj++)
    {
      (*extrication_traj).insert((*extrication_traj).begin(), (*start_intermediate_poses_it).begin(),
                                 (*start_intermediate_poses_it).end());
      start_intermediate_poses_it++;
    }
  }

  unsigned index(0);
  if (display_markers)
  {
    for (EigenSTL::vector_Isometry3d traj : grinding_trajectories)
    {
      if (index > 11)
        index = 0;
      displayTrajectory(traj, visualToolsColorFromIndex(index++), true);
    }

    index = 0;
    for (EigenSTL::vector_Isometry3d traj : extrication_trajectories)
    {
      if (index > 11)
        index = 0;
      displayTrajectory(traj, visualToolsColorFromIndex(index++), true);
    }

    for (EigenSTL::vector_Isometry3d traj : end_intermediate_poses_trajectories)
    {
      if (index > 11)
        index = 0;
      displayTrajectory(traj, visualToolsColorFromIndex(index++), true);
    }

    for (EigenSTL::vector_Isometry3d traj : start_intermediate_poses_trajectories)
    {
      if (index > 11)
        index = 0;
      displayTrajectory(traj, visualToolsColorFromIndex(index++), true);
    }

    // Display the start pose of the trajectory
    Eigen::Isometry3d display_start_pose(start_pose);
    visual_tools_->publishXArrow(display_start_pose, rviz_visual_tools::GREEN, rviz_visual_tools::XXXXSMALL, 0.008);
    visual_tools_->publishZArrow(display_start_pose, rviz_visual_tools::GREEN, rviz_visual_tools::XXXXSMALL, 0.008);
    start_pose.translation() -= 0.01 * display_start_pose.affine().col(2).head<3>();
    visual_tools_->publishText(display_start_pose, "Start pose", rviz_visual_tools::GREEN, rviz_visual_tools::SMALL, false);
    visual_tools_->trigger();
    // Display intermediates poses between trajectory start point and the first grinding point of the first grinding line
    displayTrajectory(first_grinding_line_intermediate_poses, visualToolsColorFromIndex(index++), true);
  }

  // Add one grinding traj, one extrication traj...
  trajectory.clear();
  is_grinding_pose.clear();
  std::vector<EigenSTL::vector_Isometry3d>::iterator extrication_iterator(extrication_trajectories.begin());

  trajectory.push_back(start_pose);
  is_grinding_pose.push_back(false);
  for (Eigen::Isometry3d &pose : first_grinding_line_intermediate_poses)
  {
    // Add intermediate poses between the start pose of the trajectory and the first grinding pose
    trajectory.push_back(pose);
    is_grinding_pose.push_back(false);
  }

  for (EigenSTL::vector_Isometry3d grinding_traj : grinding_trajectories)
  {
    for (Eigen::Isometry3d grinding_pose : grinding_traj)
    {
      trajectory.push_back(grinding_pose);
      is_grinding_pose.push_back(true);
    }
    if (extrication_iterator != extrication_trajectories.end())
    {
      for (Eigen::Isometry3d extrication_pose : *extrication_iterator)
      {
        trajectory.push_back(extrication_pose);
        is_grinding_pose.push_back(false);
      }
      extrication_iterator++;
    }
  }

  return "";
}

void BezierGrindingSurfacing::setSlicingOrientation(const Eigen::Vector3d &cutting_plane_normal)
{
  slicing_orientation_ = cutting_plane_normal;

  if (slicing_orientation_ != Eigen::Vector3d::Zero())
    slicing_orientation_.normalize();
}

void BezierGrindingSurfacing::generateIntermediatePoseOnLine(EigenSTL::vector_Isometry3d &poses,
                                      const Eigen::Vector3d &start_point,
                                      const Eigen::Vector3d &end_point,
                                      const unsigned number_of_poses)
{
  // This function generates a specified number of point between a start and an end point along a line.
  // Each point is located at an equally far away position of his predecessor.
  // Each point is generated using the formula : Point = StartPoint + d*LineVector
  // with d the distance between the start point and the generated point and LineVector the vector between the
  // start point and the end point of the line.

  // Direction vector of the line
  Eigen::Vector3d direction(end_point - start_point);
  // Distance between each point
  double distance = direction.norm() / (number_of_poses + 1);
  direction.normalize();
  poses.clear();
  for (unsigned index(1); index <= number_of_poses; index++)
  {
    double d = distance * (index);
    Eigen::Vector3d point(start_point + d * direction);
    Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
    pose.translation() << point;
    poses.push_back(pose);
  }
}

std::string BezierGrindingSurfacing::validateParameters()
{
  if (grinding_disk_machining_width_ <= 0.0)
    return "Grinding disk diameter must be > 0";

  if (covering_percentage_ > 99)
    return "Covering percentage must be lower than 100%";
  return "";
}
