// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <tf/transform_listener.h>
#pragma GCC diagnostic pop
#include <tf_conversions/tf_eigen.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "bezier_library/bezier_grinding_surfacing.hpp"

/** Name of the move_group used to move the robot */
const std::string move_group_name("manipulator");

/** Name of the TCP that should be used to compute the trajectories */
const std::string tcp_name("/tool0");

/** @brief The main function
 * @param[in] argc
 * @param[in] argv
 * @return Exit status */
int main(int argc,
         char **argv)
{
  std::string package = "bezier_application";
  ros::init(argc, argv, package);
  ros::NodeHandle nh;

  // Get package path
  std::string meshes_path = ros::package::getPath(package) + "/meshes/";
  std::string mesh_ressources = "package://" + package + "/meshes/";

  // Parameters defined in the launch file
  std::string mesh_cad_filename;
  std::string mesh_defect_filename;
  bool surfacing_mode(false);
  nh.getParam("surfacing_mode_param", surfacing_mode);
  nh.getParam("mesh_cad_param", mesh_cad_filename);
  nh.getParam("mesh_defect_param", mesh_defect_filename);

  if (mesh_cad_filename.size() > 4) // size > ".ply"
    ROS_INFO_STREAM("CAD mesh file:" << mesh_cad_filename);
  else
  {
    ROS_ERROR_STREAM("Command line error, please specify a CAD mesh file!");
    return -1;
  }
  std::string mesh_cad_path = meshes_path + mesh_cad_filename;

  std::string mesh_defect_path("");
  if (!surfacing_mode)
  {
    ROS_ERROR_STREAM("Not implemented yet! (only surface mode working now)");
    return -1;
  }

  // Generate trajectory
  double grinder_width = 0.031; // meters
  unsigned covering_percentage = 0; // %
  double extrication_radius = 0.04; // meters
  double angle_value = -0.22; // radians
  Bezier::AXIS_OF_ROTATION axis = Bezier::AXIS_OF_ROTATION::Y;
  std::shared_ptr<BezierGrindingSurfacing> bezier_planner;
  try
  {
    bezier_planner.reset(
        new BezierGrindingSurfacing(mesh_cad_path, grinder_width, covering_percentage, extrication_radius, angle_value,
                                    axis));
  }
  catch (std::exception &bezier_exception)
  {
    ROS_ERROR_STREAM(bezier_exception.what());
    return 0;
  }

  // Set-up publishers for the visualization
  std::shared_ptr<ros::Publisher> cad_mesh_pub;
  cad_mesh_pub.reset(new ros::Publisher(nh.advertise<visualization_msgs::Marker>("bezier_cad_mesh", 0)));
  std::shared_ptr<ros::Publisher> dilated_mesh_pub;
  dilated_mesh_pub.reset(new ros::Publisher(nh.advertise<visualization_msgs::Marker>("bezier_dilated_mesh", 0)));

  while (nh.ok() && cad_mesh_pub->getNumSubscribers() < 1)
  {
    ROS_WARN_STREAM("Waiting for a subscriber on topic \"" << cad_mesh_pub->getTopic() << "\"");
    sleep(1);
  }
  while (nh.ok() && dilated_mesh_pub->getNumSubscribers() < 1)
  {
    ROS_WARN_STREAM("Waiting for a subscriber on topic \"" << dilated_mesh_pub->getTopic() << "\"");
    sleep(1);
  }

  bezier_planner->setMeshesPublishers(cad_mesh_pub, dilated_mesh_pub);
  bezier_planner->waitForRvizVisualToolsSubscriber();

  EigenSTL::vector_Isometry3d way_points_vector;
  std::string error_message;
  std::vector<bool> is_grinding_pose;
  error_message = bezier_planner->generateTrajectory(way_points_vector, is_grinding_pose);

  if (!error_message.empty() || way_points_vector.empty())
  {
    if (way_points_vector.empty())
      ROS_WARN_STREAM("Empty trajectory!");

    ROS_ERROR_STREAM("Could not generate trajectory:" << std::endl << error_message);
    while (nh.ok())
    {
    }
    return -1;
  }

  ROS_INFO_STREAM("Trajectory size: " << way_points_vector.size());

  // Copy the vector of Eigen poses into a vector of ROS poses
  std::vector<geometry_msgs::Pose> way_points_msg;
  for (Eigen::Isometry3d eigen_pose : way_points_vector)
  {
    //ROS_INFO_STREAM("Pose : " << std::endl << eigen_pose.matrix());
    geometry_msgs::Pose tmp;
    tf::poseEigenToMsg(eigen_pose, tmp);
    way_points_msg.push_back(tmp);
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Initialize move group
  moveit::planning_interface::MoveGroupInterface group(move_group_name);
  group.setPoseReferenceFrame("/base_link");
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPlanningTime(2);

  // Execute robot trajectory
  moveit_msgs::ExecuteKnownTrajectory srv;
  srv.request.wait_for_execution = true;
  ros::ServiceClient executeKnownTrajectoryServiceClient = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(
      "/execute_kinematic_path");

  tf::TransformListener listener(nh);
  while (nh.ok())
  {
    // Bezier trajectory
    if (group.computeCartesianPath(way_points_msg, 0.05, 0.0, srv.request.trajectory) < 0.95)
    {
      ROS_ERROR_STREAM(
          "Bezier application: A solution could not be found to move the robot along the trajectory, aborting.");
      break;
    }
    executeKnownTrajectoryServiceClient.call(srv);

    // Return to first point
    std::vector<geometry_msgs::Pose> way_points_msg_tmp(1);
    way_points_msg_tmp[0] = way_points_msg[0];
    listener.waitForTransform("/base", tcp_name, ros::Time::now(), ros::Duration(3.0));
    sleep(1);
    if (group.computeCartesianPath(way_points_msg_tmp, 0.05, 0.0, srv.request.trajectory) < 0.95)
    {
      ROS_ERROR_STREAM(
          "Bezier application: A solution could not be found to move the robot along the trajectory, aborting.");
      continue;
    }
    executeKnownTrajectoryServiceClient.call(srv);
    sleep(1);
  }

  spinner.stop();
  return 0;
}
