#include "bezier_library/bezier_library.hpp"

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_ros/point_cloud.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/StdVector>

/**
 * @file bezier_application.hpp
 * @brief This is an application using bezier_library.
 * @author Francois Lasson - Institut Maupertuis (France)
 * @date Project started in February 2015
 */
/**@mainpage This application is a test of bezier_library.
 * It has been developed for a Fanuc M10iA Robot under Ubuntu 14.04 with PCL 1.8.0
 * VTK 6.3 (with patch) and ROS Indigo.
 */

/** @brief The main function
 * @param[in] argc
 * @param[in] argv
 * @return Exit status */
int main(int argc, char **argv)
{
  std::string package = "bezier_application";
  ros::init(argc, argv, package);
  ros::NodeHandle node;
  tf::TransformListener listener(node);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Get package path
  std::string package_path = ros::package::getPath(package);
  std::string meshes_path = package_path + "/meshes/";
  std::string mesh_ressource = "package://" + package + "/meshes/";

  // Get PLY file name from command line
  std::string input_mesh_filename;
  std::string default_mesh_filename;
  node.getParam("filename_param", input_mesh_filename); //filename_param is a parameter defined in launch file
  if (input_mesh_filename.size() > 4) // size>".ply"
    ROS_INFO_STREAM("Mesh file imported :" << input_mesh_filename);
  else
  {
    ROS_WARN_STREAM("Command line error in file set up." << std::endl <<
                    "Usage :" << std::endl << "roslaunch my_path_generator path_generator.launch filename:=filename.ply");
    return -1;
  }
  std::string mesh_original = meshes_path + input_mesh_filename;
  std::string mesh_default;

  default_mesh_filename = input_mesh_filename.substr(0, input_mesh_filename.size() - 4) + "_default.ply";
  mesh_default = meshes_path + default_mesh_filename;

  // Create publishers for point clouds and markers
  ros::Publisher trajectory_publisher, input_mesh_publisher, default_mesh_publisher, dilated_mesh_publisher,
                 normal_publisher, fix_table_mesh_publisher, fsw_table_mesh_publisher;
  trajectory_publisher    = node.advertise<visualization_msgs::Marker>("my_trajectory", 1);
  input_mesh_publisher    = node.advertise<visualization_msgs::Marker>("my_input_mesh", 1);
  default_mesh_publisher  = node.advertise<visualization_msgs::Marker>("my_default_mesh", 1);
  dilated_mesh_publisher  = node.advertise<visualization_msgs::Marker>("my_dilated_mesh", 1);
  normal_publisher        = node.advertise<visualization_msgs::MarkerArray>("my_normals", 1);
  fix_table_mesh_publisher = node.advertise<visualization_msgs::Marker>("my_fix_table", 1);
  fsw_table_mesh_publisher = node.advertise<visualization_msgs::Marker>("my_fsw_table", 1);

  // Generate trajectory
  double size_of_fsw_tool = 0.45767; //height oh fsw tool
  double size_of_grind_tool = 0.04; //height of end-mill

  double covering_percentage = 0.25; //value between 0.0 & 1.0
  double grind_diameter = 0.014;
  double maximum_depth_of_path = 0.015;
  int extrication_frequency = 5; // Generate a new extrication mesh each 4 passes generated
  int extrication_coefficient = 5;
  Bezier bezier_planner(mesh_original, mesh_default, maximum_depth_of_path, grind_diameter, covering_percentage,
                        extrication_coefficient, extrication_frequency, true);
  std::vector<bool> points_color_viz;
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector;
  std::vector<int> index_vector;

  // Display in RVIZ
  bezier_planner.displayMesh(fix_table_mesh_publisher, mesh_ressource + "environment/TableFix.ply", .1, .4, .1);
  bezier_planner.displayMesh(fsw_table_mesh_publisher, mesh_ressource + "environment/TableFSW.ply");
  bezier_planner.displayMesh(input_mesh_publisher, mesh_ressource + input_mesh_filename);
  bezier_planner.displayMesh(default_mesh_publisher, mesh_ressource + default_mesh_filename);
  bezier_planner.generateTrajectory(way_points_vector, points_color_viz, index_vector);

  // Save dilated meshes
  bezier_planner.saveDilatedMeshes(meshes_path + "dilatedMeshes");

  // Execute robot trajectory
  move_group_interface::MoveGroup group("manipulator");
  group.setPoseReferenceFrame("/base"); // Otherwise "base_link" is the reference!

  while (ros::ok())
  {
    for (int i = 0; i < index_vector.size() - 1; i++)
    { //For each pass
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector_pass; // Get pose in this pass
      way_points_vector_pass.insert(way_points_vector_pass.begin(), way_points_vector.begin() + index_vector[i] + 1,
                                    way_points_vector.begin() + index_vector[i + 1]);
      std::vector<bool> points_color_viz_pass; // Get bool data in this pass (real/extrication path in this pass)
      points_color_viz_pass.insert(points_color_viz_pass.begin(), points_color_viz.begin() + index_vector[i] + 1,
                                    points_color_viz.begin() + index_vector[i + 1]);

      std::string number(boost::lexical_cast<std::string>(i));
      bezier_planner.displayMesh(dilated_mesh_publisher, mesh_ressource + "dilatedMeshes/mesh_" + number + ".ply");
      bezier_planner.displayTrajectory(way_points_vector_pass, points_color_viz_pass, trajectory_publisher); // Display trajectory in this pass
      bezier_planner.displayNormal(way_points_vector_pass, points_color_viz_pass, normal_publisher); // Display normals in this pass

      // Add offset BASE/BASE_LINK
      for (int j = 0; j < way_points_vector_pass.size(); j++)
      {
        way_points_vector_pass[j].translation().z() -= 0.950;
        //reverse tool orientation for cables (Specific to institut maupertuis' robot)
        way_points_vector_pass[j].linear().col(0) = - way_points_vector_pass[j].linear().col(0);
        way_points_vector_pass[j].linear().col(1) = - way_points_vector_pass[j].linear().col(1);
      }
      // Copy the vector of Eigen poses into a vector of ROS poses
      std::vector<geometry_msgs::Pose> way_points_msg;
      way_points_msg.resize(way_points_vector_pass.size());
      tf::Transform link6_to_tcp_tf;
      link6_to_tcp_tf = tf::Transform(tf::Matrix3x3::getIdentity(),
                                      tf::Vector3(0.0, 0.0, -size_of_fsw_tool - size_of_grind_tool));
      for (size_t j = 0; j < way_points_msg.size(); j++)
      {
        tf::poseEigenToMsg(way_points_vector_pass[j], way_points_msg[j]);
        tf::Transform world_to_link6_tf, world_to_tcp_tf;
        geometry_msgs::Pose world_to_link6 = way_points_msg[j];
        geometry_msgs::Pose world_to_tcp;
        tf::poseMsgToTF(world_to_link6, world_to_link6_tf);
        world_to_tcp_tf = world_to_link6_tf * link6_to_tcp_tf;
        tf::poseTFToMsg(world_to_tcp_tf, world_to_tcp);
        way_points_msg[j] = world_to_tcp;
      }

      // Execute this trajectory
      moveit_msgs::ExecuteKnownTrajectory srv;
      srv.request.wait_for_execution = true;
      ros::ServiceClient executeKnownTrajectoryServiceClient = node.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(
          "/execute_kinematic_path");
      group.computeCartesianPath(way_points_msg, 0.05, 0.0, srv.request.trajectory);
      executeKnownTrajectoryServiceClient.call(srv);

      // Return to first point
      way_points_msg.resize(1);
      listener.waitForTransform("/base", "/tool0", ros::Time::now(), ros::Duration(3.0));
      sleep(1);
      group.computeCartesianPath(way_points_msg, 0.05, 0.0, srv.request.trajectory);
      executeKnownTrajectoryServiceClient.call(srv);
      sleep(1);
    }
  }
  return 0;
}
