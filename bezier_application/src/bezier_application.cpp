#include "bezier_library/bezier_library.hpp"

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/StdVector>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

/** Pointer to the move group */
boost::shared_ptr<move_group_interface::MoveGroup> group;

/** Name of the move_group used to move the robot during calibration */
const std::string move_group_name("grinding_disk");
/** Name of the TCP that should be used to compute the trajectories */
const std::string tcp_name("/grinding_disk_tcp");

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
  std::string mesh_ressources = "package://" + package + "/meshes/";

  // Get PLY file name from command line
  std::string input_mesh_filename;
  std::string defect_mesh_filename;
  std::string surface_mode;
  node.getParam("meshname_param", input_mesh_filename); //meshname_param is a parameter defined in launch file
  node.getParam("surface_param", surface_mode); // grinding_param is a parameter defined in the launch file
  if (input_mesh_filename.size() > 4) // size>".ply"
    ROS_INFO_STREAM("Mesh file imported :" << input_mesh_filename);
  else
  {
    ROS_ERROR_STREAM("Command line error, please specify a mesh file (eg meshname:=plane/plane.ply)");
    return -1;
  }
  std::string mesh_original = meshes_path + input_mesh_filename;
  std::string mesh_defect;

  defect_mesh_filename = input_mesh_filename.substr(0, input_mesh_filename.size() - 4) + "_defect.ply";
  mesh_defect = meshes_path + defect_mesh_filename;

  // Create publishers for point clouds and markers
  ros::Publisher trajectory_publisher,
                 input_mesh_publisher,
                 defect_mesh_publisher,
                 dilated_mesh_publisher,
                 normal_publisher;
  trajectory_publisher = node.advertise<visualization_msgs::Marker>("my_trajectory", 1);
  input_mesh_publisher = node.advertise<visualization_msgs::Marker>("my_input_mesh", 1);
  defect_mesh_publisher = node.advertise<visualization_msgs::Marker>("my_defect_mesh", 1);
  dilated_mesh_publisher = node.advertise<visualization_msgs::Marker>("my_dilated_mesh", 1);
  normal_publisher = node.advertise<visualization_msgs::MarkerArray>("my_normals", 1);

  // Generate trajectory
  std::string lean_angle_axis = "y";
  double angle_value = -0.4;
  double covering_percentage = 0.4; //value between 0.0 & 1.0
  double grind_diameter = 0.03;
  double maximum_depth_of_path = 0.015;
  int extrication_frequency = 5; // Generate a new extrication mesh each 4 passes generated
  int extrication_coefficient = 4;
  Bezier bezier_planner(mesh_original,
                        mesh_defect,
                        lean_angle_axis,
                        angle_value,
                        maximum_depth_of_path,
                        grind_diameter,
                        covering_percentage,
                        extrication_coefficient,
                        extrication_frequency,
                        false);
  std::vector<bool> points_color_viz;
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector;
  std::vector<int> index_vector;
  if (boost::iequals(surface_mode, "on"))
  {
    //Grinding on surface mode has been selected
    bezier_planner.setSurfacingOn();
  }
  else
  {
    //Apply the full grinding mode
    bezier_planner.setSurfacingOff();
  }
  // Display in RVIZ
  bezier_planner.displayMesh(input_mesh_publisher, mesh_ressources + input_mesh_filename);
  bezier_planner.displayMesh(defect_mesh_publisher, mesh_ressources + defect_mesh_filename, 0.1, 0.1, 0.1, 0.6);
  bezier_planner.generateTrajectory(way_points_vector, points_color_viz, index_vector);

  // Save dilated meshes
  // Create directory
  boost::filesystem::path dir(meshes_path + "dilated_meshes");
  if (!boost::filesystem::create_directory(dir))
    ROS_WARN_STREAM(meshes_path + "dilated_meshes" << " directory could not be created.");
  // Save
  bezier_planner.saveDilatedMeshes(meshes_path + "dilated_meshes");

  // Execute robot trajectory
  // Initialize move group
  group.reset(new move_group_interface::MoveGroup("grinding_disk"));
  group->setPoseReferenceFrame("/base_link");
  group->setPlannerId("RRTConnectkConfigDefault");
  group->setPlanningTime(2);

  while (ros::ok())
  {
    for (unsigned int i = 0; i < index_vector.size() - 1; i++)
    { //For each pass
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector_pass; // Get pose in this pass
      way_points_vector_pass.insert(way_points_vector_pass.begin(), way_points_vector.begin() + index_vector[i] + 1,
                                    way_points_vector.begin() + index_vector[i + 1]);

      std::vector<bool> points_color_viz_pass; // Get bool data in this pass (real/extrication path in this pass)
      points_color_viz_pass.insert(points_color_viz_pass.begin(), points_color_viz.begin() + index_vector[i] + 1,
                                   points_color_viz.begin() + index_vector[i + 1]);

      std::string number(boost::lexical_cast<std::string>(i));
      bezier_planner.displayMesh(dilated_mesh_publisher, mesh_ressources + "dilated_meshes/mesh_" + number + ".ply");
      bezier_planner.displayTrajectory(way_points_vector_pass, points_color_viz_pass, trajectory_publisher); // Display trajectory in this pass
      //bezier_planner.displayNormal(way_points_vector_pass, points_color_viz_pass, normal_publisher); // Display normals in this pass

      // Copy the vector of Eigen poses into a vector of ROS poses
      std::vector<geometry_msgs::Pose> way_points_msg;
      way_points_msg.resize(way_points_vector_pass.size());

      for (size_t j = 0; j < way_points_msg.size(); j++)
      {
        tf::poseEigenToMsg(way_points_vector_pass[j], way_points_msg[j]);
        tf::Transform world_to_link6_tf, world_to_tcp_tf;

        geometry_msgs::Pose world_to_link6 = way_points_msg[j];
        tf::poseMsgToTF(world_to_link6, world_to_link6_tf);
        world_to_tcp_tf = world_to_link6_tf;

        geometry_msgs::Pose world_to_tcp;
        tf::poseTFToMsg(world_to_tcp_tf, world_to_tcp);
        way_points_msg[j] = world_to_tcp;
      }

      // Execute this trajectory
      moveit_msgs::ExecuteKnownTrajectory srv;
      srv.request.wait_for_execution = true;
      ros::ServiceClient executeKnownTrajectoryServiceClient = node.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(
          "/execute_kinematic_path");
      if (group->computeCartesianPath(way_points_msg, 0.05, 0.0, srv.request.trajectory) < 0.95)
      {
        ROS_ERROR_STREAM("A solution could not be found to move the robot along the trajectory, aborting.");
        return -1;
      }
      executeKnownTrajectoryServiceClient.call(srv);

      // Return to first point
      way_points_msg.resize(1);
      listener.waitForTransform("/base", tcp_name, ros::Time::now(), ros::Duration(3.0));
      sleep(1);
      group->computeCartesianPath(way_points_msg, 0.05, 0.0, srv.request.trajectory);
      executeKnownTrajectoryServiceClient.call(srv);
      sleep(1);
    }
  }

  while (node.ok())
  {
  }
  spinner.stop();
  return 0;
}
