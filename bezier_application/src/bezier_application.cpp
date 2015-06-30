#include "bezier_library/bezier_library.hpp"
//C++ common
#include <limits>

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <eigen_conversions/eigen_msg.h>
#include <math.h>
#include <Eigen/StdVector>

/**
 * @file bezier_application.hpp
 * @brief This is a application using bezier_library.
 * @author Francois Lasson _ Institut Maupertuis (France)
 * @date Project started in February 2015
 */
/**@mainpage This application is a test of bezier_library.
 * It has been developed for a Fanuc M10ia Robot under Ubuntu 14.04 with PCL 1.8.0
 * VTK 6.3 (with patch : more details in Readme.txt) and ROS Indigo.
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

    ////////// GET PACKAGE PATH //////////
    std::string DIR = ros::package::getPath(package);
    std::string MESH_DIR = DIR + "/meshes/";
    std::string mesh_ressource = "package://"+package+"/meshes/";

    ////////// GET PLY FILENAME FROM COMMAND LINE //////////
    std::string input_mesh_filename;
    std::string default_mesh_filename;
    node.getParam("filename_param", input_mesh_filename); //filename_param is a parameter defined in launch file
    if (input_mesh_filename.size() > 4) // size>".ply"
    {
        ROS_INFO("\n\nMesh file imported : %s \n", input_mesh_filename.c_str());
    }
    else
    {
        ROS_WARN(
                "\n\nCommand line error in file set up. Use : roslaunch my_path_generator path_generator.launch filename:=filename.ply \n");
        return -1;
    }
    std::string MESH_ORIGIN_DIR = MESH_DIR + input_mesh_filename;
    std::string MESH_DEFAULT_DIR;

    default_mesh_filename = input_mesh_filename.substr(0, input_mesh_filename.size() - 4) + "_default.ply";
    MESH_DEFAULT_DIR = MESH_DIR + default_mesh_filename;

    ////////// CREATE PUBLISHERS FOR POINT CLOUDS AND MARKER //////////
    ros::Publisher trajectory_publisher, input_mesh_publisher, default_mesh_publisher, dilated_mesh_publisher, normal_publisher;
    trajectory_publisher = node.advertise<visualization_msgs::Marker>("my_trajectory", 1);
    input_mesh_publisher = node.advertise<visualization_msgs::Marker>("my_input_mesh", 1);
    default_mesh_publisher = node.advertise<visualization_msgs::Marker>("my_default_mesh", 1);
    dilated_mesh_publisher = node.advertise<visualization_msgs::Marker>("my_dilated_mesh", 1);
    normal_publisher = node.advertise<visualization_msgs::MarkerArray>("my_normals", 1);

    ////////// GENERATE TRAJECTORY //////////
    double covering_percentage = 0.5; //value between 0 & 1
    double grind_diameter = 0.1;
    double grind_depth = 0.05;
    int extrication_frequency = 5; //genere a new extrication mesh each 4 pass generated
    int extrication_coefficiant = 1;
    Bezier grind_object(MESH_ORIGIN_DIR,MESH_DEFAULT_DIR,grind_depth,grind_diameter,covering_percentage,extrication_coefficiant,extrication_frequency);
    std::vector<bool> points_color_viz;
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector;
    std::vector<int> index_vector;

    ////////// DISPLAY IN RVIZ //////////

    grind_object.displayMesh(input_mesh_publisher, mesh_ressource+input_mesh_filename);
    grind_object.displayMesh(default_mesh_publisher, mesh_ressource+default_mesh_filename);
    grind_object.generateTrajectory(way_points_vector,points_color_viz, index_vector);

    ////////// SAVE DILATED MESH //////////
    grind_object.saveDilatedMeshes(MESH_DIR+"dilatedMeshes");

    ////////////////// ROBOT TRAJECTORY //////////////////
    move_group_interface::MoveGroup group("manipulator");
    group.setPoseReferenceFrame("/base"); // Otherwise "base_link" is the reference!

    while (ros::ok())
    {
        for(int i=0;i<index_vector.size()-1;i++){ //For each passe
            std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector_passe; //get pose in this passe
            way_points_vector_passe.insert(way_points_vector_passe.begin(), way_points_vector.begin() + index_vector[i]+1, way_points_vector.begin() + index_vector[i+1]);
            std::vector<bool> points_color_viz_passe; //get bool data in this passe (real/extrication path in this passe)
            points_color_viz_passe.insert(points_color_viz_passe.begin(), points_color_viz.begin() + index_vector[i]+1, points_color_viz.begin() + index_vector[i+1]);

            std::string number (boost::lexical_cast<std::string>(i));
            grind_object.displayMesh(dilated_mesh_publisher,mesh_ressource+"dilatedMeshes/mesh_"+number+".ply");
            grind_object.displayTrajectory(way_points_vector_passe, points_color_viz_passe, trajectory_publisher); //display trajectory in this passe
            grind_object.displayNormal(way_points_vector_passe, points_color_viz_passe, normal_publisher); //display normal in this passe

            ////////// ADD OFFSET BASE/BASE_LINK //////////
            for (int j = 0; j < way_points_vector_passe.size(); j++)
            {
                way_points_vector_passe[j].translation().z() -= 0.45; //In order to face Z robot offset
            }

            // Copy the vector of Eigen poses into a vector of ROS poses
            std::vector<geometry_msgs::Pose> way_points_msg;
            way_points_msg.resize(way_points_vector_passe.size());
            for (size_t j = 0; j < way_points_msg.size(); j++)
                tf::poseEigenToMsg(way_points_vector_passe[j], way_points_msg[j]);

            // execute this trajectory
            ROS_WARN("Start ExecuteKnownTrajectory trajectory");
            moveit_msgs::ExecuteKnownTrajectory srv;
            srv.request.wait_for_execution = true;
            ros::ServiceClient executeKnownTrajectoryServiceClient = node.serviceClient<
                    moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");
            group.computeCartesianPath(way_points_msg, 0.05, 0.0, srv.request.trajectory);
            executeKnownTrajectoryServiceClient.call(srv);

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
