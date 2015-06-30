#ifndef BEZIER_LIBRARY_HPP
#define BEZIER_LIBRARY_HPP

#include <iostream>
#include <string.h>
#include <limits>

// PCL headers
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h> //removeNan
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h> //Kdtree
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/surface/vtk_smoothing/vtk_utils.h>

// Eigen headers
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/StdVector>
#include <Eigen/Dense>

// VTK headers
#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkProperty.h>
#include <vtkMath.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkTriangleFilter.h>
#include <vtkStripper.h>
#include <vtkCellData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkGlyph3D.h>
#include <vtkArrowSource.h>
#include <vtkImplicitModeller.h>
#include <vtkMarchingCubes.h>
#include <vtkKdTreePointLocator.h>
#include <vtkIdList.h>
#include <vtkPLYWriter.h>
#include <vtkPolyDataNormals.h>
#include <vtkSortDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkCellArray.h>

// ROS headers
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/lexical_cast.hpp>

/**
 * @file bezier_library.hpp
 * @brief Library used to generate 3D paths (robot poses) from CAO ply files.
 * @author Francois Lasson _ Institut Maupertuis (France)
 * @date Project started in February 2015
 */


/** @mainpage bezier_library is a library developed by the "Institut Maupertuis",
 * a french research institute that works on robotic processes.
 *
 * This library has been created in order to generate 3D paths for grind robots.
 * Using ROS, VTK and PCL, this program generates paths from PLY files (meshes) and sends
 * trajectories (poses) to the robot by ROS.
 *
 * It has been developed under Ubuntu 14.04 with PCL 1.8.0
 * VTK 6.3 (with patch more details in Readme.txt) and ROS Indigo.
 *
 * Process is quite easy to understand.
 * Initially, we have a mesh in which we would like to generate some trajectory.
 * The process of path generation is as follow :
 * Firstly, we get mesh normal using Ransac (Random Sample Consensus)
 * This normal allows us to determine a cut direction.
 * We dilate the input mesh in order to expend path in all directions and to be able to grind detected defaults
 * For each dilated mesh, we use the VTK function VtkCutter to cut several slices in mesh.
 * Then, we get all point normals on these slices (polylines) in order generate vector spaces and to set flange orientation.
 * Last, we execute trajectory with ROS and display some markers in RVIZ.
 */

///@brief PointT is a pcl::PointXYZRGBA
typedef pcl::PointXYZRGBA PointT;
///@brief PointCloudT is a PointT cloud
typedef pcl::PointCloud<PointT> PointCloudT;

/** @brief Bezier */
class Bezier
{
  public:

    ///@brief Default Constructor
    Bezier();

    /**@brief Initialized constructor
     * @param[in] filename_inputMesh filename of input poly data (input mesh)
     * @param[in] filename_defaultMesh filename of default poly data (default mesh)
     * @param[in] grind_depth grinding depth (in meters)
     * @param[in] effector_diameter diameter of effector (in meters)
     * @param[in] covering Percentage of covering (decimal value)
     * @param[in] extrication_coefficiant extrication depth equal of how many grind_depth (coefficiant)
     * @param[in] extrication_frequency new extrication mesh generated each 1/extrication_frequency times.
     */
    Bezier(std::string filename_inputMesh, std::string filename_defaultMesh, double grind_depth, double effector_diameter, double covering, int extrication_coefficiant,int extrication_frequency);
    //Bezier(polydatapointer, polydatapointer, double grind_depth, double effector_diameter, double covering);
    //Bezier(polygonMesh, polygonMesh, double grind_depth, double effector_diameter, double covering);

    ~Bezier();

    /**@brief generateTrajectory is the most important function of this library. This public function contains the process of 3D path generation.
     * @param[out] way_points_vector in order to save robot poses.
     * @param[out] color_vector in order to know if pose is an extrication pose or not (Use to display path)
     * @param[out] index_vector in order to know index of start and end pose in each passe (Use to display path)
     * @return boolean flag reflects the function proceedings.
     */
    bool generateTrajectory(std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &way_points_vector, std::vector<bool> &color_vector, std::vector<int> &index_vector);

    /**@brief public function used to save all dilated polydatas.
     * @param[in] path it's the path of folder where we want saved polydatas.
     * @return boolean flag reflects the function proceedings.
     **/
    bool saveDilatedMeshes (std::string path);

    /**@brief public function used to get the private parameters @ref vector_dir_
     * @return @ref vector_dir_
     **/
    Eigen::Vector3d get_vector_direction();

    /**@brief public function used to display normals in RVIZ
     * @param[in] way_points_vector vector containing robot poses
     * @param[in] points_color_viz it's a bool vector using to distinguish real path and extrication path
     * @param[out] &normal_publisher publisher used to display normals in RVIZ
     * @note Boolean vector and trajectory vector need to have same sizes.
     **/
    void displayNormal( std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector, std::vector<bool> points_color_viz, ros::Publisher &normal_publisher);


    /**@brief Generate a trajectory (LINE_STRIP) marker and publish it.
     * @param[in] way_points_vector 3D trajectory vector (containing poses)
     * @param[in] points_color_viz it's a bool vector using to distinguish real path and extrication path
     * @param[out] &trajectory_publisher reference to a ros publisher used to display trajectory marker
     * @note Boolean vector and trajectory vector need to have same sizes.
     */
    void displayTrajectory(std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > way_points_vector, std::vector<bool> points_color_viz, ros::Publisher &trajectory_publisher);

    /**@brief public function used to display mesh in RVIZ
     * @param[in] mesh_path path of mesh
     * @param[out] &mesh_publisher publisher used to display the mesh in RVIZ
     **/
    void displayMesh(ros::Publisher &mesh_publisher, std::string mesh_path);

  private:
    /** @brief input mesh */
    vtkSmartPointer<vtkPolyData> inputPolyData_;
    /** @brief  default mesh */
    vtkSmartPointer<vtkPolyData> defaultPolyData_;
    /** @brief vector containing several dilated meshes */
    std::vector<vtkSmartPointer<vtkPolyData> > dilationPolyDataVector_;
    /** @brief grinding depth (in meters) */
    double grind_depth_;
    /** @brief extrication depth equal of how many grind_depth (coefficiant) */
    int extrication_coefficiant_;
    /** @brief new extrication mesh generated each 1/extrication_frequency times */
    int extrication_frequency_;
    /** @brief diameter of effector (in meters) */
    double effector_diameter_;
    /** @brief Percentage of covering (decimal value) */
    double covering_;
    /** @brief normal mesh */
    Eigen::Vector3d mesh_normal_vector_;
    /** @brief Vector direction for slicing */
    Eigen::Vector3d vector_dir_;

    ///@brief Function used to display some bezier library's parameters (effector diameter, grind depth and covering)
    void printSelf();

    /**@brief This function used vtkImplicitModeller in order to dilate inputpolydata surface.
     * @param[in] depth depth for grind process (passe depth)
     * @param[in] polydata Polydata we would like to dilate
     * @param[out] dilate_poly_data dilate_poly_data is the result of input_poly_data (this->inputPolyData) dilation.
     * @return boolean flag reflects the function proceedings.
     * @bug dilation problem detected when depth is to high, dilated mesh has unexpected holes
     * These holes are problematic. In fact, when cutting process is called on dilated mesh, slices are divided in some parts due to these holes
     * and this affects the path generation, especially for extrical trajectory. We have to find a solution, perhaps find best parameters
     * in order to resolve this problem.
     */
    bool dilatation(double depth, vtkSmartPointer<vtkPolyData> poly_data,vtkSmartPointer<vtkPolyData> &dilate_poly_data);

    /**@brief This function allows to optimize path generation. When passes are generated (dilation), we make an intersection between
     * dilated mesh and default in order to only save useful part of mesh (for useful poses).
     * @param[in, out] poly_data mesh generated after dilation and before intersection
     * @return boolean flag reflects the function proceedings.
     * @bug :Sometimes, unexpected part of mesh are save.
     */
    bool defaultIntersectionOptimisation(vtkSmartPointer<vtkPolyData> &poly_data);

    /**@brief generateCellNormals function determines normal of cells in a mesh
     * @param[in, out] &poly_data polydata in which normals are detected
     * @return boolean flag reflects the function proceedings.
     */
    bool generateCellNormals(vtkSmartPointer<vtkPolyData> &poly_data);

    /**@brief generatePointNormals function determines normal of points in a mesh
     * @param[in, out] &poly_data polydata in which normals are detected
     * @return boolean flag reflects the function proceedings.
     */
    bool generatePointNormals(vtkSmartPointer<vtkPolyData> &poly_data);

    /**@brief Function allows to load a Polydata from a PLY file
     * @param[in] filename string name of file. filename contains path and name of file
     * @param[out] poly_Data polydata loaded.
     * @return boolean flag reflects the function proceedings.
     */
    bool loadPLYPolydata(std::string filename, vtkSmartPointer<vtkPolyData> &poly_Data);

    /**@brief Function allows to save a Polydata from a PLY file
     * @param[in] filename string name of file. filename contains path and name of file
     * @param[in] poly_data polydata we have to save.
     * @return boolean flag reflects the function proceedings.
     */
    bool savePLYPolyData(std::string filename, vtkSmartPointer<vtkPolyData> poly_data);

    /**@brief This function is an application of Random Sample consensus theorem
     * Goal is to find a plan model of our inputpolydata
     * @note Threshold = max of inputpolydata dimensions in order to have a model of all point of inputpolydata
     * Using inputPolyData as mesh reference and determines mesh_normal_vector, global normal of our mesh reference.
     */
    void ransac();

    /** @brief Goal of generateDirection is to find a cut direction.
     * This direction is automatically determined using mesh normal (ransac plan model equation).
     * In fact, in this process we would like a cut direction orthogonal to the mesh normal (for grind reason).
     * So, cut direction is a vector belong ransac plan model.
     */
    void generateDirection();

    /**@brief calculate scalar product between cut direction and all points of polydata pointcloud,
     * to get length to cut and to determine line_number expected. (difference between max & min results)
     * @param[in] poly_data To get all points of mesh
     * @param[in] vector_dir cut direction
     * @return line_number expected to cut mesh with covering expected
     */
    unsigned int determineSliceNumberExpected(vtkSmartPointer<vtkPolyData> poly_data, Eigen::Vector3d vector_dir);

    /**Intern cutMesh function used to return real line number, using scalar product between each
     * line.point[0] and cut direction. In fact, if there are holes in polydata, line number returned by VTK is wrong.
     * @param stripper slice cut without organization
     * @param vector_dir cut direction
     * @return Real line number
     */
    unsigned int getRealSliceNumber(vtkSmartPointer<vtkStripper> stripper, Eigen::Vector3d vector_dir);

    /**@brief The CutMesh function goal is to cut several slices in a VTKPolyData (Mesh type on VTK)
     * @param[in] PolyData This VtkPolyData represents the mesh we have to cut
     * @param[in] line_number_expected Number of slices expected
     * @param[in] cut_dir Eigen vector containing plan cut direction (x,y,z)
     * @param[out] stripper Reference on a vtkstripper smartPointer used to get cutter results.
     * @return boolean flag reflects the function proceedings.
    */
    bool cutMesh(vtkSmartPointer<vtkPolyData> PolyData, Eigen::Vector3d cut_dir, unsigned int line_number_expected, vtkSmartPointer<vtkStripper> &stripper);

    /**@brief This function checks orientation of lines and compares them with a reference vector (cut_direction.dot(mesh_normal))
     * So, all lines have the same direction.
     * @param[in, out] lines lines is a vector containing several lines. Each line is a vector of pairs (first point position, second z normal)
     * @return boolean flag reflects the function proceedings.
    */
    bool checkOrientation(std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > &lines);

    /**@brief removeNearNeighborPoints function is used to remove too close points in a point clouds.
     * In fact, too close points could generate errors in vector spaces generation (vector normalization
     * and division by 0). In order, to remove potential problems in trajectory generation (orientation
     * of flange) removeNearNeighborPoints function reads point cloud and remove too close points.
     * @param[in, out] &lines lines is a vector containing several lines. Each line is a vector of pairs (first point position, second z normal)
    */
    void removeNearNeighborPoints(std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > &lines);

    /**@brief Helped with z normals and point cloud, this function determines x and y normals
     *        in order to generate the vector shape(robot pose).
     * @param[in] point Eigen vector of point position
     * @param[in] point_next Eigen vector of next point position
     * @param[in] normal Eigen vector of point normal (z normal)
     * @param[out] &pose Eigen affine matrix contains x,y,z normals and point position (robot pose)
     * @return boolean flag reflects the function proceedings.
     */
    bool generateRobotPoses(Eigen::Vector3d point, Eigen::Vector3d point_next, Eigen::Vector3d normal, Eigen::Affine3d &pose);

    /**@brief The generateStripperClouds function is an important function in this path generation process.
     *        It generates 3D trajectories on a mesh(vtkPolyData) with help of other functions.
     *        So, it generates stripper clouds on a surface.
     *        Process Mesh -> Cut mesh -> generate z normals and get point positions-> reorganized lines (order and orientations)-> remove too close points
     * @param[in] PolyData This VtkPolyData represents the mesh we have to cut
     * @param[out] lines lines is a vector containing several lines. Each line is a vector of pairs (first point position, second z normal)
     * @return boolean flag reflects the function proceedings.
     */
    bool generateStripperOnSurface(vtkSmartPointer<vtkPolyData> PolyData, std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > &lines);

    /**@brief Function used for extrication path. Extrication between two lines. It allows to find the closest extrication line of a point.
     * @param[in] point_vector Eigen vector of point position
     * @param[in] extrication_lines lines is a vector containing several lines. Each line is a vector of pairs (first point position, second z normal)
     *                                  extrication_lines are lines generated on dilated (extrication) mesh.
     * @return int value equal to index of closest line in extrication_lines vector.
     */
    int seekClosestLine(Eigen::Vector3d point_vector, std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > > extrication_lines);

    /**@brief Function used for extrication path. Extrication between two lines. It allows to find the closest extrication point of a point in a line.
     * @param[in] point_vector Eigen vector of point position
     * @param[in] extrication_line Line is a vector of pairs (first point position, second z normal)
     *                                  extrication_line are the closest line on the dilated (extrication) mesh.
     * @return int value equal to index of closest point in extrication_line vector.
     */
    int seekClosestPoint(Eigen::Vector3d point_vector, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > extrication_line);

    /**@brief Function used for extrication path. Extrication between two passes. It allows to find the closest extrication point of a point in the last extrication line.
     * @param[in] point_vector Eigen vector of point position
     * @param[in] extrication_poses extrication poses is a vector containing all poses generated in the stripper between a passe i and a passe i+1.
     * @return int value equal to index of closest point in extrication_poses vector.
     */
    int seekClosestExtricationPassPoint(Eigen::Vector3d point_vector, std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > extrication_poses);
};

#endif

