#ifndef BEZIER_HPP
#define BEZIER_HPP

// STL
#include <map>
#include <string>
#include <vector>

// Boost
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// VTK
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkSTLReader.h>
#include <vtkOBJExporter.h>
#include <vtkPLYWriter.h>
#include <vtkSTLWriter.h>
#include <vtkPolyDataNormals.h>
#include <vtkImplicitModeller.h>
#include <vtkMarchingCubes.h>
#include <vtkStripper.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkTriangleFilter.h>
#include <vtkMath.h>
#include <vtkFloatArray.h>
#include <vtkKdTreePointLocator.h>
#include <vtkReverseSense.h>
#include <vtkPointData.h>
#include "error_observer.h"

// PCL
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/PolygonMesh.h>

// ROS
#include <rviz_visual_tools/rviz_visual_tools.h>

// Exceptions
#include "append_bezier_exception.hpp"

/**
 * Because it's missing in Eigen
 */
namespace EigenSTL
{
typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > vector_Vector4d;
}

/**
 * Bezier abstract class
 */
class Bezier
{
public:
  /**
   * Convenience typedef
   */
  typedef pcl::PointXYZ PointT;

  /**
   * Convenience typedef
   */
  typedef pcl::PointCloud<PointT> PointCloudT;
protected:

public:
  Bezier();
  virtual ~Bezier()
  {
  }

  /**
   * @returns the name of the current Bezier object
   */
  std::string
  virtual
  name() = 0;

  /**
   * Generate a trajectory using the input mesh and parameters
   * @param[out] trajectory
   * @param[out] is_grinding_pose whether the pose is a grinding or extrication one
   * @param[in] display_markers
   * @return An empty string on success, an error string otherwise
   * @note trajectory and is_grinding_pose are always the same size
   */
  std::string virtual generateTrajectory(EigenSTL::vector_Affine3d &trajectory,
                                         std::vector<bool> &is_grinding_pose,
                                         bool display_markers) = 0;

  /**
   * Blocking call to make sure there is at least one subscriber to the RViz visual tool publisher
   */
  void waitForRvizVisualToolsSubscriber();

protected:
  /**
   * VTK error observer to manage failures when loading files etc.
   */
  vtkSmartPointer<ErrorObserver> vtk_observer_;

  /**
   * A visual tool object to allow publishing markers (text, trajectories etc.)
   */
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  /**
   * Meshes provided to Bezier
   */
  std::vector<vtkSmartPointer<vtkPolyData> > input_meshes_;

  /**
   * Check if the parameters provided to Bezier are correct
   */
  std::string virtual validateParameters() = 0;

  /**
   * Append a OBJ, PLY or STL mesh into the vector of meshes input_meshes_
   * @param[in] file_absolute_path
   * @return True if successful, false otherwise
   */
  bool appendInputMesh(const std::string file_absolute_path);

  /**
   * Save a mesh file on the disk, supported formats are PLY and STL
   * @param[in] file_absolute_path
   * @param[in] polydata the mesh to be saved
   * @return True if successful, false otherwise
   */
  bool saveMesh(const std::string file_absolute_path,
                const vtkSmartPointer<vtkPolyData> &polydata);

  /**
   * Publish a PLY mesh on a ROS topic
   * @param[in] mesh_publisher
   * @param[in] mesh_path
   * @param[in] red
   * @param[in] green
   * @param[in] blue
   * @param[in] alpha
   * @param[in] frame_id
   */
  void displayMesh(const std::shared_ptr<ros::Publisher> &mesh_publisher,
                   const std::string mesh_path,
                   const float red,
                   const float green,
                   const float blue,
                   const float alpha = 1.0,
                   std::string frame_id = "/base_link");

  /**
   * Display a colored trajectory
   * @param[in] trajectory
   * @param color
   * @param display_normals display Z normal
   * @param display_labels display pose ID
   */
  void displayTrajectory(const EigenSTL::vector_Affine3d &trajectory,
                         rviz_visual_tools::colors color,
                         const bool display_normals = false,
                         const bool display_labels = false);

  /**
   * Estimate a global normal for a mesh
   * @param[in] polydata
   * @param[out] mesh_normal
   * @param[in] iterations RANSAC iterations
   * @return True if successful, false otherwise
   */
  bool estimateGlobalMeshNormal(vtkSmartPointer<vtkPolyData> &polydata,
                                Eigen::Vector3d &mesh_normal,
                                const unsigned iterations = 2000);

  /**
   * Automatically computes a cutting orientation vector given a polydata.\n
   * The orientation computed is normalized
   * @param[in] polydata
   * @param[out] mesh_orientation The global mesh orientation
   * @param[out] orientation The slicing orientation
   * @return True if successful, false otherwise
   */
  bool estimateSlicingOrientation(vtkSmartPointer<vtkPolyData> &polydata,
                                  Eigen::Vector3d &mesh_orientation,
                                  Eigen::Vector3d &orientation);

  /**
   * Slice a mesh with a plane and get the output stripper
   * @param[in] polydata
   * @param[in] plane_equation
   * @param[in] origin
   * @param[out] stripper
   * @return True if successful, false otherwise
   * @warning Returns true even if the stripper is empty!
   */
  bool sliceMeshWithPlane(const vtkSmartPointer<vtkPolyData> &polydata,
                          const Eigen::Vector4d &plane_equation,
                          const Eigen::Vector3d &origin,
                          vtkSmartPointer<vtkStripper> &stripper);

  /**
   * Slice a mesh with multiple planes and fill a vector of strippers
   * @param[in] polydata
   * @param[in] plane_equations
   * @param[in] origin
   * @param[out] strippers
   * @return True if successful, false otherwise
   * @warning Empty strippers are not pushed into the vector!
   *
   */
  bool sliceMeshWithPlanes(const vtkSmartPointer<vtkPolyData> &polydata,
                           const EigenSTL::vector_Vector4d &plane_equations,
                           const Eigen::Vector3d &origin,
                           std::vector<vtkSmartPointer<vtkStripper> > &strippers);

  /**
   * Compute points and cells normals of a polydata
   * @param[in] polydata
   * @return True if successful, false otherwise
   */
  bool computeNormals(vtkSmartPointer<vtkPolyData> &polydata);

  /** @brief This function uses vtkImplicitModeller in order to dilate the polydata surface/mesh
   *  @param[in,out] polydata the mesh to be dilated
   *  @param[in] radius of the dilation
   *  @return True if successful, false otherwise
   *  @bug Dilation problem happens when the radius is too high, the dilated mesh has unexpected holes.
   */
  bool
  dilate(vtkSmartPointer<vtkPolyData> &polydata,
         const double radius);

  /**
   * This removes all points "below" the dilated mesh using the base mesh.\n
   * During the dilation of a surface the mesh is expanded across all orientations, this means the
   * result is a closed mesh. In our process we only are interested in the upper part of the mesh.\n
   * A k-d tree is built and scalar products allows to determine if the current cell (= face) is
   * "above" or "below" the mesh.
   * @param[in] base_polydata_ the normals are computed in the polydata
   * @param[in] dilated_polydata the mesh to be processed
   * @param[out] upper_part_dilated_polydata the resulting mesh
   * @return True if successful, false otherwise
   * @note This probably should be implemented in VTK rather than filtered afterwards!
   */
  bool keepUpperPartofDilatedMesh(vtkSmartPointer<vtkPolyData> &base_polydata_,
                                  const vtkSmartPointer<vtkPolyData> &dilated_polydata,
                                  vtkSmartPointer<vtkPolyData> &upper_part_dilated_polydata);

  /**
   * Get an rviz_visual_tool color given an index
   * @param[in] index of the color
   * @returns a color enum from rviz_visual_tools corresponding to the index
   */
  rviz_visual_tools::colors visualToolsColorFromIndex(const unsigned index);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
