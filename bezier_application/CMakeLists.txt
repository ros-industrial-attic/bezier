cmake_minimum_required(VERSION 2.8.3)
project(bezier_application)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp bezier_library tf tf_conversions moveit_ros_planning_interface visualization_msgs)
find_package(PCL 1.8.0 REQUIRED)
find_package(VTK 7.0 REQUIRED COMPONENTS vtkFiltersHybrid NO_MODULE)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS bezier_library)

include_directories(${VTK_USE_FILE} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}) # Order matters!
add_definitions(${PCL_DEFINITIONS} ${VTK_DEFINITIONS})

add_executable (bezier_application src/bezier_application.cpp)
if(VTK_LIBRARIES) # May never happen because we expect VTK 6.3 minimum
  target_link_libraries (bezier_application ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES} bezier_library)
else()
  target_link_libraries (bezier_application ${catkin_LIBRARIES} vtkHybrid vtkWidgets ${PCL_LIBRARIES} bezier_library)
endif()

