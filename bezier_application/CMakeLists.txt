cmake_minimum_required(VERSION 2.8.3)
project(bezier_application)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
             bezier_library
             moveit_ros_planning_interface 
             roscpp 
             rviz_visual_tools
             tf 
             tf_conversions
             visualization_msgs)

find_package(PCL 1.8.0 REQUIRED)
find_package(VTK 7.0 REQUIRED COMPONENTS vtkFiltersHybrid NO_MODULE)

## Declare a catkin package
catkin_package()

include_directories(${VTK_USE_FILE} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}) # Order matters!
add_definitions(${PCL_DEFINITIONS} ${VTK_DEFINITIONS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11 -Wall")
if(VTK_LIBRARIES) # May never happen because we expect VTK 7.0 minimum
  target_link_libraries(${PROJECT_NAME} bezier_library bezier_grinding_surfacing ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} bezier_library bezier_grinding_surfacing ${catkin_LIBRARIES} vtkHybrid vtkWidgets ${PCL_LIBRARIES})
endif()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_bezier_application.xml)
endif()
