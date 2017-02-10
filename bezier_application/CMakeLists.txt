cmake_minimum_required(VERSION 2.8.3)
project(bezier_application)
add_compile_options(-std=c++11  -Wall -Wextra)

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

set(PAINTING_APPLICATION "painting_application")
set(SURFACING_APPLICATION "surfacing_application")

add_executable(${PAINTING_APPLICATION} src/${PAINTING_APPLICATION}.cpp)
if(VTK_LIBRARIES) # May never happen because we expect VTK 7.0 minimum
  target_link_libraries(${PAINTING_APPLICATION} ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})
else()
  target_link_libraries(${PAINTING_APPLICATION} ${catkin_LIBRARIES} vtkHybrid vtkWidgets ${PCL_LIBRARIES})
endif()

add_executable(${SURFACING_APPLICATION} src/${SURFACING_APPLICATION}.cpp)
if(VTK_LIBRARIES) # May never happen because we expect VTK 7.0 minimum
  target_link_libraries(${SURFACING_APPLICATION} ${catkin_LIBRARIES} ${VTK_LIBRARIES} ${PCL_LIBRARIES})
else()
  target_link_libraries(${SURFACING_APPLICATION} ${catkin_LIBRARIES} vtkHybrid vtkWidgets ${PCL_LIBRARIES})
endif()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_bezier_application.xml)
endif()

