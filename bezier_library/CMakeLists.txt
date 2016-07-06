cmake_minimum_required(VERSION 2.8.3)
project(bezier_library)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             rviz_visual_tools)

find_package(PCL 1.8.0 REQUIRED)
find_package(VTK 7.0 REQUIRED) # You need https://gitlab.kitware.com/vtk/vtk/merge_requests/213
include(${VTK_USE_FILE})

catkin_package(CATKIN_DEPENDS
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               roscpp 
               rviz_visual_tools)

include_directories(include ${VTK_USE_FILE} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}) # Order matters!
add_definitions(${PCL_DEFINITIONS}) # ${VTK_DEFINITIONS} crashes compilation

add_library(${PROJECT_NAME}_obj OBJECT src/${PROJECT_NAME}.cpp)
set_property(TARGET ${PROJECT_NAME}_obj APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11 -fPIC -Wall")
add_library(${PROJECT_NAME} $<TARGET_OBJECTS:${PROJECT_NAME}_obj>)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${catkin_LIBRARIES})

add_library(bezier_grinding_surfacing_obj OBJECT src/bezier_grinding_surfacing.cpp)
set_property(TARGET bezier_grinding_surfacing_obj APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11 -fPIC -Wall")
add_library(bezier_grinding_surfacing $<TARGET_OBJECTS:bezier_grinding_surfacing_obj>)
target_link_libraries(bezier_grinding_surfacing ${PROJECT_NAME} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${catkin_LIBRARIES})

install(
    TARGETS ${PROJECT_NAME}
            bezier_grinding_surfacing
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_custom_target (${PROJECT_NAME}_doc 
    rosdoc_lite -o ${CATKIN_DEVEL_PREFIX}/doc/${PROJECT_NAME} ${CATKIN_DEVEL_PREFIX}/../src/bezier/${PROJECT_NAME})
