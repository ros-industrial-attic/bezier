cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_FLAGS "-fPIC")
project(bezier_library)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(PCL 1.8.0 REQUIRED)
find_package(VTK 7.0 REQUIRED) # You need https://gitlab.kitware.com/vtk/vtk/merge_requests/213
include(${VTK_USE_FILE})

catkin_package(
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})

include_directories(include ${VTK_USE_FILE} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}) # Order matters!
add_definitions(${PCL_DEFINITIONS}) # ${VTK_DEFINITIONS} crashes compilation

add_library (bezier_library_obj OBJECT src/bezier_library.cpp)
set_property(TARGET bezier_library_obj APPEND_STRING PROPERTY COMPILE_FLAGS -Wall)
add_library (${PROJECT_NAME} $<TARGET_OBJECTS:bezier_library_obj>)

install(
    TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_custom_target (bezier_library_doc 
    rosdoc_lite -o ${CATKIN_DEVEL_PREFIX}/doc/bezier_library ${CATKIN_DEVEL_PREFIX}/../src/bezier/bezier_library)

