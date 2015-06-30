cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_FLAGS "-fPIC")
project(bezier_library)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(PCL 1.8.0 REQUIRED)
find_package(VTK 6.3 REQUIRED)
#include(${VTK_USE_FILE})

catkin_package(
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})

include_directories(include ${VTK_USE_FILE} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}) # Order matters!
add_definitions(${PCL_DEFINITIONS}) # ${VTK_DEFINITIONS} crashes compilation

add_library (bezier_library_obj OBJECT src/bezier_library.cpp)
add_library (${PROJECT_NAME} $<TARGET_OBJECTS:bezier_library_obj>)

install(
    TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

option(BUILD_${PROJECT_NAME}_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ON)
IF(BUILD_${PROJECT_NAME}_DOCUMENTATION)

  find_package(Doxygen)
  IF(NOT DOXYGEN_FOUND)
    message(FATAL_ERROR "Doxygen is needed to build the documentation.")
  ENDIF()

  # Path to the images
  set(doxy_images            ${PROJECT_SOURCE_DIR}/documentation/images) # PROJECT_LOGO and IMAGE_PATH in doxy.in
  set(doxy_input             ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/) # INPUT in doxy.in

  set(doxyfile_in          ${CMAKE_CURRENT_SOURCE_DIR}/documentation/${PROJECT_NAME}.doxy.in)
  set(doxyfile             ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.doxy)
  set(doxy_html_index_file ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_doc/index.html)
  set(doxy_output_root     ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_doc) # OUTPUT_DIRECTORY in doxy.in

  configure_file(${doxyfile_in} ${doxyfile} @ONLY)

  add_custom_command(OUTPUT ${doxy_html_index_file}
                     COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
                     MAIN_DEPENDENCY ${doxyfile} 
                     DEPENDS ${PROJECT_NAME}
                     COMMENT "Generating HTML documentation")

  add_custom_target(${PROJECT_NAME}_doc ALL DEPENDS ${doxy_html_index_file})
ENDIF()
