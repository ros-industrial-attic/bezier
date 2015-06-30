##Documentation generation
=========

#Required
-------------

 -> [Doxygen](http://www.stack.nl/~dimitri/doxygen/) : used to generate documentation as from code

 -> Flag "ON" in CMakeList.txt : 
		option(BUILD_${PROJECT_NAME}_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ON)


#Generation
-------------

**have to build package :**
cd $(ros_repository)/build && make


#Link 
-------------

$(ros_repository)/build/bezier/bezier_library/html/index.html

