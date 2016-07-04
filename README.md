 [![Institut Maupertuis logo](https://avatars1.githubusercontent.com/u/12760694?v=3&s=80)](http://www.institutmaupertuis.fr) Bezier
=============================

ROS-Industrial Special Project: 6D tool path planner

[Pierre Étienne Bézier](https://en.wikipedia.org/wiki/Pierre_B%C3%A9zier) September 1, 1910 – November 25, 1999; *was a French engineer and one of the founders of the fields of solid, geometric and physical modeling as well as in the field of representing curves, especially in CAD/CAM systems. As an engineer at Renault, he became a leader in the transformation of design and manufacturing, through mathematics and computing tools, into computer-aided design and three-dimensional modeling. Bézier patented and popularized, but did not invent the Bézier curves and Bézier surfaces that are now used in most computer-aided design and computer graphics systems.*

Travis CI
=========
[![Build Status](https://travis-ci.org/ros-industrial-consortium/bezier.svg?branch=indigo-devel)](https://travis-ci.org/ros-industrial-consortium/bezier)

General information
-------------------
This project has been developed by the [Institut Maupertuis](http://www.institutmaupertuis.fr), a French research institute that is working on robotic industrial processes.
This project goal is to create an automatic grinding path generator for 6-axis robots working on diverse/random meshes.

Bezier planner generates robot poses (3D trajectories) in harmony with a grinding process.
It is able to create rectilinear trajectories on complicated surfaces (3D surfaces) and to dilate them in all directions in order to grind defects with a pass principle.

![bezier_application](bezier_library/doc/bezier_application.png)

Directories in the project
--------------------------

| Directory  | Description
------------ | -----------
`bezier` | Meta-package
`bezier_application` | Example usage of the `bezier_library` for grinding operations on a Fanuc R-1000iA robot.
`bezier_library` | Library containing several path planning algorithms

Dependencies
------------
- [Robot Operating System](http://wiki.ros.org/ROS/Installation)
- [`industrial-core`](http://wiki.ros.org/industrial_core)
- [`fanuc`](http://wiki.ros.org/fanuc)
- [`Visualization Toolkit`](https://gitlab.kitware.com/vtk/vtk/) version `7.0` or later
- [`Point Cloud Library`](https://github.com/PointCloudLibrary/pcl) version `1.8.0` or later. :warning: PCL has to be compiled against the same VTK version that is used for this package.
- `C++11` is required 

This package has been tested with Ubuntu 14.04 and ROS Indigo.

Documentation
-------------
Please read [bezier_library/doc/README.md](bezier_library/doc/README.md).

Quick help
----------

**Build**

Clone this repository into your catkin source folder and build the workspace:
```bash
cd $(catkin_workspace)/src
git clone https://github.com/ros-industrial-consortium/bezier.git
```

**Launch**

```bash
roslaunch bezier_application bezier_application_m20ia.launch surfacing_mode:=true mesh_cad:=plane/plane_defect.ply
```

In this example, `bezier_application` will be launched with `plane_defect.ply` as the CAD mesh and the grinding will be done in surface mode, meaning that we only pass on the surface of the mesh to smooth it.

```bash
roslaunch bezier_application bezier_application_m20ia.launch mesh_cad:=plane/plane.ply mesh_defect:=plane/plane_defect.ply
```

In this example, `bezier_application` will be launched with `plane.ply` as the CAD mesh and `plane_defect.ply` as defect mesh.

Others examples of meshes are available in:
```bash
$(catkin_workspace)/src/bezier/bezier_application/meshes
```

