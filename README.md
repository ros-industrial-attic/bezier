 [![Institut Maupertuis logo](https://avatars1.githubusercontent.com/u/12760694?v=3&s=80)](http://www.institutmaupertuis.fr) Bezier
=============================

ROS-Industrial Special Project: 5D tool path planner

[Pierre Étienne Bézier](https://en.wikipedia.org/wiki/Pierre_B%C3%A9zier) September 1, 1910 – November 25, 1999; *was a French engineer and one of the founders of the fields of solid, geometric and physical modeling as well as in the field of representing curves, especially in CAD/CAM systems. As an engineer at Renault, he became a leader in the transformation of design and manufacturing, through mathematics and computing tools, into computer-aided design and three-dimensional modeling. Bézier patented and popularized, but did not invent the Bézier curves and Bézier surfaces that are now used in most computer-aided design and computer graphics systems.*

Description
-----------

This project has been developed by the [Institut Maupertuis](http://www.institutmaupertuis.fr), a French research institute that is working on robotic industrial processes.
This project goal is to create an automatic grinding path generator for 6-axis robots working on diverse/random meshes.

Bezier planner generates robot poses (3D trajectory) in harmony with a grinding process.
It is able to create rectilinear trajectories on complicated surfaces (3D surfaces) and to dilate them in all directions in order to grind defects with a pass principle.

Dependencies
------------

- [ROS](http::wiki.ros.org/ROS/Installation) (Robot Operating System)
- [`insdustrial-core`](http://wiki.ros.org/industrial_core)
- [`fanuc`](https://github.com/InstitutMaupertuis/fanuc) :warning: Joint limits have been tweaked
- [`fanuc experimental`](https://github.com/InstitutMaupertuis/fanuc_experimental) :warning: Joint limits have been tweaked
- [`VTK 7.0`](https://gitlab.kitware.com/vtk/vtk/merge_requests/213) or later, using the `master` branch is fine.
- [`Point Cloud Library`](https://github.com/PointCloudLibrary/pcl) version 1.8 or trunk

Bezier has been tested with ROS Indigo under Ubuntu 14.04.

Documentation
-------------

Please read [bezier_library/doc/README.md](bezier_library/doc/README.md).

Contents
--------

- `bezier`: meta-package
- `bezier_library`: library containing several functions for robotic grinding
- `bezier_application`: example usage of the `bezier_library`

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
roslaunch bezier_application bezier_application.launch filename:=oriented_complicated_mesh.ply
```

In this example, `bezier_application` will be launched with `oriented_complicated_mesh.ply` as input mesh and `oriented_complicated_mesh_default.ply` as default mesh.

Others examples of meshes are available in:
```bash
$(catkin_workspace)/src/bezier/bezier_application/meshes
```

