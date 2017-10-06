 [![Institut Maupertuis logo](https://avatars1.githubusercontent.com/u/12760694?v=3&s=80)](http://www.institutmaupertuis.fr) Bezier
===

ROS-Industrial Special Project: 6D tool path planner

[Pierre Étienne Bézier](https://en.wikipedia.org/wiki/Pierre_B%C3%A9zier) September 1, 1910 – November 25, 1999; *was a French engineer and one of the founders of the fields of solid, geometric and physical modeling as well as in the field of representing curves, especially in CAD/CAM systems. As an engineer at Renault, he became a leader in the transformation of design and manufacturing, through mathematics and computing tools, into computer-aided design and three-dimensional modeling. Bézier patented and popularized, but did not invent the Bézier curves and Bézier surfaces that are now used in most computer-aided design and computer graphics systems.*

# General information
This project has been developed by the [Institut Maupertuis](http://www.institutmaupertuis.fr), a French research institute that is working on robotic industrial processes.
This project goal is to create an automatic grinding path generator for 6-axis robots working on diverse/random meshes, it heavily relies on the [Robot Operating System](www.ros.org).

Bezier planner generates robot poses (3D trajectories) in harmony with a grinding process.
It is able to create rectilinear trajectories on complex surfaces (3D surfaces) and to dilate them in all directions in order to grind defects with a pass principle.

![bezier_application](bezier_library/doc/bezier_application.png)

[![Fanuc grinding Youtube video](https://github.com/InstitutMaupertuis/fanuc_grinding/raw/indigo-devel/documentation/fanuc_grinding.jpg)](https://www.youtube.com/watch?v=aLp8zxx1PnU)
Click the image to see a video of a Fanuc robot grinding thanks to the Bezier library, the project is [fanuc_grinding](https://github.com/InstitutMaupertuis/fanuc_grinding).

# Dependencies
This package has been tested with Ubuntu 16.04 and ROS Kinetic.

## VTK 7.1
VTK 7.1 or higher is required in order to compile this project.

Here are quick steps to download, compile and install VTK 7.1:

```bash
mkdir -p $HOME/libraries/VTK-7.1/build_release
cd $HOME/libraries/VTK-7.1/
wget http://www.vtk.org/files/release/7.1/VTK-7.1.1.zip
unzip VTK-7.1.1.zip
mv VTK-7.1.1 src
cd build_release
cmake ../src -DCMAKE_BUILD_TYPE=Release
make -j4
```

Install with:
```bash
sudo make -j4 install
sudo ldconfig
```

## wstool
Install [wstool](wiki.ros.org/wstool).

## rosdep
Install, initialize and update [rosdep](http://wiki.ros.org/rosdep).

# Compiling
Create a catkin workspace and clone the project:

```bash
mkdir -p catkin_workspace
cd catkin_workspace
git clone https://github.com/ros-industrial/bezier.git
wstool init src bezier/bezier.rosinstall
mv bezier src
```

## Resolve ROS dependencies
```bash
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
```

## Compile
Use `catkin_make` or `catkin tools`:

```bash
catkin_make
catkin build
```

# Launch
Source the `devel/setup.bash` file and launch the surfacing application:
```bash
roslaunch bezier_application fanuc_m20ia_surfacing.launch surfacing_mode:=true mesh_cad:=plane/plane_defect.ply
```
Use the following command for the painting application:
```bash
roslaunch bezier_application fanuc_m20ia_painting.launch mesh_cad:=ocean/ocean.ply
```

In this example, `bezier_application` will be launched with `plane_defect.ply` as the CAD mesh and the grinding will be done in surface mode, meaning that we only pass on the surface of the mesh to smooth it.

```bash
roslaunch bezier_application fanuc_m20ia_surfacing.launch mesh_cad:=plane/plane.ply mesh_defect:=plane/plane_defect.ply
```

In this example, `bezier_application` will be launched with `plane.ply` as the CAD mesh and `plane_defect.ply` as defect mesh.

Others examples of meshes are available in:
```bash
$(catkin_workspace)/src/bezier/bezier_application/meshes
```

# Documentation

Please read [bezier_library/doc/README.md](bezier_library/doc/README.md).
