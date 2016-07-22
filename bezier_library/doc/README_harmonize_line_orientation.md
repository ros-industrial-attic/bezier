 [![Institut Maupertuis logo](https://avatars1.githubusercontent.com/u/12760694?v=3&s=80)](http://www.institutmaupertuis.fr) harmonizeLineOrientation
===

This function makes sure that all the trajectories (vector of poses) have the same orientation.
It takes a vector acting as an orientation reference. The trajectory is reversed if it's
orientation does not match the one of the reference. The X axis of each pose will also be reversed thus allowing the robot to move along the right direction, see [invertXAxisOfPoses](README_invert_x_axis_of_poses.md).

The following animation shows the result of the harmonzation on a trajectory (the blue line is reversed) : 
![harmonize_line_orientation](harmonize_line_orientation.gif)

