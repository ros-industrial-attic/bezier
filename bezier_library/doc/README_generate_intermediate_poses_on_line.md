 [![Institut Maupertuis logo](https://avatars1.githubusercontent.com/u/12760694?v=3&s=80)](http://www.institutmaupertuis.fr) generateRobotPosesAlongStripper
===

This function generates a specified number of point between a start and an end point along a line.
Each point is located at an equally far away position of his predecessor and generated using the formula : `Point = StartPoint + d*LineVector`
with `d` the distance between the start point and the generated point and `LineVector` the vector between the
start point and the end point of the line. 

The following animation illustrates the generation of the intermediate poses :

![intermediate_poses](intermediate_poses.gif)

The following picture illustrates the result of the process :

![intermediate_poses_picture](intermediate_poses.png) 
