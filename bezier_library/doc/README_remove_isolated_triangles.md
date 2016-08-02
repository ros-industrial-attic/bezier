 [![Institut Maupertuis logo](https://avatars1.githubusercontent.com/u/12760694?v=3&s=80)](http://www.institutmaupertuis.fr) filterExtricationTrajectory
===

This function uses a [vtkPolyDataConnectivityFilter](http://www.vtk.org/doc/release/7.0/html/classvtkPolyDataConnectivityFilter.html) to remove small isolated triangles from a mesh.
It first extracts all the regions of the mesh (a region is a group of triangles connected to each other) and computes the total number of cells of each region.
If the number computed is less than the minimal value specified, the region is removed. At the end of the operation, a new vtkPolyData is built
using a [vtkAppendPolyData](http://www.vtk.org/doc/release/7.0/html/classvtkAppendPolyData.html) containing only the regions matching the filter criteria.

The following picture illustrates the process :

![remove_isolated_triangle_filter](remove_isolated_triangle_filter.png)
