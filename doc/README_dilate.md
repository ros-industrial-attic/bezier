 [![Institut Maupertuis logo](https://avatars1.githubusercontent.com/u/12760694?v=3&s=80)](http://www.institutmaupertuis.fr) dilate
===

This function is implemented with the `VTK` library. It uses a [`vtkImplicitModeller`](http://www.vtk.org/doc/release/7.0/html/classvtkImplicitModeller.html) to generate a dilated mesh.
The principle of dilation in 3 dimensions can be seen as a sphere of a given diameter rolling all around a surface (the mesh).

![dilation](dilation.png)                   | ![dilate](dilate.png)
------------------------------------------- | ------------------------------
Dilation principle                          | Example of dilation (grey = dilated mesh)

