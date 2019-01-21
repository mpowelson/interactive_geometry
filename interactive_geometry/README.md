# Introduction

This is a ROS package for interacting with geometry of various shapes in RVIZ. The idea is that a person might want to drag a geometric object around in RVIZ and then export that for use elsewhere. This is currently achieved by using a triange array marker to display a mesh and then adding interactive markers at the edges that dynamically resize the mesh. A menu interactive marker allows the user to export the mesh.


Currently only an half-ellipsoid is implemented, but other choices might be

* Rectangular prism
* Full ellipsoid
* Plane
* Some generic surface function

To use

1) Launch the interactive geometry package
2) Move the mesh by clicking in the center of the ellipsoid (the 6 DOF interactive marker)
3) Resize the mesh by dragging the interactive markers at the edges of the mesh
4) To export, right click on the 6 DOF marker and select method
