# UniformPolygonTriangulation

Implementation of uniform adaptive polygon triangulation algorithm illustrated in paper "Adaptive and quality 3D meshing from imaging data" using VTK9.2.6

![image](https://github.com/gongwaner/UniformPolygonTriangulation/assets/29704759/88a5a538-54ff-4275-8c45-b03a61ef73c8)

(figure 5: Adaptive Triangulation. The red curve represents the isocontour, green points represent minimizers. Right: Case Table for Decomposing the Interior Cell into Triangles.)

Note: This algorithm adds new vertices during triangulation process.


**Input** 
- A list of ordered(orientation can be either clockwise or counter-clockwise) coplanar points(in vtkVector3d) that form a non-self-intersected polygon.
The algorithm works for both input polygon without any inner hole(s) and polygon with inner hole(s).
For polygon with inner hole(s), the inner hole(s) must be in opposite orientation as the outer contour. 
eg. if a polygon has 2 inner holes, and outer contour is clockwise, then the 2 inner holes must be counter-clockwise. 


**Algorithm**
 - Calculate the bounding box of the polygon
 - starting from bounding box's upper left corner, sweep the polygon in a left-to-right, upper-to-bottom order using specified square/pixel size
	 - for each square/pixel
	     - calculate the intersected polygon between square and polygon(polygon clipping)
	     - perform optimal triangulation(minimum weight triangulation) on intersected polygon
  - combine all triangulated polygon in each pixel into a combined polygon, this is the uniform triangulated polygon.

**Adaptivity**
- This is an adaptive triangulation algorithm that adds new vertices during triangulation. The algorithm doesn't have a parameter for setting the number of vertices added, but this can be somewhat controlled by setting the size of the pixel/square. 
Also the optimal triangulation can be done by using customized weight functions. Currently in the code I used aspect ratio as weight function(ie.the triangulation aims to triangulate with minimum aspect ratio of triangle).

**Advantage**
- VTK provides various filters that enables fast and robust triangulation of polygon, such as  [vtkTriangleFilter](https://vtk.org/doc/nightly/html/classvtkTriangleFilter.html) . However, due to the shape of polygon, it might produce silver triangles. By introducing new vertices, this algorithm (almost) eliminates the generation of silver triangles with relatively good performance.
