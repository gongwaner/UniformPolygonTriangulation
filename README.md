# UniformPolygonTriangulation

Implementation of uniform adaptive polygon triangulation algorithm illustrated in paper "Adaptive and quality 3D meshing from imaging data"(Zhang, Yongjie & Bajaj, Chandrajit & Sohn, Bong-soo, 2003) with slight changes using VTK9.2.6.

![image](https://github.com/gongwaner/UniformPolygonTriangulation/assets/29704759/e6a3d3d1-d49b-4c7e-91f5-4def7f8d7c12)


This algorithm adds new vertices during triangulation process. Works for both input polygon without any inner hole(s) and polygon with inner hole(s) and arbitrary shape(convex/concave). 

<br/>**Input** 
<br/>A list of ordered(orientation can be either clockwise or counter-clockwise) coplanar points(in vtkVector3d) that form a non-self-intersected polygon.
<br/>For polygon with inner hole(s), the inner hole(s) must be in opposite orientation as the outer contour. 
<br/>eg. if a polygon has 2 inner holes, and outer contour is clockwise, then the 2 inner holes must be counter-clockwise. 

<br/>**Algorithm**
 - Calculate the oriented bounding box of the polygon
 - starting from bounding box's upper left corner, sweep the polygon in a left-to-right, upper-to-bottom order using specified square/pixel size
	 - for each square/pixel
	     - calculate the intersected polygon between square and polygon(polygon clipping)
	     - perform optimal triangulation(minimum weight triangulation) on intersected polygon
  - combine all triangulated polygon in each pixel into a combined polygon, this is the uniform triangulated polygon.

<br/>**Adaptivity**
<br/>This is an adaptive triangulation algorithm that adds new vertices during triangulation. The algorithm doesn't have a parameter for setting the number of vertices added, but this can be somewhat controlled by setting the size of the pixel/square. 
<br/>Also the optimal triangulation can be done by using customized weight functions. Currently in the code I used aspect ratio as weight function(ie.the triangulation aims to triangulate with minimum aspect ratio of triangle).

<br/>**Advantage**
<br/>VTK provides various filters that enable fast and robust triangulation of polygon, such as  [vtkTriangleFilter](https://vtk.org/doc/nightly/html/classvtkTriangleFilter.html). However, due to the shape of polygon, it might produce silver triangles. By introducing new vertices, this algorithm (almost) eliminates the generation of silver triangles with relatively good performance.

<br/>**Build**
<br/>The code runs on both win and mac platform.
<br/>Note: If you are using Mac Intel chip, disable/comment out the<br/> 
    `set(CMAKE_OSX_ARCHITECTURES arm64)`
<br/>command in CMakeLists.txt as intel uses x86_64 architecture.

<br/>**Screenshot**
- polygon without inner hole
![image](https://github.com/gongwaner/UniformPolygonTriangulation/assets/29704759/e9e6fc26-3d07-4ea9-be65-c146f7de354e)

- polygon with inner holes
![image](https://github.com/gongwaner/UniformPolygonTriangulation/assets/29704759/c94848f9-2738-4563-bc24-87f0ca6c354f)

