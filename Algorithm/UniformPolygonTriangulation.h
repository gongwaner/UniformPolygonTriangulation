#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkVector.h>

#include <vector>


namespace Algorithm
{
    /**
     * uniform polygon triangulation algorithm as illustrated in
     * Zhang, Yongjie & Bajaj, Chandrajit & Sohn, Bong-soo. (2003). Adaptive Multiresolution and Quality 3D Meshing from Imaging Data.
     *
     * works by introducing new vertices.
     * Input:
     * ordered polygon points. The polygon should NOT self intersect.
     * If there are inner holes, pass inner holes in opposite orientation as outer contour.
     *
     * General algorithm:
     * 1.calculate the oriented bounding box of the polygon
     * 2.starting from bounding box's upper left corner, sweep the polygon in a left-to-right, upper-to-bottom order using specified square/pixel size
     * 3.for each pixel, calculate the intersected polygon of square and polygon
     * 4.triangulate intersected sub-polygon using optimal triangulation
     * 5.combine all triangulated sub-polygon into final triangulated polygon
     *
     * Note: weight function for optimal triangulation used here is aspect ratio of triangle, but this can be customized to apply to different scenarios
     */
    class UniformPolygonTriangulation
    {
    public:
        void SetPolygonPoints(const std::vector<vtkVector3d>& polygonPoints);
        void SetHoles(const std::vector<std::vector<vtkVector3d>>& holes);
        void SetNormal(const vtkVector3d& planeNormal);
        void SetSquareSize(double size);
        void Triangulate();

        vtkSmartPointer<vtkPolyData> GetOutPut() const;
        vtkVector3d GetAxisX() const;

        bool mDebug = false;


    private:
        void InitializePlane();
        void InitializePolygon();
        void InitializeHoles();
        vtkSmartPointer<vtkPolyData> GetTriangulatedPolygon() const;

        bool mHasHoles = false;
        std::vector<vtkVector3d> mPolygonPoints;
        std::vector<std::vector<vtkVector3d>> mInnerHoles;

        bool mUpdateCalculation = false;

        vtkVector3d mPlaneCenter = vtkVector3d(0, 0, 0);
        vtkVector3d mAxisX = vtkVector3d(0, 1, 0);
        vtkVector3d mAxisY = vtkVector3d(1, 0, 0);
        vtkVector3d mNormal = vtkVector3d(0, 0, 1);;

        //inside/outside query
        std::vector<double> mPolygonPointsData2d;
        double mPolygonBounds[6];

        std::vector<std::vector<double>> mHolePointsData2dVector;
        std::vector<std::vector<double>> mHolesBounds;

        vtkVector3d mUpperLeftCorner = vtkVector3d(0, 0, 0);
        double mBoundingBoxWidth = 0.0;
        double mBoundingBoxHeight = 0.0;
        double mLength = 0.0;

        vtkSmartPointer<vtkPolyData> mTriangulatedPolygon = nullptr;
    };
}
