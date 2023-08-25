#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkVector.h>
#include <vector>


namespace Utility
{
    enum LineIntersectionType
    {
        NoIntersection,
        CommonEndPoint,//two lines intersects in common end point
        EndPoint,//two lines intersects in one of the end point of a line
        Intersection,
    };

    vtkSmartPointer<vtkPolyData> GetPolygonPolyData(const std::vector<vtkVector3d>& polygonPoints);
    bool LineIntersects(const vtkVector3d& line1Start, const vtkVector3d& line1End, const vtkVector3d& line2Start, const vtkVector3d& line2End);
    LineIntersectionType GetLineIntersectionType(const vtkVector3d& line1Start, const vtkVector3d& line1End, const vtkVector3d& line2Start, const vtkVector3d& line2End, const double epsilon = 1e-6);
    bool GetLineIntersection(const vtkVector3d& line1Start, const vtkVector3d& line1End, const vtkVector3d& line2Start, const vtkVector3d& line2End, vtkVector3d& intersectionPoint, const double epsilon = 1e-6);
    void ComputePolygonNormal(const std::vector<vtkVector3d>& polygonPoints, double normal[3]);
    bool EpsilonEqual(const vtkVector3d& p0, const vtkVector3d& p1, const double epsilon = 1e-6);
    vtkSmartPointer<vtkTriangle> GetTriangle(const std::vector<vtkVector3d>& points, int vid0, int vid1, int vid2, const vtkVector3d& planeNormal);
    bool PointInPolygon(const int numOfPoints, const double* polygonPoints2d, const vtkVector3d& planeCenter, const vtkVector3d& axisX, const vtkVector3d& axisY, double polygonBounds[6], const vtkVector3d& point);
}
