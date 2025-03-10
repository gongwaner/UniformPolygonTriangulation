#include "PolygonUtil.h"

#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPolygon.h>
#include <vtkVectorOperators.h>
#include <vtkLine.h>

#include <queue>


namespace Utility
{
    LineIntersectionType GetLineIntersectionType(const vtkVector3d& line1Start, const vtkVector3d& line1End,
                                                 const vtkVector3d& line2Start, const vtkVector3d& line2End,
                                                 vtkVector3d& intersectionPoint, const double epsilon)
    {
        double u, v;
        vtkLine::Intersection(line1Start.GetData(), line1End.GetData(), line2Start.GetData(), line2End.GetData(), u, v);
        intersectionPoint = line1Start + u * (line1End - line1Start);

        if((abs(u) <= epsilon || abs(u - 1.0) <= epsilon) && (abs(v) <= epsilon || abs(v - 1.0) <= epsilon))
            return LineIntersectionType::CommonEndPoint;

        if(((abs(u) <= epsilon || abs(u - 1.0) <= epsilon) && (v > epsilon && v < 1.0 - epsilon)) ||
           ((abs(v) <= epsilon || abs(v - 1.0) <= epsilon) && (u > epsilon && u < 1.0 - epsilon)))
            return LineIntersectionType::EndPoint;

        if(u > epsilon && u < 1 - epsilon && v > epsilon && v < 1 - epsilon)
            return LineIntersectionType::Intersection;

        return LineIntersectionType::NoIntersection;
    }

    bool PointInPolygon(const int numOfPoints, const double* polygonPoints2d, const double polygonBounds[6],
                        const vtkVector3d& planeCenter, const vtkVector3d& axisX, const vtkVector3d& axisY,
                        const vtkVector3d& point)
    {
        //projection along local axes
        auto cp = point - planeCenter;
        auto xProj = cp.Dot(axisX);
        auto yProj = cp.Dot(axisY);

        double pointToQuery[3]{xProj, yProj, 0};
        double normal2d[3]{0, 0, 1};
        //WARNING: explicitly drop const correctness here to fit vtk function call parameter list
        return vtkPolygon::PointInPolygon(pointToQuery, numOfPoints, const_cast<double*>(polygonPoints2d),
                                          const_cast<double*>(polygonBounds), normal2d);
    }
}

