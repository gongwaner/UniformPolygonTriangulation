#pragma once

#include <vtkVector.h>
#include <vector>


class vtkPolyData;

namespace Utility
{
    enum class LineIntersectionType
    {
        NoIntersection,
        CommonEndPoint,//two lines intersects in common end point
        EndPoint,//two lines intersects in one of the end point of a line
        Intersection,
    };

    LineIntersectionType GetLineIntersectionType(const vtkVector3d& line1Start, const vtkVector3d& line1End,
                                                 const vtkVector3d& line2Start, const vtkVector3d& line2End,
                                                 vtkVector3d& intersectionPoint, double epsilon = 1e-6);

    /**
     * Check if a point if within polygon by projecting it to 2d plane then perform check.
     * Directly call vtkPolygon::PointInPolygon() will fail for cases where point is epsilon larger in z than plane
     * note: This function returns TRUE if query point is polygon point
     *
     * @param numOfPoints number of polygon points
     * @param polygonPoints2d polygon points in(xProjection, yProjection, 0) form
     * @param polygonBounds polygon bounds returned by polygonBoundingBox.GetBounds()
     * @param planeCenter polygon plane center
     * @param axisX local X axis
     * @param axisY local Y axis
     * @param point point to query
     */
    bool PointInPolygon(int numOfPoints, const double* polygonPoints2d, const double polygonBounds[6],
                        const vtkVector3d& planeCenter, const vtkVector3d& axisX, const vtkVector3d& axisY,
                        const vtkVector3d& point);
}
