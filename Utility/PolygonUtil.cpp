#include "PolygonUtil.h"

#include <vtkPoints.h>
#include <vtkPolygon.h>
#include <vtkVectorOperators.h>
#include <vtkLine.h>
#include <vtkTriangle.h>

#include <queue>


namespace Utility
{
    vtkSmartPointer<vtkPolyData> GetPolygonPolyData(const std::vector<vtkVector3d>& polygonPoints)
    {
        auto points = vtkSmartPointer<vtkPoints>::New();
        for (auto& point: polygonPoints)
        {
            points->InsertNextPoint(point.GetData());
        }

        //create boundary
        vtkNew<vtkPolygon> polygon;
        for (int i = 0; i < polygonPoints.size(); ++i)
            polygon->GetPointIds()->InsertNextId(i);

        auto cellArray = vtkSmartPointer<vtkCellArray>::New();
        cellArray->InsertNextCell(polygon);

        //create a polydata
        auto polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
        polygonPolyData->SetPoints(points);
        polygonPolyData->SetPolys(cellArray);

        return polygonPolyData;
    }

    bool LineIntersects(const vtkVector3d& line1Start, const vtkVector3d& line1End, const vtkVector3d& line2Start, const vtkVector3d& line2End)
    {
        double u, v;
        int result = vtkLine::Intersection(line1Start.GetData(), line1End.GetData(), line2Start.GetData(), line2End.GetData(), u, v);

        if (result == vtkLine::IntersectionType::NoIntersect)
            return false;

        //include both on line and intersect
        return true;
    }

    bool GetLineIntersection(const vtkVector3d& line1Start, const vtkVector3d& line1End, const vtkVector3d& line2Start, const vtkVector3d& line2End,
                             vtkVector3d& intersectionPoint, const double epsilon)
    {
        double u, v;
        int result = vtkLine::Intersection(line1Start.GetData(), line1End.GetData(), line2Start.GetData(), line2End.GetData(), u, v);

        if (result == vtkLine::IntersectionType::NoIntersect)
            return false;

        if (u > epsilon && u < 1 && v > epsilon && v < 1)
        {
            intersectionPoint = line1Start + u * (line1End - line1Start);
            return true;
        }

        return false;
    }

    /**
     * compute normal for any kind of polygon. as vtkPolygon::ComputeNormal only works for convex polygon
     * ref: https://gitlab.kitware.com/vtk/vtk/-/issues/11988
     */
    void ComputePolygonNormal(const std::vector<vtkVector3d>& polygonPoints, double normal[3])
    {
        normal[0] = 0;
        normal[1] = 0;
        normal[2] = 0;

        vtkVector3d pt0 = polygonPoints[0];
        vtkVector3d pt1;
        auto numOfPoints = polygonPoints.size();

        for (unsigned int i = 0; i < numOfPoints; i++)
        {
            pt1 = polygonPoints[(i + 1) % numOfPoints];

            normal[0] += (pt0[1] - pt1[1]) * (pt0[2] + pt1[2]);
            normal[1] += (pt0[2] - pt1[2]) * (pt0[0] + pt1[0]);
            normal[2] += (pt0[0] - pt1[0]) * (pt0[1] + pt1[1]);

            pt0[0] = pt1[0];
            pt0[1] = pt1[1];
            pt0[2] = pt1[2];
        }

        vtkMath::Normalize(normal);
    }

    bool EpsilonEqual(const vtkVector3d& p0, const vtkVector3d& p1, double epsilon)
    {
        return abs(p0[0] - p1[0]) < epsilon && abs(p0[1] - p1[1]) < epsilon && abs(p0[2] - p1[2]) < epsilon;
    }

    vtkSmartPointer<vtkTriangle> GetTriangle(const std::vector<vtkVector3d>& points, int vid0, int vid1, int vid2, const vtkVector3d& planeNormal)
    {
        double triNormal[3];
        vtkTriangle::ComputeNormal(points[vid0].GetData(), points[vid1].GetData(), points[vid2].GetData(), triNormal);

        auto triangle = vtkSmartPointer<vtkTriangle>::New();
        if (vtkVector3d(triNormal).Dot(planeNormal) < 0)
        {
            triangle->GetPointIds()->SetId(0, vid0);
            triangle->GetPointIds()->SetId(1, vid1);
            triangle->GetPointIds()->SetId(2, vid2);
        }
        else
        {
            triangle->GetPointIds()->SetId(0, vid2);
            triangle->GetPointIds()->SetId(1, vid1);
            triangle->GetPointIds()->SetId(2, vid0);
        }

        return triangle;
    }
}

