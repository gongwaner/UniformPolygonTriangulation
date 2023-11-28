#include "GeometricObjectUtil.h"

#include <vtkSphereSource.h>
#include <vtkPlaneSource.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkVectorOperators.h>
#include <vtkLineSource.h>


namespace Utility
{
    vtkSmartPointer<vtkPolyData> GetPointsPolyData(vtkPoints* points)
    {
        auto pointsPolyData = vtkSmartPointer<vtkPolyData>::New();
        pointsPolyData->SetPoints(points);

        auto vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vertexFilter->SetInputData(pointsPolyData);
        vertexFilter->Update();

        return vertexFilter->GetOutput();
    }

    void GetPlaneAxes(double center[3], double normal[3], double axisX[3], double axisY[3])
    {
        auto planeSource = vtkSmartPointer<vtkPlaneSource>::New();
        planeSource->SetCenter(center);
        planeSource->SetNormal(normal);
        planeSource->Update();

        planeSource->GetAxis1(axisX);
        planeSource->GetAxis2(axisY);
    }

    vtkSmartPointer<vtkPolyData> GetLinePolyData(const vtkVector3d& start, const vtkVector3d& end)
    {
        auto lineSource = vtkSmartPointer<vtkLineSource>::New();
        lineSource->SetPoint1(start.GetData());
        lineSource->SetPoint2(end.GetData());
        lineSource->Update();

        return lineSource->GetOutput();
    }

    vtkSmartPointer<vtkPolyData> GetLinePolyData(const vtkVector3d& start, const vtkVector3d& dir, const double length)
    {
        auto end = start + dir * length;
        return GetLinePolyData(start, end);
    }
}
