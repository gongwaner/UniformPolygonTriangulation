#include "ActorUtil.h"

#include "GeometricObjectUtil.h"

#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkProperty.h>


namespace Utility
{
    vtkSmartPointer<vtkActor> GetPolyDataActor(vtkPolyDataMapper* mapper, vtkPolyData* polyData, const double* diffuseColor,
                                               const double alpha)
    {
        mapper->SetInputData(polyData);

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetDiffuseColor(diffuseColor);
        actor->GetProperty()->SetSpecular(0.6);
        actor->GetProperty()->SetSpecularPower(20);
        actor->GetProperty()->SetOpacity(alpha);

        return actor;
    }

    vtkSmartPointer<vtkActor> GetPolyDataActor(vtkPolyData* polyData, const double* diffuseColor, const double alpha)
    {
        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        return GetPolyDataActor(mapper, polyData, diffuseColor, alpha);
    }

    vtkSmartPointer<vtkActor> GetPointActor(const vtkVector3d& point, const float pointSize, const double* pointColor)
    {
        auto points = vtkSmartPointer<vtkPoints>::New();
        points->InsertNextPoint(point[0], point[1], point[2]);
        auto pointPolyData = GetPointsPolyData(points);

        auto pointActor = GetPolyDataActor(pointPolyData, pointColor);
        pointActor->GetProperty()->SetPointSize(pointSize);

        return pointActor;
    }

    vtkSmartPointer<vtkActor> GetPointsActor(const std::vector<vtkVector3d>& pointsVector, const float pointSize, const double* pointColor)
    {
        auto points = vtkSmartPointer<vtkPoints>::New();
        for(const auto& point: pointsVector)
            points->InsertNextPoint(point[0], point[1], point[2]);
        auto pointPolyData = GetPointsPolyData(points);

        auto pointActor = GetPolyDataActor(pointPolyData, pointColor);
        pointActor->GetProperty()->SetPointSize(pointSize);

        return pointActor;
    }
}
