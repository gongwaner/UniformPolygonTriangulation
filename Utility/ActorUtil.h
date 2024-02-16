#pragma once

#include <vtkSmartPointer.h>
#include <vtkVector.h>


class vtkActor;
class vtkPolyDataMapper;
class vtkPolyData;

namespace Utility
{
    vtkSmartPointer<vtkActor> GetPolyDataActor(vtkPolyDataMapper* mapper, vtkPolyData* polyData, const double* diffuseColor, double alpha = 1.0);
    vtkSmartPointer<vtkActor> GetPolyDataActor(vtkPolyData* polyData, const double* diffuseColor, double alpha = 1.0);
    vtkSmartPointer<vtkActor> GetPointActor(const vtkVector3d& point, float pointSize, const double* pointColor);
    vtkSmartPointer<vtkActor> GetPointsActor(const std::vector<vtkVector3d>& pointsVector, float pointSize, const double* pointColor);
}
