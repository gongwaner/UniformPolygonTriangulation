#pragma once

#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkVector.h>
#include <vtkPolyDataMapper.h>
#include <vtkLookupTable.h>


namespace Utility
{
    vtkSmartPointer<vtkActor> GetPolyDataActor(vtkSmartPointer<vtkPolyDataMapper> mapper, vtkSmartPointer<vtkPolyData> polyData, const double* diffuseColor, const double alpha = 1.0);
    vtkSmartPointer<vtkActor> GetPolyDataActor(vtkSmartPointer<vtkPolyData> polyData, const double* diffuseColor, const double alpha = 1.0);
    vtkSmartPointer<vtkActor> GetPointActor(const vtkVector3d& point, const float pointSize, const double* pointColor);
    vtkSmartPointer<vtkActor> GetPointsActor(const std::vector<vtkVector3d>& pointsVector, const float pointSize, const double* pointColor);
}
