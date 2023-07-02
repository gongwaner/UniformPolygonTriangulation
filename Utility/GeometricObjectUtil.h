#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkVector.h>

namespace Utility
{
	vtkSmartPointer<vtkPolyData> GetPointsPolyData(vtkSmartPointer<vtkPoints> points);
	void GetPlaneAxes(double center[3], double normal[3], double axisX[3], double axisY[3]);
	vtkSmartPointer<vtkPolyData> GetLinePolyData(const vtkVector3d& start, const vtkVector3d& end);
	vtkSmartPointer<vtkPolyData> GetLinePolyData(const vtkVector3d& start, const vtkVector3d& dir, double length);
}