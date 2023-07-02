#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkVector.h>
#include <vector>

namespace Utility
{
	vtkSmartPointer<vtkPolyData> GetPolygonPolyData(const std::vector<vtkVector3d>& polygonPoints);
	bool LineIntersects(const vtkVector3d& line1Start, const vtkVector3d& line1End, const vtkVector3d& line2Start, const vtkVector3d& line2End);
	bool GetLineIntersection(const vtkVector3d& line1Start, const vtkVector3d& line1End, const vtkVector3d& line2Start, const vtkVector3d& line2End, vtkVector3d& intersectionPoint);
	void ComputePolygonNormal(const std::vector<vtkVector3d>& polygonPoints, double normal[3]);
	bool EpsilonEqual(const vtkVector3d& p0, const vtkVector3d& p1, double epsilon = 1e-6);
}