#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkVector.h>

#include <vector>

namespace Utility
{
	vtkSmartPointer<vtkPolyData> ReadPolyData(const std::string& fileName);
	void WritePolyData(const std::string& fileName, vtkSmartPointer<vtkPolyData> polyData);
	std::vector<vtkVector3d> ReadFromFile(const char* dir);
	void WriteToFile(const std::vector<vtkVector3d>& vector, const char* dir);
}