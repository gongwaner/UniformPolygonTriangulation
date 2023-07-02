#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkVector.h>

#include <vector>

namespace Utility
{
    vtkVector3d GetAverageCenter(const std::vector<vtkVector3d>& points);
    vtkSmartPointer<vtkPolyData> GetCombinedPolyData(const std::vector<vtkSmartPointer<vtkPolyData>>& meshes);
}