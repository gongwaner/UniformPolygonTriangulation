#pragma once

#include <vtkSmartPointer.h>
#include <vtkVector.h>

#include <vector>


class vtkPolyData;

namespace Utility
{
    vtkVector3d GetAverage(const std::vector<vtkVector3d>& data);
    vtkSmartPointer<vtkPolyData> GetCombinedPolyData(const std::vector<vtkSmartPointer<vtkPolyData>>& meshes);
}
