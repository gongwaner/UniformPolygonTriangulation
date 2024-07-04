#pragma once

#include <vtkVector.h>

#include <vector>


class vtkPolyData;

namespace Utility
{
    std::vector<vtkVector3d> ReadVectorFromFile(const char* fileDir);
    void WriteVectorToFile(const std::vector<vtkVector3d>& vector, const char* fileDir);
}
