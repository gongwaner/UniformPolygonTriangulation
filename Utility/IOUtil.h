#pragma once

#include <vtkVector.h>

#include <vector>


class vtkPolyData;

namespace Utility
{
    vtkSmartPointer<vtkPolyData> ReadPolyData(const char* fileDir);
    void WritePolyData(const char* fileDir, vtkPolyData* polyData);
    void WriteColorPolyData(const char* fileDir, vtkPolyData* polyData);
    std::vector<vtkVector3d> ReadVectorFromFile(const char* fileDir);
    void WriteVectorToFile(const std::vector<vtkVector3d>& vector, const char* fileDir);
}
