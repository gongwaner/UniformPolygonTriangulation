#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkVector.h>

#include <vector>


namespace Utility
{
    vtkSmartPointer <vtkPolyData> ReadPolyData(const char* fileDir);
    void WritePolyData(const char* fileDir, vtkSmartPointer <vtkPolyData> polyData);
    std::vector <vtkVector3d> ReadVectorFromFile(const char* dir);
    void WriteVectorToFile(const std::vector <vtkVector3d>& vector, const char* dir);
}
