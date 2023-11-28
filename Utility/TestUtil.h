#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkVector.h>
#include <vector>


namespace TestUtil
{
    void AddPolyData(const vtkSmartPointer<vtkPolyData>& polyData, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors, bool wireFrame = false);
    void AddPoint(const vtkVector3d& point, float pointSize, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors);
    void AddPoints(const std::vector<vtkVector3d>& points, float pointSize, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors);
    void AddVector(const vtkVector3d& start, const vtkVector3d& end, float lineWidth, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors);
}
