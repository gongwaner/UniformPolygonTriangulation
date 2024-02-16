#pragma once

#include <vtkVector.h>
#include <vector>


class vtkPolyData;
class vtkActor;

namespace TestUtil
{
    void AddPolyData(vtkPolyData* polyData, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors, bool wireFrame = false);
    void AddPoint(const vtkVector3d& point, float pointSize, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors);
    void AddPoints(const std::vector<vtkVector3d>& points, float pointSize, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors);
    void AddVector(const vtkVector3d& start, const vtkVector3d& end, float lineWidth, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors);
}
