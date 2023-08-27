#pragma once

#include <vtkVector.h>
#include <vector>


namespace Utility
{
    bool EpsilonEqual(const vtkVector3d& p0, const vtkVector3d& p1, const double epsilon = 1e-6);
    bool HasElements(const std::vector<std::vector<vtkVector3d>>& vector);
    bool EpsilonContains(const std::vector<vtkVector3d>& pointsVector, const vtkVector3d& point, const double epsilon = 1e-6);
};
