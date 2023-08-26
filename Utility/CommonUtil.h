#pragma once

#include <vtkVector.h>
#include <vector>


namespace Utility
{
    bool HasElements(const std::vector<std::vector<vtkVector3d>>& vector);
};
