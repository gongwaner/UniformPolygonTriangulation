#pragma once

#include <vtkVector.h>
#include <vector>


namespace Utility
{
    template<typename T>
    bool HasElements(const std::vector<std::vector<T>>& vector)
    {
        if(vector.empty())
            return false;

        for(const auto& element: vector)
        {
            if(!element.empty())
                return true;
        }

        return false;
    }
}
