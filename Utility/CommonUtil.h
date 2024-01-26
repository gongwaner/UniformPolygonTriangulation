#pragma once

#include <vtkVector.h>
#include <vector>


namespace Utility
{
    template<typename T, typename F>
    std::vector<size_t> SortIndices(const std::vector<T>& v, F comparator)
    {
        // initialize original index locations
        std::vector<size_t> idx(v.size());
        for(size_t i = 0; i < v.size(); ++i)
            idx[i] = i;

        std::sort(idx.begin(), idx.end(), comparator);

        return idx;
    }

    template<typename T> bool HasElements(const std::vector<std::vector<T>>& vector)
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

    bool EpsilonEqual(const vtkVector3d& p0, const vtkVector3d& p1, double epsilon = 1e-6);
    bool EpsilonContains(const std::vector<vtkVector3d>& pointsVector, const vtkVector3d& point, double epsilon = 1e-6);
}
