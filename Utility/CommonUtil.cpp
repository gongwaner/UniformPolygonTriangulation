#include "CommonUtil.h"


namespace Utility
{
    bool HasElements(const std::vector<std::vector<vtkVector3d>>& vector)
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
