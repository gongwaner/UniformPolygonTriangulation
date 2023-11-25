#include "CommonUtil.h"


namespace Utility
{
    bool EpsilonEqual(const vtkVector3d& p0, const vtkVector3d& p1, const double epsilon)
    {
        return abs(p0[0] - p1[0]) < epsilon && abs(p0[1] - p1[1]) < epsilon && abs(p0[2] - p1[2]) < epsilon;
    }

    bool EpsilonContains(const std::vector<vtkVector3d>& pointsVector, const vtkVector3d& point, const double epsilon)
    {
        for(const auto& p: pointsVector)
        {
            if(EpsilonEqual(p, point, epsilon))
                return true;
        }

        return false;
    }
}
