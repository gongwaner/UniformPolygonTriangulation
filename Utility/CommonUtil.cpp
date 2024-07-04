#include "CommonUtil.h"

#include <vtkVectorOperators.h>


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

    vtkVector3d GetAverage(const std::vector<vtkVector3d>& data)
    {
        vtkVector3d avg(0, 0, 0);
        for(const auto& value: data)
        {
            avg += value;
        }

        for(int i = 0; i < 3; ++i)
            avg[i] /= (double) data.size();

        return avg;
    }
}
