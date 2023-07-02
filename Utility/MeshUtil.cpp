#include "MeshUtil.h"

#include <vtkVectorOperators.h>
#include <vtkPointData.h>
#include <vtkAppendPolyData.h>

#include <stack>


namespace Utility
{
    vtkVector3d GetAverageCenter(const std::vector<vtkVector3d>& points)
    {
        vtkVector3d avgCenter(0, 0, 0);
        for (const auto& point: points)
        {
            avgCenter += point;
        }

        for (int i = 0; i < 3; ++i)
            avgCenter[i] /= (double) points.size();

        return avgCenter;
    }

    vtkSmartPointer<vtkPolyData> GetCombinedPolyData(const std::vector<vtkSmartPointer<vtkPolyData>>& meshes)
    {
        if (meshes.size() == 1)
            return meshes[0];

        auto appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
        for (const auto& mesh: meshes)
            appendFilter->AddInputData(mesh);
        appendFilter->Update();

        return appendFilter->GetOutput();
    }
}