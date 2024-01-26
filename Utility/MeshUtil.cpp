#include "MeshUtil.h"

#include <vtkVectorOperators.h>
#include <vtkPointData.h>
#include <vtkAppendPolyData.h>


namespace Utility
{
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

    vtkSmartPointer<vtkPolyData> GetCombinedPolyData(const std::vector<vtkSmartPointer<vtkPolyData>>& meshes)
    {
        if(meshes.size() == 1)
            return meshes[0];

        auto appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
        for(const auto& mesh: meshes)
            appendFilter->AddInputData(mesh);
        appendFilter->Update();

        return appendFilter->GetOutput();
    }
}
