#include "TestUtil.h"

#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkActor.h>

#include "GeometricObjectUtil.h"
#include "ActorUtil.h"


namespace TestUtil
{
    void AddPolyData(vtkPolyData* polyData, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors, const bool wireFrame)
    {
        auto polydataActor = Utility::GetPolyDataActor(polyData, color);
        if(wireFrame)
            polydataActor->GetProperty()->SetRepresentationToWireframe();
        actors.push_back(polydataActor);
    }

    void AddPoint(const vtkVector3d& point, const float pointSize, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors)
    {
        auto pointActor = Utility::GetPointActor(point, pointSize, color);
        actors.push_back(pointActor);
    }

    void AddPoints(const std::vector<vtkVector3d>& points, const float pointSize, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors)
    {
        auto pointsActor = Utility::GetPointsActor(points, pointSize, color);
        actors.push_back(pointsActor);
    }

    void AddVector(const vtkVector3d& start, const vtkVector3d& end, const float lineWidth, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors)
    {
        auto vectorActor = Utility::GetPolyDataActor(GeometricObjectUtil::GetLinePolyData(start, end), color);
        vectorActor->GetProperty()->SetLineWidth(lineWidth);
        actors.push_back(vectorActor);
    }
}
