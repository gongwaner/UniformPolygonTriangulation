#include "TestUtil.h"
#include "ActorUtil.h"
#include "GeometricObjectUtil.h"

#include <vtkProperty.h>


namespace TestUtil
{
	void AddPolyData(vtkSmartPointer<vtkPolyData> polyData, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors, bool wireFrame)
	{
		auto polydataActor = Utility::GetPolyDataActor(polyData, color);
		if (wireFrame)
			polydataActor->GetProperty()->SetRepresentationToWireframe();
		actors.push_back(polydataActor);
	}

	void AddPoint(const vtkVector3d& point, double pointSize, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors)
	{
		auto pointActor = Utility::GetPointActor(point, pointSize, color);
		actors.push_back(pointActor);
	}

	void AddPoints(const std::vector<vtkVector3d>& points, double pointSize, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors)
	{
		auto boundaryPointsActor = Utility::GetPointsActor(points, pointSize, color);
		actors.push_back(boundaryPointsActor);
	}

	void AddVector(const vtkVector3d& start, const vtkVector3d& end, double lineWidth, const double* color, std::vector<vtkSmartPointer<vtkActor>>& actors)
	{
		auto vectorActor = Utility::GetPolyDataActor(Utility::GetLinePolyData(start, end), color);
		vectorActor->GetProperty()->SetLineWidth(lineWidth);
		actors.push_back(vectorActor);
	}
}