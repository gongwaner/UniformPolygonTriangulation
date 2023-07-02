#include "ActorUtil.h"
#include "GeometricObjectUtil.h"

#include <vtkPoints.h>
#include <vtkProperty.h>


namespace Utility
{
	vtkSmartPointer<vtkActor> GetPolyDataActor(vtkSmartPointer<vtkPolyDataMapper> mapper, vtkSmartPointer<vtkPolyData> polyData, const double* diffuseColor, double alpha)
	{
		mapper->SetInputData(polyData);

		auto actor = vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(mapper);
		actor->GetProperty()->SetDiffuseColor(diffuseColor);
		actor->GetProperty()->SetSpecular(0.6);
		actor->GetProperty()->SetSpecularPower(20);
		actor->GetProperty()->SetOpacity(alpha);

		return actor;
	}

	vtkSmartPointer<vtkActor> GetPolyDataActor(vtkSmartPointer<vtkPolyData> polyData, const double* diffuseColor, double alpha)
	{
		auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		return GetPolyDataActor(mapper, polyData, diffuseColor,alpha);
	}

	vtkSmartPointer<vtkActor> GetPointActor(vtkVector3d point, float pointSize, const double* pointColor)
	{
		vtkNew<vtkPoints> points;
		points->InsertNextPoint(point[0], point[1], point[2]);
		auto pointPolyData = GetPointsPolyData(points);

		auto pointActor = GetPolyDataActor(pointPolyData, pointColor);
		pointActor->GetProperty()->SetPointSize(pointSize);

		return pointActor;
	}


	vtkSmartPointer<vtkActor> GetPointsActor(const std::vector<vtkVector3d>& pointsVector, float pointSize, const double* pointColor)
	{
		vtkNew<vtkPoints> points;
		for (const auto& point : pointsVector)
			points->InsertNextPoint(point[0], point[1], point[2]);
		auto pointPolyData = GetPointsPolyData(points);

		auto pointActor = GetPolyDataActor(pointPolyData, pointColor);
		pointActor->GetProperty()->SetPointSize(pointSize);

		return pointActor;
	}
}
