#pragma once

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkVector.h>

#include <vector>


namespace Algorithm
{
	class UniformPolygonTriangulation
	{
	public:
		UniformPolygonTriangulation() = default;
		~UniformPolygonTriangulation() = default;

		void SetPolygonPoints(const std::vector<vtkVector3d>& polygonPoints);
		void SetHoles(const std::vector<std::vector<vtkVector3d>>& holes);
		void SetNormal(const vtkVector3d& planeNormal);
		void SetSquareSize(const double size);
		void Triangulate();
		vtkSmartPointer<vtkPolyData> GetTriangulatedPolygon() const;

		//debug 
		bool mDebug = false;
		vtkVector3d GetPlaneCenter() const;
		vtkVector3d GetAxisX() const;
		vtkVector3d GetAxisY() const;
		vtkVector3d GetUpperLeftCorner() const;
		double GetBoundingBoxWidth() const;
		double GetBoundingBoxHeight() const;


	private:
		void InitializePlane();
		void InitializePolygon();
		void InitializeHoles();
		bool IsInPolygon(const vtkVector3d& point);
		bool IsInHole(const vtkVector3d& point, int holeID);
		bool AllPointsInPolygon(const std::vector<vtkVector3d>& points);
		bool AllPointsInHole(const std::vector<vtkVector3d>& points);
		std::vector<std::vector<vtkVector3d>> GetSquarePolygonIntersectionPolygon(const std::vector<vtkVector3d>& squarePoints);

		std::vector<vtkVector3d> mPolygonPoints;
		std::vector<std::vector<vtkVector3d>> mInnerHoles;
		double mLength = 0;

		vtkVector3d mPlaneCenter;
		vtkVector3d mNormal;
		vtkVector3d mAxisX;
		vtkVector3d mAxisY;

		//inside/outside query
		std::vector<double> mPolygonPointsData2d;
		double mPolygonBounds[6];

		std::vector<std::vector<double>> mHolePointsData2dVector;
		std::vector<std::vector<double>> mHolesBounds;

		vtkVector3d mUpperLeftCorner;
		double mBoundingBoxWidth, mBoundingBoxHeight;

		std::vector<vtkSmartPointer<vtkPolyData>> mSubTriangulationVector;
	};
}


