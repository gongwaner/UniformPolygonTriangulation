#pragma once

#include <vtkVector.h>
#include <unordered_map>
#include <unordered_set>
#include <set>

namespace Algorithm
{
    /**
     * Calculate polygon intersection using weiler-atherton polygon clipping algorithm
     * ref: https://www.geeksforgeeks.org/weiler-atherton-polygon-clipping-algorithm/
     */
	class SquarePolygonIntersection
	{
	public:
		SquarePolygonIntersection() = default;
		~SquarePolygonIntersection() = default;

		void SetPolygonPoints(const std::vector<vtkVector3d>& polygonPoints);
		void SetHoles(const std::vector<std::vector<vtkVector3d>>& holes);
		void SetSquarePoints(const std::vector<vtkVector3d>& squarePoints);
		void SetPlane(const vtkVector3d& planeCenter, const vtkVector3d& planeNormal, const vtkVector3d& axisX, const vtkVector3d& axisY);
		void CalculateIntersectionPolygons();
		std::vector<std::vector<vtkVector3d>> GetIntersectedPolygon() const;

		bool mDebug = false;

	private:
		void InitializePolygon();
		void InitializeHoles();
		void InitializeSquare();
		bool PointInPolygon(const vtkVector3d& point);
		bool PointInSquare(const vtkVector3d& point, const double epsilon = 1e-6);
		bool LineIntersects(int squareLineID, const std::pair<int, int>& polyLine, vtkVector3d& intersectionPoint, const double epsilon);
		bool HasIntersection();
		void SetUpPolygonIntersection(const std::unordered_map<int, std::vector<vtkVector3d>>& polygonMap);
		void SetUpSquareIntersection(const std::unordered_map<int, std::vector<vtkVector3d>>& squareMap);
		int GetNextPolygonIndex(int index);
		int GetNextIntersectionPolygonIndex(int index);

		std::vector<vtkVector3d> mPolygonPoints;
		std::vector<std::vector<vtkVector3d>> mInnerHoles;
		std::vector<std::pair<int, int>> mPolyLines;
		std::vector<vtkVector3d> mSquarePoints;

		//plane
		vtkVector3d mPlaneCenter;
		vtkVector3d mPlaneNormal;
		vtkVector3d mAxisX;
		vtkVector3d mAxisY;

		//bounds calculation
		std::vector<double> mPolygonPointsData2d;
		double mPolygonBounds[6];

		std::vector<double> mSquarePointsData2d;
		double mSquareBounds[4];

		//middle results for intersection calculation
		std::vector<vtkVector3d> mPolygonVertices;//polygon vertices with intersection points
		std::vector<vtkVector3d> mSquareVertices;//square vertices with intersection points
		std::set<vtkVector3d> mIntersectionPointsSet;//all intersection points
		std::vector<int> mEnterIndices;
		std::vector<bool> mIsExitPoint;
		std::unordered_set<int> mComponentEndIndices;
		std::vector<std::pair<int, int>> mComponentIntervals;
		std::vector<std::pair<int, int>> mIntersectionIntervals;
		std::vector<std::vector<vtkVector3d>> mSubPolygons;
	};
}


