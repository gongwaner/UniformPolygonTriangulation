#pragma once

#include <vtkVector.h>
#include <unordered_map>
#include <unordered_set>
#include <set>


namespace Algorithm
{
    /**
     * Calculate polygon intersection using Weiler-Atherton polygon clipping algorithm
     * ref: https://www.geeksforgeeks.org/weiler-atherton-polygon-clipping-algorithm/
     */
    class SquarePolygonIntersection
    {
    public:
        void SetPolygonPoints(const std::vector<vtkVector3d>& polygonPoints);
        void SetHoles(const std::vector<std::vector<vtkVector3d>>& holes);
        void SetSquarePoints(const std::vector<vtkVector3d>& squarePoints);
        void SetPlane(const vtkVector3d& planeCenter, const vtkVector3d& axisX, const vtkVector3d& axisY);
        void SetPrecision(double epsilon);

        void CalculateIntersectedPolygons();

        std::vector<std::vector<vtkVector3d>> GetIntersectedPolygon() const;

        bool mDebug = false;

    private:
        void InitializePolygon();
        void InitializeHoles();
        void InitializeSquare();

        bool PointInSquare(const vtkVector3d& point) const;
        bool LineIntersects(int squareLineID, const std::pair<int, int>& polyLine, vtkVector3d& intersectionPoint) const;
        bool HasIntersection();

        void SetUpPolygonIntersection(const std::unordered_map<int, std::vector<vtkVector3d>>& polygonMap);
        void SetUpSquareIntersection(const std::unordered_map<int, std::vector<vtkVector3d>>& squareMap);

        int GetNextPolygonIndex(int index) const;
        int GetNextIntersectionPolygonIndex(int index) const;

        std::vector<vtkVector3d> mOuterContourPoints;
        std::vector<vtkVector3d> mContourPoints;
        std::vector<double> mOuterPolygonPointsData2d;
        std::vector<double> mContourPointsData2d;
        std::vector<std::pair<int, int>> mOuterPolyLines;
        std::vector<std::pair<int, int>> mContourPolyLines;
        std::vector<std::vector<vtkVector3d>> mInnerHoles;
        std::vector<vtkVector3d> mSquarePoints;

        //plane
        vtkVector3d mPlaneCenter = vtkVector3d(0, 0, 0);
        vtkVector3d mAxisX = vtkVector3d(1, 0, 0);
        vtkVector3d mAxisY = vtkVector3d(0, 1, 0);

        //square bounds calculation
        std::vector<double> mSquarePointsData2d;
        double mSquareBounds[4]{0.0, 0.0, 0.0, 0.0};

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

        double mEpsilon = 1e-6;

        bool mUpdatePolygon = false;
        bool mUpdateSquare = false;
        bool mUpdateHoles = false;
    };
}
