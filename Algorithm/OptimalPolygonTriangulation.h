#pragma once

#include <vtkVector.h>
#include <vtkPolyData.h>
#include <functional>


namespace Algorithm
{
    struct Weights
    {
        double weight;
        int k;

        Weights()
        {
            weight = 0.0;
            k = 0;
        }

        Weights(double val, int k)
        {
            this->weight = val;
            this->k = k;
        }
    };

    /**
     * optimal(aka.minimum weight) triangulation using dynamic programming
     * given customized weight function
     */
    class OptimalPolygonTriangulation
    {
    public:
        OptimalPolygonTriangulation();
        ~OptimalPolygonTriangulation() = default;
        void SetPolygonPoints(const std::vector<vtkVector3d>& polygonPoints);
        void SetNormal(const vtkVector3d& normal);
        void Triangulate(const std::function<double(const vtkVector3d&, const vtkVector3d&, const vtkVector3d&)>& weightFunc);
        vtkSmartPointer<vtkPolyData> GetTriangulatedPolygon() const;

    private:
        void AddNewTriangle(int vid0, int vid1, int vid2);
        void AddNewTriangles(const std::vector<std::vector<Weights>>& table, int i, int j);
        void GeneratePolyData();
        void ConvexTriangulation(const std::function<double(const vtkVector3d&, const vtkVector3d&, const vtkVector3d&)>& weightFunc);
        void ConcaveTriangulation(std::function<double(const vtkVector3d&, const vtkVector3d&, const vtkVector3d&)> weightFunc);
        bool IsValidDiagonal(int i, int j) const;
        std::vector<std::pair<int, int>>  GetDiagonals() const;
        bool IsTriangle(const std::vector<std::vector<bool>>& diagonalMatrix, int i, int j, int k) const;

        std::vector<vtkVector3d> mPolygonPoints;
        vtkVector3d mNormal;
        bool mIsConvex;
        vtkSmartPointer<vtkCellArray> mNewTriangles = nullptr;
        vtkSmartPointer<vtkPolyData> mTriangulatedPolygon = nullptr;
    };
}

