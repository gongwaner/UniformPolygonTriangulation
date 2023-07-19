#include "OptimalPolygonTriangulation.h"
#include "../Utility/PolygonUtil.h"

#include <vtkTriangle.h>
#include <vtkVectorOperators.h>
#include <vtkPolygon.h>


namespace Algorithm
{
    void OptimalPolygonTriangulation::SetPolygonPoints(const std::vector<vtkVector3d>& polygonPoints)
    {
        mPolygonPoints = polygonPoints;

        //check if it's convex
        auto points = vtkSmartPointer<vtkPoints>::New();
        for (const auto& point: mPolygonPoints)
            points->InsertNextPoint(point.GetData());

        mIsConvex = vtkPolygon::IsConvex(points);
    }

    void OptimalPolygonTriangulation::SetNormal(const vtkVector3d& normal)
    {
        mNormal = normal;
    }

    void OptimalPolygonTriangulation::SetWeightFunction(const std::function<double(const vtkVector3d&, const vtkVector3d&, const vtkVector3d&)>& weightFunc)
    {
        mWeightFunc = weightFunc;
    }

    void OptimalPolygonTriangulation::AddNewTriangle(int vid0, int vid1, int vid2)
    {
        auto triangle = Utility::GetTriangle(mPolygonPoints, vid0, vid1, vid2, mNormal);
        mNewTriangles->InsertNextCell(triangle);
    }

    void OptimalPolygonTriangulation::AddNewTriangles(const std::vector<std::vector<Weights>>& table, int i, int j)
    {
        if (j <= i + 1)
            return;

        if (j == i + 2)
        {
            AddNewTriangle(i, i + 1, i + 2);
            return;
        }

        //std::cout << std::format("i={}, k={}, j={}", i, table[i][j].k, j) << std::endl;
        AddNewTriangle(i, table[i][j].k, j);

        AddNewTriangles(table, i, table[i][j].k);
        AddNewTriangles(table, table[i][j].k, j);
    }

    void OptimalPolygonTriangulation::ConvexTriangulation()
    {
        int n = mPolygonPoints.size();

        // table to store results of subproblems.  table[i][j] stores cost of triangulation of points from i to j.
        // The entry table[0][n-1] stores the final result.
        std::vector<std::vector<Weights>> table;
        for (int i = 0; i < n; ++i)
        {
            std::vector<Weights> entry(n);
            table.push_back(entry);
        }

        //dynamic programming
        for (int gap = 0; gap < n; gap++)
        {
            for (int i = 0, j = gap; j < n; i++, j++)
            {
                if (j < i + 2)
                    table[i][j].weight = 0;
                else
                {
                    table[i][j].weight = DBL_MAX;
                    for (int k = i + 1; k < j; k++)
                    {
                        double weight = table[i][k].weight + table[k][j].weight + mWeightFunc(mPolygonPoints[i], mPolygonPoints[j], mPolygonPoints[k]);

                        if (table[i][j].weight > weight)
                        {
                            table[i][j].weight = weight;
                            table[i][j].k = k;
                        }
                    }
                }
            }
        }

        mNewTriangles = vtkSmartPointer<vtkCellArray>::New();
        AddNewTriangles(table, 0, n - 1);
    }

    /**
     * check is 2 indices form a valid diagonal in polygon
     * for convex polygon the diagonal is valid as long as i,j has interval >= 2
     * for concave polygon the diagonal should be inside polygon
     */
    bool OptimalPolygonTriangulation::IsValidDiagonal(int i, int j, const double epsilon) const
    {
        if (abs(i - j) < 2)
            return false;

        if (i > j)
            std::swap(i, j);

        int n = mPolygonPoints.size();

        int begin1 = i + 1;
        int end1 = j - 1;
        int begin2 = (j + 1) % n;
        int end2 = (i > 0) ? i - 1 : n - 1;

        //check intersection between (i,j) and all polygon edges
        for (int k = begin1; k < end1; ++k)
        {
            if (Utility::LineIntersects(mPolygonPoints[i], mPolygonPoints[j], mPolygonPoints[k], mPolygonPoints[k + 1]))
                return false;
        }
        for (int k = begin2; k != end2; k = (k + 1) % n)
        {
            if (Utility::LineIntersects(mPolygonPoints[i], mPolygonPoints[j], mPolygonPoints[k], mPolygonPoints[(k + 1) % n]))
                return false;
        }

        //no intersections, check if line segment ij is completely inside or completely outside polygon
        vtkVector3d v1 = mPolygonPoints[i + 1] - mPolygonPoints[i];
        vtkVector3d v2 = i > 0 ? (mPolygonPoints[i - 1] - mPolygonPoints[i]) : (mPolygonPoints[n - 1] - mPolygonPoints[i]);
        vtkVector3d v3 = mPolygonPoints[j] - mPolygonPoints[i];

        double crossProducts[3];
        crossProducts[0] = v1.Cross(v2).Dot(mNormal);// V1xV2
        crossProducts[1] = v1.Cross(v3).Dot(mNormal); // V1xV3
        crossProducts[2] = v3.Cross(v2).Dot(mNormal); // V3xV2

        //angle between v1 and v2 (in counter-clockwise direction) is <= 180
        if ((crossProducts[0] > 0 || abs(crossProducts[0]) < epsilon) && (crossProducts[1] > 0 || abs(crossProducts[1]) < epsilon) &&
            (crossProducts[2] > 0 || abs(crossProducts[2]) < epsilon))
            return true;
            //angle between v1 and v2 (in counter-clockwise direction) is > 180
        else if ((crossProducts[0] < 0) &&
                 ((crossProducts[1] > 0 || abs(crossProducts[1]) < epsilon) || (crossProducts[2] > 0 || abs(crossProducts[2]) < epsilon)))
            return true;

        return false;
    }

    std::vector<std::pair<int, int>> OptimalPolygonTriangulation::GetDiagonals() const
    {
        std::vector<std::pair<int, int>> diagonalVec;

        int diagonals = mPolygonPoints.size() * (mPolygonPoints.size() - 3);
        diagonals = diagonals >> 1; // n(n-3)/2 for convex polygons

        int index = 0;
        int end;
        if (mIsConvex)
        {
            while (diagonals > 0)
            {
                end = (index > 0) ? mPolygonPoints.size() - 1 : mPolygonPoints.size() - 2;
                for (int i = index + 2; i <= end; ++i)
                {
                    diagonalVec.push_back(std::make_pair(index, i));
                    --diagonals;
                }
                ++index;
            }
        }
        else
        {
            while (diagonals > 0)
            {
                end = (index > 0) ? mPolygonPoints.size() - 1 : mPolygonPoints.size() - 2;
                for (int i = index + 2; i <= end; ++i)
                {
                    if (IsValidDiagonal(index, i))
                        diagonalVec.push_back(std::make_pair(index, i));
                    --diagonals;
                }
                ++index;
            }
        }

        return diagonalVec;
    }

    /**
     * a triangle consists of 1 edge and 2 diagonals or 2 edges and 1 diagonal or 3 diagonals
     * if a line segment is neither edge nor diagonal, it can't be a part of a triangle
     */
    bool OptimalPolygonTriangulation::IsTriangle(const std::vector<std::vector<bool>>& diagonalMatrix, int i, int j, int k) const
    {
        bool edge;
        int absValue;

        absValue = abs(i - j);
        edge = ((absValue == 1) || (absValue == (mPolygonPoints.size() - 1)));
        if (!(edge || diagonalMatrix[i][j]))
            return false;

        absValue = abs(i - k);
        edge = ((absValue == 1) || (absValue == (mPolygonPoints.size() - 1)));
        if (!(edge || diagonalMatrix[i][k]))
            return false;

        absValue = abs(j - k);
        edge = ((absValue == 1) || (absValue == (mPolygonPoints.size() - 1)));
        if (!(edge || diagonalMatrix[j][k]))
            return false;

        return true;
    }

    void OptimalPolygonTriangulation::ConcaveTriangulation()
    {
        int n = mPolygonPoints.size();

        //get all valid diagonals
        std::vector<std::pair<int, int>> validDiagonals = GetDiagonals();
        std::vector<std::vector<bool>> diagonalMatrix;
        for (int i = 0; i < n; ++i)
        {
            std::vector<bool> entry(n);
            std::fill(entry.begin(), entry.end(), false);
            diagonalMatrix.push_back(entry);
        }

        //initialize valid diagonals
        for (const auto& diagonal: validDiagonals)
        {
            diagonalMatrix[diagonal.first][diagonal.second] = true;
            diagonalMatrix[diagonal.second][diagonal.first] = true;
        }

        // table to store results of subproblems.  table[i][j] stores cost of triangulation of points from i to j.
        // The entry table[0][n-1] stores the final result.
        std::vector<std::vector<Weights>> table;
        for (int i = 0; i < n; ++i)
        {
            std::vector<Weights> entry(n);
            table.push_back(entry);
        }

        //dynamic programming
        for (int gap = 0; gap < n; gap++)
        {
            for (int i = 0, j = gap; j < n; i++, j++)
            {
                if (j < i + 2)
                    table[i][j].weight = 0;
                else if (j == i + 2)
                {
                    if (diagonalMatrix[i][j])
                        table[i][j].weight = mWeightFunc(mPolygonPoints[i], mPolygonPoints[i + 1], mPolygonPoints[i + 2]);
                    else
                        table[i][j].weight = DBL_MAX;
                }
                else
                {
                    table[i][j].weight = DBL_MAX;
                    if ((j < n - 1) && !diagonalMatrix[i][j]) //ij must be a diagonal, except 0 -> n - 1
                    {
                        continue;
                    }

                    for (int k = i + 1; k < j; k++)
                    {
                        if (!IsTriangle(diagonalMatrix, i, j, k))
                        {
                            continue;
                        }

                        double weight = table[i][k].weight + table[k][j].weight + mWeightFunc(mPolygonPoints[i], mPolygonPoints[j], mPolygonPoints[k]);

                        if (table[i][j].weight > weight)
                        {
                            table[i][j].weight = weight;
                            table[i][j].k = k;
                        }
                    }
                }
            }
        }

        mNewTriangles = vtkSmartPointer<vtkCellArray>::New();
        AddNewTriangles(table, 0, n - 1);
    }

    void OptimalPolygonTriangulation::GeneratePolyData()
    {
        mTriangulatedPolygon = vtkSmartPointer<vtkPolyData>::New();

        auto points = vtkSmartPointer<vtkPoints>::New();
        for (const auto& point: mPolygonPoints)
            points->InsertNextPoint(point.GetData());

        mTriangulatedPolygon->SetPoints(points);
        mTriangulatedPolygon->SetPolys(mNewTriangles);
    }

    void OptimalPolygonTriangulation::Triangulate()
    {
        mNewTriangles = vtkSmartPointer<vtkCellArray>::New();

        if (mPolygonPoints.size() < 3)
            return;

        if (mPolygonPoints.size() == 3)
        {
            AddNewTriangle(0, 1, 2);
            GeneratePolyData();
            return;
        }

        if (mIsConvex)
            ConvexTriangulation();
        else
            ConcaveTriangulation();

        GeneratePolyData();
    }

    vtkSmartPointer<vtkPolyData> OptimalPolygonTriangulation::GetTriangulatedPolygon() const
    {
        return mTriangulatedPolygon;
    }
}