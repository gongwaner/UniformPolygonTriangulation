#include "UniformPolygonTriangulation.h"
#include "SquarePolygonIntersection.h"
#include "OptimalPolygonTriangulation.h"
#include "../Utility/GeometricObjectUtil.h"
#include "../Utility/MeshUtil.h"
#include "../Utility/PolygonUtil.h"

#include <vtkPolygon.h>
#include <vtkVectorOperators.h>
#include <vtkTriangle.h>


namespace Algorithm
{
    vtkSmartPointer<vtkTriangle> GetTriangle(const std::vector<vtkVector3d>& points, int vid0, int vid1, int vid2, const vtkVector3d& planeNormal)
    {
        double triNormal[3];
        vtkTriangle::ComputeNormal(points[vid0].GetData(), points[vid1].GetData(), points[vid2].GetData(), triNormal);

        auto triangle = vtkSmartPointer<vtkTriangle>::New();
        if (vtkVector3d(triNormal).Dot(planeNormal) < 0)
        {
            triangle->GetPointIds()->SetId(0, vid0);
            triangle->GetPointIds()->SetId(1, vid1);
            triangle->GetPointIds()->SetId(2, vid2);
        }
        else
        {
            triangle->GetPointIds()->SetId(0, vid2);
            triangle->GetPointIds()->SetId(1, vid1);
            triangle->GetPointIds()->SetId(2, vid0);
        }

        return triangle;
    }

    vtkSmartPointer<vtkPolyData> MakeSquare(const std::vector<vtkVector3d>& squarePoints, const vtkVector3d& planeNormal)
    {
        auto squarePolyData = vtkSmartPointer<vtkPolyData>::New();

        auto points = vtkSmartPointer<vtkPoints>::New();
        for (const auto& point: squarePoints)
            points->InsertNextPoint(point.GetData());

        auto newTris = vtkSmartPointer<vtkCellArray>::New();
        newTris->InsertNextCell(GetTriangle(squarePoints, 0, 1, 2, planeNormal));
        newTris->InsertNextCell(GetTriangle(squarePoints, 0, 2, 3, planeNormal));

        squarePolyData->SetPoints(points);
        squarePolyData->SetPolys(newTris);

        return squarePolyData;
    }

    void UniformPolygonTriangulation::SetPolygonPoints(const std::vector<vtkVector3d>& polygonPoints)
    {
        mPolygonPoints = polygonPoints;
    }

    void UniformPolygonTriangulation::SetHoles(const std::vector<std::vector<vtkVector3d>>& holes)
    {
        if (holes.empty())
            return;

        mInnerHoles = holes;
    }

    void UniformPolygonTriangulation::SetNormal(const vtkVector3d& planeNormal)
    {
        mNormal = planeNormal;
        InitializePlane();
    }

    void UniformPolygonTriangulation::SetSquareSize(const double size)
    {
        if (size <= 0)
            return;

        mLength = size;
    }

    void UniformPolygonTriangulation::InitializePlane()
    {
        mPlaneCenter = Utility::GetAverageCenter(mPolygonPoints);

        double axisX[3];
        double axisY[3];
        Utility::GetPlaneAxes(mPlaneCenter.GetData(), mNormal.GetData(), axisX, axisY);

        mAxisX = vtkVector3d(axisX);
        mAxisY = vtkVector3d(axisY);
    }

    void UniformPolygonTriangulation::InitializePolygon()
    {
        vtkBoundingBox polygonBoundingBox;
        mPolygonPointsData2d.clear();

        double minXProj = DBL_MAX;
        double maxXProj = -DBL_MAX;
        double minYProj = DBL_MAX;
        double maxYProj = -DBL_MAX;

        for (const auto& point: mPolygonPoints)
        {
            //projection along local axes
            auto cp = point - mPlaneCenter;
            auto xProj = cp.Dot(mAxisX);
            auto yProj = cp.Dot(mAxisY);

            polygonBoundingBox.AddPoint(xProj, yProj, 0);
            mPolygonPointsData2d.push_back(xProj);
            mPolygonPointsData2d.push_back(yProj);
            mPolygonPointsData2d.push_back(0);

            if (xProj > maxXProj)
                maxXProj = xProj;

            if (xProj < minXProj)
                minXProj = xProj;

            if (yProj > maxYProj)
                maxYProj = yProj;

            if (yProj < minYProj)
                minYProj = yProj;
        }

        polygonBoundingBox.GetBounds(mPolygonBounds);//bounds of AABB

        //obb bounds
        mBoundingBoxWidth = maxXProj - minXProj;
        mBoundingBoxHeight = maxYProj - minYProj;
        mUpperLeftCorner = mPlaneCenter + mAxisX * minXProj + mAxisY * maxYProj;
    }

    void UniformPolygonTriangulation::InitializeHoles()
    {
        for (const auto& hole: mInnerHoles)
        {
            vtkBoundingBox holeBoundingBox;
            std::vector<double> holePointsData2d;
            for (const auto& point: hole)
            {
                //projection along local axes
                auto cp = point - mPlaneCenter;
                auto xProj = cp.Dot(mAxisX);
                auto yProj = cp.Dot(mAxisY);

                holeBoundingBox.AddPoint(xProj, yProj, 0);
                holePointsData2d.push_back(xProj);
                holePointsData2d.push_back(yProj);
                holePointsData2d.push_back(0);
            }
            mHolePointsData2dVector.push_back(holePointsData2d);

            double holeBounds[6];
            holeBoundingBox.GetBounds(holeBounds);

            std::vector<double> boundsVec(holeBounds, holeBounds + sizeof(holeBounds) / sizeof(holeBounds[0]));
            mHolesBounds.push_back(boundsVec);
        }
    }

    bool UniformPolygonTriangulation::IsInPolygon(const vtkVector3d& point)
    {
        //projection along local axes
        auto cp = point - mPlaneCenter;
        auto xProj = cp.Dot(mAxisX);
        auto yProj = cp.Dot(mAxisY);

        double pointToQuery[3]{xProj, yProj, 0};
        double normal2d[3]{0, 0, 1};
        return vtkPolygon::PointInPolygon(pointToQuery, mPolygonPoints.size(), mPolygonPointsData2d.data(), mPolygonBounds, normal2d);
    }

    bool UniformPolygonTriangulation::IsInHole(const vtkVector3d& point, int holeID)
    {
        if (holeID >= mInnerHoles.size())
            return false;

        //projection along local axes
        auto cp = point - mPlaneCenter;
        auto xProj = cp.Dot(mAxisX);
        auto yProj = cp.Dot(mAxisY);

        double pointToQuery[3]{xProj, yProj, 0};
        double normal2d[3]{0, 0, 1};

        return vtkPolygon::PointInPolygon(pointToQuery, mInnerHoles[holeID].size(), mHolePointsData2dVector[holeID].data(), mHolesBounds[holeID].data(),
                                          normal2d);
    }

    bool UniformPolygonTriangulation::AllPointsInPolygon(const std::vector<vtkVector3d>& points)
    {
        bool allInside = true;
        for (const auto& point: points)
        {
            if (!IsInPolygon(point))
            {
                allInside = false;
                break;
            }
        }

        return allInside;
    }

    bool UniformPolygonTriangulation::AllPointsInHole(const std::vector<vtkVector3d>& points)
    {
        bool isInAnyHole = false;
        for (int holeID = 0; holeID < mInnerHoles.size(); ++holeID)
        {
            bool allInside = true;
            for (const auto& point: points)
            {
                if (!IsInHole(point, holeID))
                {
                    allInside = false;
                    break;
                }
            }

            if (allInside)
            {
                isInAnyHole = true;
                break;
            }
        }

        return isInAnyHole;
    }

    std::vector<std::vector<vtkVector3d>> UniformPolygonTriangulation::GetSquarePolygonIntersectionPolygon(const std::vector<vtkVector3d>& squarePoints)
    {
        SquarePolygonIntersection polygonIntersection;
        polygonIntersection.SetPolygonPoints(mPolygonPoints);
        polygonIntersection.SetHoles(mInnerHoles);
        polygonIntersection.SetSquarePoints(squarePoints);
        polygonIntersection.SetPlane(mPlaneCenter, mNormal, mAxisX, mAxisY);
        polygonIntersection.CalculateIntersectionPolygons();

        return polygonIntersection.GetIntersectedPolygon();
    }

    double WeightOf(const vtkVector3d& p0, const vtkVector3d& p1, const vtkVector3d& p2)
    {
        std::vector<double> edgeLengths;
        edgeLengths.push_back((p1 - p0).Norm());
        edgeLengths.push_back((p2 - p1).Norm());
        edgeLengths.push_back((p0 - p2).Norm());

        return *std::max_element(edgeLengths.begin(), edgeLengths.end()) / *std::min_element(edgeLengths.begin(), edgeLengths.end());
    }

    vtkSmartPointer<vtkPolyData> GetOptimalTriangulation(const std::vector<vtkVector3d>& polygonPoints, const vtkVector3d& normal)
    {
        OptimalPolygonTriangulation triangulation;
        triangulation.SetPolygonPoints(polygonPoints);
        triangulation.SetNormal(normal);
        triangulation.Triangulate(WeightOf);

        return triangulation.GetTriangulatedPolygon();
    }

    void UniformPolygonTriangulation::Triangulate()
    {
        InitializePolygon();
        bool hasHoles = !mInnerHoles.empty();
        if (hasHoles)
        {
            std::cout << "holes cnt: " << mInnerHoles.size() << std::endl;
            InitializeHoles();
        }

        if (mLength == 0)
            mLength = std::min(mBoundingBoxWidth, mBoundingBoxHeight) * 0.2;

        if (mDebug)
            std::cout << "bounding box width = " << mBoundingBoxWidth << ", height = " << mBoundingBoxHeight << ", pixel size = " << mLength << std::endl;

        //make a square whose orientation is same as polygon
        std::vector<vtkVector3d> startSquarePoints(4);
        startSquarePoints[0] = mUpperLeftCorner;
        startSquarePoints[1] = mUpperLeftCorner - mAxisY * mLength;
        startSquarePoints[2] = startSquarePoints[1] + mAxisX * mLength;
        startSquarePoints[3] = mUpperLeftCorner + mAxisX * mLength;

        double squareNormal[3];
        Utility::ComputePolygonNormal(startSquarePoints, squareNormal);

        if (vtkVector3d(squareNormal).Dot(mNormal) < 0)
            std::reverse(startSquarePoints.begin(), startSquarePoints.end());

        std::vector<vtkVector3d> squarePoints(4);
        int x = 0, y = 0;
        for (double offsetX = 0; offsetX < mBoundingBoxWidth; offsetX += mLength)
        {
            y = 0;
            for (double offsetY = 0; offsetY < mBoundingBoxHeight; offsetY += mLength)
            {
                for (int i = 0; i < squarePoints.size(); ++i)
                {
                    squarePoints[i] = startSquarePoints[i];
                    squarePoints[i] += offsetX * mAxisX;
                    squarePoints[i] -= offsetY * mAxisY;
                }

                if (mDebug)
                    debugSquarePoints = squarePoints;

                bool allSquarePointsInPolygon = AllPointsInPolygon(squarePoints);

                //check if all square points are inside polygon
                if (!hasHoles && allSquarePointsInPolygon)
                {
                    if (mDebug)
                        std::cout << "no holes and all square points inside polygon. Make square" << std::endl;
                    mSubTriangulationVector.push_back(MakeSquare(squarePoints, mNormal));
                    continue;
                }

                if (hasHoles)
                {
                    bool allSquarePointsInHole = AllPointsInHole(squarePoints);

                    if (allSquarePointsInHole)
                    {
                        //leave this as blank area
                        if (mDebug)
                            std::cout << "all square points in hole" << std::endl;
                        continue;
                    }
                }

                //calculate intersected polygon and triangulate
                auto subPolygons = GetSquarePolygonIntersectionPolygon(squarePoints);
                if (subPolygons.empty() && allSquarePointsInPolygon)
                {
                    if (mDebug)
                        std::cout << "outside hole and inside polygon. make square" << std::endl;
                    mSubTriangulationVector.push_back(MakeSquare(squarePoints, mNormal));
                    continue;
                }

                for (const auto& subPolygon: subPolygons)
                {
                    if (!subPolygon.empty())
                    {
                        if (mDebug)
                        {
                            std::cout << "sub polygon points cnt = " << subPolygon.size() << std::endl;
                            debugSubPolygons.push_back(Utility::GetPolygonPolyData(subPolygon));
                        }

                        auto triangulatedPolygon = GetOptimalTriangulation(subPolygon, mNormal);
                        mSubTriangulationVector.push_back(triangulatedPolygon);
                    }
                }
                y++;
            }
            x++;
        }
    }

    vtkSmartPointer<vtkPolyData> UniformPolygonTriangulation::GetTriangulatedPolygon() const
    {
        return Utility::GetCombinedPolyData(mSubTriangulationVector);
    }

    vtkVector3d UniformPolygonTriangulation::GetPlaneCenter() const
    {
        return mPlaneCenter;
    }

    vtkVector3d UniformPolygonTriangulation::GetAxisX() const
    {
        return mAxisX;
    }

    vtkVector3d UniformPolygonTriangulation::GetAxisY() const
    {
        return mAxisY;
    }

    vtkVector3d UniformPolygonTriangulation::GetUpperLeftCorner() const
    {
        return mUpperLeftCorner;
    }

    double UniformPolygonTriangulation::GetBoundingBoxWidth() const
    {
        return mBoundingBoxWidth;
    }

    double UniformPolygonTriangulation::GetBoundingBoxHeight() const
    {
        return mBoundingBoxHeight;
    }
}
