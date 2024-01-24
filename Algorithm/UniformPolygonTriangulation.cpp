#include "UniformPolygonTriangulation.h"

#include <vtkPolygon.h>
#include <vtkVectorOperators.h>
#include <vtkTriangle.h>
#include <vtkCleanPolyData.h>

#include "SquarePolygonIntersection.h"
#include "OptimalPolygonTriangulation.h"
#include "../Utility/GeometricObjectUtil.h"
#include "../Utility/MeshUtil.h"
#include "../Utility/PolygonUtil.h"
#include "../Utility/CommonUtil.h"


namespace Algorithm
{
    void GetSquareBounds2D(const vtkVector3d& planeCenter, const vtkVector3d& axisX, const vtkVector3d& axisY,
                           const std::vector<vtkVector3d>& squarePoints, double squareBounds[4])
    {
        double minXProj = DBL_MAX;
        double maxXProj = -DBL_MAX;
        double minYProj = DBL_MAX;
        double maxYProj = -DBL_MAX;

        for(const auto& point: squarePoints)
        {
            //projection along local axes
            const auto cp = point - planeCenter;
            const auto xProj = cp.Dot(axisX);
            const auto yProj = cp.Dot(axisY);

            if(xProj > maxXProj)
                maxXProj = xProj;

            if(xProj < minXProj)
                minXProj = xProj;

            if(yProj > maxYProj)
                maxYProj = yProj;

            if(yProj < minYProj)
                minYProj = yProj;
        }

        squareBounds[0] = minXProj;
        squareBounds[1] = maxXProj;
        squareBounds[2] = minYProj;
        squareBounds[3] = maxYProj;
    }

    bool AllPolygonPointsOutsideSquare(const vtkVector3d& planeCenter, const vtkVector3d& axisX, const vtkVector3d& axisY,
                                       const std::vector<vtkVector3d>& squarePoints, const std::vector<vtkVector3d>& polygonPoints, const double epsilon = 1e-6)
    {
        double squareBounds[4];
        GetSquareBounds2D(planeCenter, axisX, axisY, squarePoints, squareBounds);

        for(const auto& polyPnt: polygonPoints)
        {
            //point is one of the square point. consider this outside square
            if(Utility::EpsilonContains(squarePoints, polyPnt))
                continue;

            //local projection
            const auto cp = (polyPnt - planeCenter);
            const double xProj = cp.Dot(axisX);
            const double yProj = cp.Dot(axisY);

            //point on square edge
            if(abs(xProj - squareBounds[0]) < epsilon || abs(xProj - squareBounds[1]) < epsilon ||
               abs(yProj - squareBounds[2]) < epsilon || abs(yProj - squareBounds[3]) < epsilon)
            {
                continue;
            }

            //check if it's outside square
            if(xProj > squareBounds[0] && xProj < squareBounds[1] && yProj > squareBounds[2] && yProj < squareBounds[3])
            {
                return false;
            }
        }

        return true;
    }

    bool AllPointsInPolygon(const std::vector<vtkVector3d>& polygonPoints, const std::vector<double>& polygonPointsData2d, const vtkVector3d& planeCenter,
                            const vtkVector3d& axisX, const vtkVector3d& axisY, const double polygonBounds[6], const std::vector<vtkVector3d>& points)
    {
        for(const auto& point: points)
        {
            //corner case: if point IS polygon point, it's considered inside polygon
            if(Utility::EpsilonContains(polygonPoints, point))
                continue;

            if(!Utility::PointInPolygon(polygonPoints.size(), polygonPointsData2d.data(), polygonBounds, planeCenter, axisX, axisY, point))
            {
                return false;
            }
        }

        return true;
    }

    bool AllPointsOutsidePolygon(const std::vector<vtkVector3d>& polygonPoints, const std::vector<double>& polygonPointsData2d, const vtkVector3d& planeCenter,
                                 const vtkVector3d& axisX, const vtkVector3d& axisY, const double polygonBounds[6], const std::vector<vtkVector3d>& points)
    {
        for(const auto& point: points)
        {
            //corner case: if point IS polygon point, it's considered outside polygon
            if(Utility::EpsilonContains(polygonPoints, point))
                continue;

            if(Utility::PointInPolygon(polygonPoints.size(), polygonPointsData2d.data(), polygonBounds, planeCenter, axisX, axisY, point))
            {
                return false;
            }
        }

        return true;
    }

    vtkSmartPointer<vtkPolyData> DivideSquareByDiagonal(const std::vector<vtkVector3d>& squarePoints, const vtkVector3d& planeNormal)
    {
        auto squarePolyData = vtkSmartPointer<vtkPolyData>::New();

        auto points = vtkSmartPointer<vtkPoints>::New();
        for(const auto& point: squarePoints)
            points->InsertNextPoint(point.GetData());

        auto newTris = vtkSmartPointer<vtkCellArray>::New();
        newTris->InsertNextCell(Utility::GetTriangle(squarePoints, 0, 1, 2, planeNormal));
        newTris->InsertNextCell(Utility::GetTriangle(squarePoints, 0, 2, 3, planeNormal));

        squarePolyData->SetPoints(points);
        squarePolyData->SetPolys(newTris);

        return squarePolyData;
    }

    double WeightOfTriangle(const vtkVector3d& p0, const vtkVector3d& p1, const vtkVector3d& p2)
    {
        std::vector<double> edgeLengths;
        edgeLengths.push_back((p1 - p0).Norm());
        edgeLengths.push_back((p2 - p1).Norm());
        edgeLengths.push_back((p0 - p2).Norm());

        return *std::max_element(edgeLengths.begin(), edgeLengths.end()) /
               *std::min_element(edgeLengths.begin(), edgeLengths.end());
    }

    void UniformPolygonTriangulation::SetPolygonPoints(const std::vector<vtkVector3d>& polygonPoints)
    {
        mPolygonPoints = polygonPoints;
        mUpdateCalculation = true;
    }

    void UniformPolygonTriangulation::SetHoles(const std::vector<std::vector<vtkVector3d>>& holes)
    {
        mInnerHoles = holes;
        mUpdateCalculation = true;
    }

    void UniformPolygonTriangulation::SetNormal(const vtkVector3d& planeNormal)
    {
        mNormal = planeNormal;
        mUpdateCalculation = true;
    }

    void UniformPolygonTriangulation::SetSquareSize(const double size)
    {
        if(size <= 0)
        {
            std::cerr << "square size should be greater than 0!" << std::endl;
            return;
        }

        mLength = size;
        mUpdateCalculation = true;
    }

    void UniformPolygonTriangulation::InitializePlane()
    {
        mPlaneCenter = Utility::GetAverage(mPolygonPoints);

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

        for(const auto& point: mPolygonPoints)
        {
            //projection along local axes
            const auto cp = point - mPlaneCenter;
            const auto xProj = cp.Dot(mAxisX);
            const auto yProj = cp.Dot(mAxisY);

            polygonBoundingBox.AddPoint(xProj, yProj, 0);
            mPolygonPointsData2d.push_back(xProj);
            mPolygonPointsData2d.push_back(yProj);
            mPolygonPointsData2d.push_back(0);

            if(xProj > maxXProj)
                maxXProj = xProj;

            if(xProj < minXProj)
                minXProj = xProj;

            if(yProj > maxYProj)
                maxYProj = yProj;

            if(yProj < minYProj)
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
        for(const auto& hole: mInnerHoles)
        {
            vtkBoundingBox holeBoundingBox;
            std::vector<double> holePointsData2d;
            for(const auto& point: hole)
            {
                //projection along local axes
                const auto cp = point - mPlaneCenter;
                const auto xProj = cp.Dot(mAxisX);
                const auto yProj = cp.Dot(mAxisY);

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

    vtkSmartPointer<vtkPolyData> UniformPolygonTriangulation::GetTriangulatedPolygon() const
    {
        const int squarePntsCnt = 4;

        //make a square whose orientation is same as polygon
        std::vector<vtkVector3d> startSquarePoints(squarePntsCnt);
        startSquarePoints[0] = mUpperLeftCorner;
        startSquarePoints[1] = mUpperLeftCorner - mAxisY * mLength;
        startSquarePoints[2] = startSquarePoints[1] + mAxisX * mLength;
        startSquarePoints[3] = mUpperLeftCorner + mAxisX * mLength;

        double squareNormal[3];
        Utility::ComputePolygonNormal(startSquarePoints, squareNormal);

        if(vtkVector3d(squareNormal).Dot(mNormal) < 0)
            std::reverse(startSquarePoints.begin(), startSquarePoints.end());

        //setup sub-polygon calculation. polygon and holes only need to be passed once
        SquarePolygonIntersection polygonIntersection;
        polygonIntersection.mDebug = mDebug;
        polygonIntersection.SetPolygonPoints(mPolygonPoints);
        polygonIntersection.SetHoles(mInnerHoles);
        polygonIntersection.SetPlane(mPlaneCenter, mAxisX, mAxisY);

        //set up optimal triangulation. weight function and normal only need to be passed once
        OptimalPolygonTriangulation triangulation;
        triangulation.SetNormal(mNormal);
        triangulation.SetWeightFunction(WeightOfTriangle);

        //iterate through each pixel
        std::vector<vtkVector3d> squarePoints(squarePntsCnt);
        std::vector<vtkSmartPointer<vtkPolyData>> subTriangulationVector;
        for(double offsetX = 0.0; offsetX < mBoundingBoxWidth; offsetX += mLength)
        {
            for(double offsetY = 0.0; offsetY < mBoundingBoxHeight; offsetY += mLength)
            {
                for(int i = 0; i < squarePoints.size(); ++i)
                {
                    squarePoints[i] = startSquarePoints[i];
                    squarePoints[i] += offsetX * mAxisX;
                    squarePoints[i] -= offsetY * mAxisY;
                }

                if(AllPolygonPointsOutsideSquare(mPlaneCenter, mAxisX, mAxisY, squarePoints, mPolygonPoints))
                {
                    if(AllPointsOutsidePolygon(mPolygonPoints, mPolygonPointsData2d, mPlaneCenter, mAxisX, mAxisY, mPolygonBounds, squarePoints))
                    {
                        //square is completely outside polygon
                        continue;
                    }

                    if(AllPointsInPolygon(mPolygonPoints, mPolygonPointsData2d, mPlaneCenter, mAxisX, mAxisY, mPolygonBounds, squarePoints))
                    {
                        if(!mHasHoles)
                        {
                            //square completely inside polygon
                            subTriangulationVector.push_back(DivideSquareByDiagonal(squarePoints, mNormal));
                            continue;
                        }
                        else
                        {
                            //check if square is completely inside any of the hole
                            //ie.all hole points outside square and all square points inside hole
                            bool insideOneOfHoles = false;
                            for(int holeID = 0; holeID < mInnerHoles.size(); ++holeID)
                            {
                                auto holePnts = mInnerHoles[holeID];
                                if(AllPolygonPointsOutsideSquare(mPlaneCenter, mAxisX, mAxisY, squarePoints, holePnts) &&
                                   AllPointsInPolygon(mInnerHoles[holeID], mHolePointsData2dVector[holeID], mPlaneCenter, mAxisX, mAxisY,
                                                      mHolesBounds[holeID].data(), squarePoints))
                                {
                                    //leave this as blank area
                                    if(mDebug)
                                        printf("square inside hole %i\n", holeID);
                                    insideOneOfHoles = true;
                                    break;
                                }
                            }

                            if(insideOneOfHoles)
                                continue;

                            //check if square is completely outside all holes
                            bool allHolePointsOutsideSquare = true;
                            for(const auto& hole: mInnerHoles)
                            {
                                if(!AllPolygonPointsOutsideSquare(mPlaneCenter, mAxisX, mAxisY, squarePoints, hole))
                                {
                                    allHolePointsOutsideSquare = false;
                                    break;
                                }
                            }

                            if(allHolePointsOutsideSquare)
                            {
                                bool allSquarePointsOutsideHoles = true;
                                for(int holeID = 0; holeID < mInnerHoles.size(); ++holeID)
                                {
                                    if(!AllPointsOutsidePolygon(mInnerHoles[holeID], mHolePointsData2dVector[holeID], mPlaneCenter, mAxisX, mAxisY,
                                                                mHolesBounds[holeID].data(), squarePoints))
                                    {
                                        allSquarePointsOutsideHoles = false;
                                        break;
                                    }
                                }

                                if(allSquarePointsOutsideHoles)
                                {
                                    subTriangulationVector.push_back(DivideSquareByDiagonal(squarePoints, mNormal));
                                    continue;
                                }
                            }
                        }
                    }
                }

                //calculate intersected polygon and triangulate
                polygonIntersection.SetSquarePoints(squarePoints);
                polygonIntersection.CalculateIntersectedPolygons();
                auto subPolygons = polygonIntersection.GetIntersectedPolygon();

                for(const auto& subPolygon: subPolygons)
                {
                    if(!subPolygon.empty())
                    {
                        if(mDebug)
                            printf("sub polygon points cnt = %zu\n", subPolygon.size());

                        triangulation.SetPolygonPoints(subPolygon);
                        triangulation.Triangulate();

                        auto triangulatedPolygon = triangulation.GetTriangulatedPolygon();
                        subTriangulationVector.push_back(triangulatedPolygon);
                    }
                }
            }
        }

        auto combinedPolyData = Utility::GetCombinedPolyData(subTriangulationVector);
        if(mDebug)
        {
            printf("combined poly data points cnt=%lli, cells cnt = %lli\n", combinedPolyData->GetNumberOfPoints(),
                   combinedPolyData->GetNumberOfCells());
        }

        //merge duplicate points
        auto cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
        cleanPolyData->SetInputData(combinedPolyData);
        cleanPolyData->ConvertPolysToLinesOff();
        cleanPolyData->Update();

        return cleanPolyData->GetOutput();
    }

    void UniformPolygonTriangulation::Triangulate()
    {
        if(!mUpdateCalculation)
            return;

        mUpdateCalculation = false;

        InitializePlane();
        InitializePolygon();

        mHasHoles = Utility::HasElements(mInnerHoles);
        if(mDebug)
            printf("has holes = %s, holes cnt: %zu\n", mHasHoles ? "True" : "False", mInnerHoles.size());

        if(mHasHoles)
            InitializeHoles();

        if(mLength == 0)
            mLength = std::min(mBoundingBoxWidth, mBoundingBoxHeight) * 0.15;

        if(mDebug)
            printf("bounding box width = %f, height = %f, pixel size = %f\n", mBoundingBoxWidth, mBoundingBoxHeight,
                   mLength);

        mTriangulatedPolygon = GetTriangulatedPolygon();

        if(mDebug)
        {
            printf("After cleanup, points cnt=%lli, cells cnt = %lli\n", mTriangulatedPolygon->GetNumberOfPoints(),
                   mTriangulatedPolygon->GetNumberOfCells());
        }
    }

    vtkSmartPointer<vtkPolyData> UniformPolygonTriangulation::GetOutPut() const
    {
        return mTriangulatedPolygon;
    }

    vtkVector3d UniformPolygonTriangulation::GetAxisX() const
    {
        return mAxisX;
    }
}
