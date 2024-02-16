#include "SquarePolygonIntersection.h"

#include "../Utility/PolygonUtil.h"
#include "../Utility/CommonUtil.h"

#include <vtkBoundingBox.h>
#include <vtkVectorOperators.h>

#include <queue>
#include <cfloat>


namespace Algorithm
{
    const int INVALID_POLYGON_INDEX = -1;

    void SortIntersectionPoints(std::vector<vtkVector3d>& points, const vtkVector3d& start, const vtkVector3d& end)
    {
        if(points.size() < 2)
            return;

        //sort intersection points along start->end based on t
        std::vector<double> tVec;
        const auto dir = end - start;
        for(const auto& point: points)
        {
            tVec.push_back((point - start).Dot(dir));
        }

        auto comparator = [&tVec](size_t i1, size_t i2)
        {
            return tVec[i1] < tVec[i2];
        };

        auto sortedIndices = Utility::SortIndices(tVec, comparator);
        auto temp = points;
        for(int i = 0; i < points.size(); ++i)
        {
            points[i] = temp[sortedIndices[i]];
        }
    }

    void SquarePolygonIntersection::SetPolygonPoints(const std::vector<vtkVector3d>& polygonPoints)
    {
        mOuterContourPoints = polygonPoints;
        mUpdatePolygon = true;
    }

    void SquarePolygonIntersection::SetHoles(const std::vector<std::vector<vtkVector3d>>& holes)
    {
        if(Utility::HasElements(holes))
        {
            mInnerHoles = holes;
            mUpdateHoles = true;
        }
    }

    void SquarePolygonIntersection::SetSquarePoints(const std::vector<vtkVector3d>& squarePoints)
    {
        mSquarePoints = squarePoints;
        mUpdateSquare = true;
    }

    void SquarePolygonIntersection::SetPlane(const vtkVector3d& planeCenter, const vtkVector3d& axisX, const vtkVector3d& axisY)
    {
        mPlaneCenter = planeCenter;
        mAxisX = axisX;
        mAxisY = axisY;

        mUpdatePolygon = true;
        mUpdateHoles = true;
    }

    void SquarePolygonIntersection::SetPrecision(const double epsilon)
    {
        mEpsilon = epsilon;
    }

    void SquarePolygonIntersection::InitializePolygon()
    {
        mUpdatePolygon = false;
        mOuterPolyLines.clear();
        mOuterPolygonPointsData2d.clear();

        const int n = mOuterContourPoints.size();
        for(int i = 0; i < n; ++i)
        {
            //projection along local axes
            auto cp = mOuterContourPoints[i] - mPlaneCenter;
            auto xProj = cp.Dot(mAxisX);
            auto yProj = cp.Dot(mAxisY);

            mOuterPolygonPointsData2d.push_back(xProj);
            mOuterPolygonPointsData2d.push_back(yProj);

            mOuterPolyLines.push_back(std::pair(i, (i + 1) % n));
        }

        mContourPoints = mOuterContourPoints;
        mContourPointsData2d = mOuterPolygonPointsData2d;
        mContourPolyLines = mOuterPolyLines;
    }

    void SquarePolygonIntersection::InitializeHoles()
    {
        if(mInnerHoles.empty())
            return;

        mContourPoints = mOuterContourPoints;
        mContourPointsData2d = mOuterPolygonPointsData2d;
        mContourPolyLines = mOuterPolyLines;

        mUpdateHoles = false;
        mComponentEndIndices.clear();
        mComponentIntervals.clear();

        int intervalStart = 0;
        int holeStartIndex = mContourPoints.size();
        mComponentEndIndices.insert(holeStartIndex - 1);
        mComponentIntervals.push_back(std::pair(intervalStart, holeStartIndex - 1));
        intervalStart = holeStartIndex;

        for(const auto& hole: mInnerHoles)
        {
            for(const auto& point: hole)
            {
                mContourPoints.push_back(point);

                //projection along local axes
                auto cp = point - mPlaneCenter;
                auto xProj = cp.Dot(mAxisX);
                auto yProj = cp.Dot(mAxisY);

                mContourPointsData2d.push_back(xProj);
                mContourPointsData2d.push_back(yProj);
            }

            const int holePointsCnt = hole.size();
            for(int i = holeStartIndex; i < holeStartIndex + holePointsCnt; ++i)
            {
                int nextIndex = i + 1 < holeStartIndex + holePointsCnt ? i + 1 : i + 1 - holePointsCnt;
                mContourPolyLines.push_back(std::pair(i, nextIndex));
                if(mDebug)
                    printf("mContourPolyLines add(%i,%i)\n", i, nextIndex);
            }

            holeStartIndex += holePointsCnt;
            mComponentEndIndices.insert(holeStartIndex - 1);
            mComponentIntervals.push_back(std::pair(intervalStart, holeStartIndex - 1));
            intervalStart = holeStartIndex;
        }
    }

    void SquarePolygonIntersection::InitializeSquare()
    {
        mUpdateSquare = false;
        mSquarePointsData2d.clear();

        double minXProj = DBL_MAX;
        double maxXProj = -DBL_MAX;
        double minYProj = DBL_MAX;
        double maxYProj = -DBL_MAX;

        for(const auto& point: mSquarePoints)
        {
            //projection along local axes
            const auto cp = point - mPlaneCenter;
            const auto xProj = cp.Dot(mAxisX);
            const auto yProj = cp.Dot(mAxisY);

            mSquarePointsData2d.push_back(xProj);
            mSquarePointsData2d.push_back(yProj);

            if(xProj > maxXProj)
                maxXProj = xProj;

            if(xProj < minXProj)
                minXProj = xProj;

            if(yProj > maxYProj)
                maxYProj = yProj;

            if(yProj < minYProj)
                minYProj = yProj;
        }

        mSquareBounds[0] = minXProj;
        mSquareBounds[1] = maxXProj;
        mSquareBounds[2] = minYProj;
        mSquareBounds[3] = maxYProj;
    }

    bool SquarePolygonIntersection::PointInSquare(const vtkVector3d& point) const
    {
        const auto cp = (point - mPlaneCenter);
        const double xProj = cp.Dot(mAxisX);
        const double yProj = cp.Dot(mAxisY);

        return (xProj > mSquareBounds[0] || abs(xProj - mSquareBounds[0]) < mEpsilon) &&
               (xProj < mSquareBounds[1] || abs(xProj - mSquareBounds[1]) < mEpsilon) &&
               (yProj > mSquareBounds[2] || abs(yProj - mSquareBounds[2]) < mEpsilon) &&
               (yProj < mSquareBounds[3] || abs(yProj - mSquareBounds[3]) < mEpsilon);
    }

    /**
     * only return TRUE if intersection point of two lines is not end point of any line
     */
    bool SquarePolygonIntersection::LineIntersects(const int squareLineID, const std::pair<int, int>& polyLine, vtkVector3d& intersectionPoint) const
    {
        const int squareStartIndex = squareLineID;
        const int squareEndIndex = (squareLineID + 1) % mSquarePoints.size();
        vtkVector3d squareStartPoint(mSquarePointsData2d[squareStartIndex * 2], mSquarePointsData2d[squareStartIndex * 2 + 1], 0);
        vtkVector3d squareEndPoint(mSquarePointsData2d[squareEndIndex * 2], mSquarePointsData2d[squareEndIndex * 2 + 1], 0);

        const int polyStartIndex = polyLine.first;
        const int polyEndIndex = polyLine.second;
        vtkVector3d polyStartPoint(mContourPointsData2d[polyStartIndex * 2], mContourPointsData2d[polyStartIndex * 2 + 1], 0);
        vtkVector3d polyEndPoint(mContourPointsData2d[polyEndIndex * 2], mContourPointsData2d[polyEndIndex * 2 + 1], 0);

        vtkVector3d intersectionPoint2d;
        const auto intersectionType = Utility::GetLineIntersectionType(squareStartPoint, squareEndPoint, polyStartPoint, polyEndPoint, intersectionPoint2d);

        if(intersectionType == Utility::LineIntersectionType::Intersection)
        {
            //convert intersection point back to 3d
            intersectionPoint = mPlaneCenter + mAxisX * intersectionPoint2d[0] + mAxisY * intersectionPoint2d[1];
            return true;
        }

        return false;
    }

    bool SquarePolygonIntersection::HasIntersection()
    {
        mIntersectionPointsSet.clear();

        //calculate intersection
        std::unordered_map<int, std::vector<vtkVector3d>> polygonMap;
        std::unordered_map<int, std::vector<vtkVector3d>> squareMap;
        {
            vtkVector3d intersectionPoint;
            for(int i = 0; i < mSquarePoints.size(); ++i)
            {
                //TODO: if square edge is completely inside polygon there would be no intersection
                for(int j = 0; j < mContourPolyLines.size(); ++j)
                {
                    const auto polyLine = mContourPolyLines[j];

                    //TODO: if line is completely inside square there would be no intersection

                    //TODO: if polyline end points are on lines of square, then intersection point would be polyline end points
                    if(LineIntersects(i, polyLine, intersectionPoint))
                    {
                        if(mDebug)
                            std::cout << "polyline " << j << " intersects with square line " << i << ", intersection point: " << intersectionPoint << std::endl;

                        mIntersectionPointsSet.insert(intersectionPoint);

                        if(squareMap.count(i) == 0)//entry doesn't exists
                        {
                            std::vector<vtkVector3d> intPntsVector;
                            intPntsVector.push_back(intersectionPoint);
                            squareMap.insert({i, intPntsVector});
                        }
                        else
                        {
                            squareMap[i].push_back(intersectionPoint);
                        }

                        if(polygonMap.count(j) == 0)//entry doesn't exists
                        {
                            std::vector<vtkVector3d> intPntsVector;
                            intPntsVector.push_back(intersectionPoint);
                            polygonMap.insert({j, intPntsVector});
                        }
                        else
                        {
                            polygonMap[j].push_back(intersectionPoint);
                        }
                    }
                }
            }
        }

        if(polygonMap.empty() && squareMap.empty())
            return false;

        //store 2 separate arrays, one for polygon, one for square
        SetUpPolygonIntersection(polygonMap);
        SetUpSquareIntersection(squareMap);

        return true;
    }

    int SquarePolygonIntersection::GetNextPolygonIndex(int index) const
    {
        if(mInnerHoles.empty())
            return (index + 1) % mOuterContourPoints.size();

        for(const auto& range: mComponentIntervals)
        {
            if(index >= range.first && index <= range.second)
                return index + 1 <= range.second ? index + 1 : range.first;
        }

        return INVALID_POLYGON_INDEX;
    }

    void SquarePolygonIntersection::SetUpPolygonIntersection(const std::unordered_map<int, std::vector<vtkVector3d>>& polygonMap)
    {
        mPolygonVertices.clear();
        mEnterIndices.clear();
        mIsExitPoint.clear();
        mIntersectionIntervals.clear();

        const bool hasHoles = !mInnerHoles.empty();
        if(mDebug && hasHoles)
        {
            std::cout << "Inside SetUpPolygonIntersection(). End indices:";
            for(const auto index: mComponentEndIndices)
                std::cout << index << " ";
            std::cout << std::endl;
        }

        //store the indices of enter event points, and store whether a point is an exit event
        mIsExitPoint.reserve(mContourPoints.size() + mIntersectionPointsSet.size());
        int index = 0;
        int intervalStart = 0;

        for(int i = 0; i < mContourPoints.size(); ++i)
        {
            const auto start = mContourPoints[i];
            mPolygonVertices.push_back(start);
            mIsExitPoint.push_back(false);//polygon points will never be exit event point
            index++;
            if(mDebug)
                std::cout << "push polygon " << i << start << std::endl;

            if(polygonMap.count(i) > 0)//key exists
            {
                auto intersectionPoints = polygonMap.at(i);
                const int endIndex = GetNextPolygonIndex(i);
                const auto end = mContourPoints[endIndex];

                if(intersectionPoints.size() == 1)
                {
                    mPolygonVertices.push_back(intersectionPoints[0]);

                    const bool isEnter = !PointInSquare(start) && PointInSquare(end);

                    if(mDebug)
                    {
                        const bool startIn = PointInSquare(start);
                        const bool endIn = PointInSquare(end);
                        printf("intersection start = %i, end = %i; startIn = %s, endIn = %s, isEnter = %s\n", i, endIndex, startIn ? "true" : "false",
                               endIn ? "true" : "false", isEnter ? "true" : "false");
                    }

                    mIsExitPoint.push_back(!isEnter);
                    if(isEnter)
                        mEnterIndices.push_back(index);
                    index++;

                    if(mDebug)
                        std::cout << "push intersection point " << intersectionPoints[0] << std::endl;
                }
                else//more than 1 intersection points on one polyline
                {
                    if(mDebug)
                        printf("there are %zu intersection points on one polyline\n", intersectionPoints.size());
                    SortIntersectionPoints(intersectionPoints, start, end);

                    for(int k = 0; k < intersectionPoints.size(); ++k)
                    {
                        //the first intersection point is enter event
                        if(k == 0)
                            mEnterIndices.push_back(index);
                        mIsExitPoint.push_back(k == intersectionPoints.size() - 1);//last point is exit event
                        mPolygonVertices.push_back(intersectionPoints[k]);
                        index++;

                        if(mDebug)
                            std::cout << "push intersection point " << intersectionPoints[k] << std::endl;
                    }
                }
            }

            if(hasHoles && mComponentEndIndices.count(i))
            {
                mIntersectionIntervals.push_back(std::pair(intervalStart, index - 1));
                intervalStart = index;
            }
        }

        if(mDebug && hasHoles)
        {
            std::cout << "intersection intervals:";
            for(const auto& range: mIntersectionIntervals)
            {
                printf("[%i,%i]\n", range.first, range.second);
            }
        }
    }

    void SquarePolygonIntersection::SetUpSquareIntersection(const std::unordered_map<int, std::vector<vtkVector3d>>& squareMap)
    {
        mSquareVertices.clear();

        for(int i = 0; i < mSquarePoints.size(); ++i)
        {
            const auto start = mSquarePoints[i];
            mSquareVertices.push_back(start);
            if(squareMap.count(i) > 0)//key exists
            {
                auto intersectionPoints = squareMap.at(i);
                SortIntersectionPoints(intersectionPoints, start, mSquarePoints[(i + 1) % mSquarePoints.size()]);

                for(const auto& point: intersectionPoints)
                    mSquareVertices.push_back(point);
            }
        }
    }

    int SquarePolygonIntersection::GetNextIntersectionPolygonIndex(int index) const
    {
        if(mInnerHoles.empty())
            return (index + 1) % mPolygonVertices.size();

        for(const auto& range: mIntersectionIntervals)
        {
            if(index >= range.first && index <= range.second)
                return index + 1 <= range.second ? index + 1 : range.first;
        }

        return INVALID_POLYGON_INDEX;
    }

    void SquarePolygonIntersection::CalculateIntersectedPolygons()
    {
        mSubPolygons.clear();
        const bool hasHoles = !mInnerHoles.empty();

        if(mUpdatePolygon)
            InitializePolygon();

        if(mUpdateSquare)
            InitializeSquare();

        if(hasHoles && mUpdateHoles)
            InitializeHoles();

        if(!HasIntersection())
            return;

        if(mDebug)
        {
            printf("polygon points count:%zu, intersection points count:%zu\n", mContourPoints.size(), mIntersectionPointsSet.size());
            std::cout << "enter indices: ";
            for(auto index: mEnterIndices)
                std::cout << index << " ";
            std::cout << std::endl;
        }

        //calculate intersections
        std::set<vtkVector3d> visited;

        for(int startIndex: mEnterIndices)
        {
            auto startPoint = mPolygonVertices[startIndex];
            if(visited.count(startPoint))//already visited the point
                continue;

            std::queue<vtkVector3d> queue;
            std::vector<vtkVector3d> polygonPoints;

            //add root to queue
            queue.push(startPoint);
            visited.insert(startPoint);
            while(!queue.empty())
            {
                queue.pop();

                //iterate through polygon until an exit is found
                while(true)
                {
                    startPoint = mPolygonVertices[startIndex];
                    polygonPoints.push_back(startPoint);
                    if(mIsExitPoint[startIndex] && !visited.count(startPoint))
                    {
                        if(mDebug)
                            std::cout << "break at exit index " << startIndex << std::endl;
                        visited.insert(startPoint);
                        break;
                    }
                    startIndex = GetNextIntersectionPolygonIndex(startIndex);
                }

                //switch to square
                int squareStartIndex = std::find(mSquareVertices.begin(), mSquareVertices.end(), startPoint) - mSquareVertices.begin();
                squareStartIndex = (squareStartIndex + 1) % mSquareVertices.size();
                if(mDebug)
                    std::cout << "switch to square, start index = " << squareStartIndex << std::endl;
                bool switchBackToPolygon = false;
                while(true)
                {
                    startPoint = mSquareVertices[squareStartIndex];
                    if(visited.count(startPoint))
                        break;

                    if(mIntersectionPointsSet.count(startPoint) && !visited.count(startPoint))
                    {
                        //switch back to polygon
                        switchBackToPolygon = true;
                        break;
                    }

                    polygonPoints.push_back(startPoint);
                    squareStartIndex = (squareStartIndex + 1) % mSquareVertices.size();
                }

                if(switchBackToPolygon)
                {
                    queue.push(startPoint);
                    startIndex = std::find(mPolygonVertices.begin(), mPolygonVertices.end(), startPoint) - mPolygonVertices.begin();
                    visited.insert(startPoint);
                    if(mDebug)
                        std::cout << "switch back to polygon, start index = " << startIndex << std::endl;
                }
            }//end of while(!queue.empty)

            //finished. this is a complete sub polygon
            mSubPolygons.push_back(polygonPoints);
        }
    }

    std::vector<std::vector<vtkVector3d>> SquarePolygonIntersection::GetIntersectedPolygon() const
    {
        return mSubPolygons;
    }
}
