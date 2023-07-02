#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkColor.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkCellPicker.h>
#include <vtkVectorOperators.h>

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

#include <chrono>

#include "Utility/ActorUtil.h"
#include "Utility/PolygonUtil.h"
#include "Utility/TestUtil.h"

#include "Algorithm/UniformPolygonTriangulation.h"

void TestScanLineTriangulation(bool debug = false)
{
    using namespace Algorithm;

    std::vector<vtkVector3d> polygonPoints;
    std::vector<std::vector<vtkVector3d>> holes;

    polygonPoints.push_back(vtkVector3d(2.51, 8.32, 0));
    polygonPoints.push_back(vtkVector3d(1.86, 13.72, 0));
    polygonPoints.push_back(vtkVector3d(3.56, 19.92, 0));
    polygonPoints.push_back(vtkVector3d(9.46, 17.52, 0));
    polygonPoints.push_back(vtkVector3d(20.71, 19.52, 0));
    polygonPoints.push_back(vtkVector3d(18.61, 15.27, 0));
    polygonPoints.push_back(vtkVector3d(21.86, 9.52, 0));
    polygonPoints.push_back(vtkVector3d(11.21, 11.27, 0));

    std::vector<vtkVector3d> hole0;
    hole0.push_back(vtkVector3d(5.31, 12.67, 0));
    hole0.push_back(vtkVector3d(7.91, 12.77, 0));
    hole0.push_back(vtkVector3d(7.96, 14.92, 0));
    hole0.push_back(vtkVector3d(5.46, 15.07, 0));

    std::vector<vtkVector3d> hole1;
    hole1.push_back(vtkVector3d(12.31, 13.42, 0));
    hole1.push_back(vtkVector3d(14.76, 12.92, 0));
    hole1.push_back(vtkVector3d(16.91, 13.62, 0));
    hole1.push_back(vtkVector3d(16.11, 16.42, 0));
    hole1.push_back(vtkVector3d(12.61, 16.32, 0));

    holes.push_back(hole0);
    holes.push_back(hole1);

    double polygonNormal[3];
    Utility::ComputePolygonNormal(polygonPoints, polygonNormal);

    std::unique_ptr<UniformPolygonTriangulation> triangulation(new UniformPolygonTriangulation());
    triangulation->mDebug = debug;
    triangulation->SetPolygonPoints(polygonPoints);
    triangulation->SetHoles(holes);
    triangulation->SetNormal(vtkVector3d(polygonNormal));
    triangulation->Triangulate();


    vtkNew<vtkNamedColors> colors;
    //actors
    std::vector<vtkSmartPointer<vtkActor>> actors;
    {
        //polygon
        auto polygonPolyData = Utility::GetPolygonPolyData(polygonPoints);
        TestUtil::AddPolyData(polygonPolyData, colors->GetColor3d("LightSteelBlue").GetData(), actors, true);
        for (int i = 0; i < polygonPoints.size(); ++i)
        {
            auto color = i == 0 ? colors->GetColor3d("DarkSalmon").GetData() : colors->GetColor3d("LightGreen").GetData();
            if (i == polygonPoints.size() - 1)
                color = colors->GetColor3d("Black").GetData();

            TestUtil::AddPoint(polygonPoints[i], 8, color, actors);
        }

        //holes
        for (int holeID = 0; holeID < holes.size(); ++holeID)
        {
            auto hole = holes[holeID];
            auto holePolyData = Utility::GetPolygonPolyData(hole);
            TestUtil::AddPolyData(holePolyData, colors->GetColor3d("Navy").GetData(), actors, true);

            auto middlePointsColor = holeID == 0 ? colors->GetColor3d("Thistle").GetData() : colors->GetColor3d("Indigo").GetData();
            for (int i = 0; i < hole.size(); ++i)
            {
                auto color = i == 0 ? colors->GetColor3d("DarkSalmon").GetData() : middlePointsColor;
                if (i == hole.size() - 1)
                    color = colors->GetColor3d("Black").GetData();
                TestUtil::AddPoint(hole[i], 8, color, actors);
            }
        }

        auto axisX = triangulation->GetAxisX();
        auto axisY = triangulation->GetAxisY();

        if (debug)
        {
            //bounding box
            {
                auto upperLeftCorner = triangulation->GetUpperLeftCorner();
                TestUtil::AddPoint(upperLeftCorner, 8, colors->GetColor3d("Yellow").GetData(), actors);

                auto width = triangulation->GetBoundingBoxWidth();
                auto height = triangulation->GetBoundingBoxHeight();

                auto lowerRightCorner = upperLeftCorner + axisX * width - axisY * height;
                auto color = colors->GetColor3d("DarkSlateGray").GetData();
                int lineWidth = 5;
                TestUtil::AddVector(upperLeftCorner, upperLeftCorner + axisX * width, lineWidth, color, actors);
                TestUtil::AddVector(upperLeftCorner, upperLeftCorner - axisY * height, lineWidth, color, actors);
                TestUtil::AddVector(lowerRightCorner, lowerRightCorner - axisX * width, lineWidth, color, actors);
                TestUtil::AddVector(lowerRightCorner, lowerRightCorner + axisY * height, lineWidth, color, actors);
            }

            //axes
            {
                auto planeCenter = triangulation->GetPlaneCenter();
                const double length = 30;
                TestUtil::AddVector(planeCenter, planeCenter + axisX * length, 4, colors->GetColor3d("Red").GetData(), actors);
                TestUtil::AddVector(planeCenter, planeCenter + axisY * length, 4, colors->GetColor3d("Green").GetData(), actors);
                TestUtil::AddVector(planeCenter, planeCenter + vtkVector3d(polygonNormal) * length, 4, colors->GetColor3d("Blue").GetData(), actors);
            }
        }

        //result
        if (!triangulation->mDebug)
        {
            auto offset = axisX * ((polygonPolyData->GetBounds()[1] - polygonPolyData->GetBounds()[0]) + 10);
            auto triangulatedPolygon = triangulation->GetTriangulatedPolygon();
            auto polydataActor = Utility::GetPolyDataActor(triangulatedPolygon, colors->GetColor3d("DarkOliveGreen").GetData());
            polydataActor->SetPosition(offset.GetData());
            polydataActor->GetProperty()->SetRepresentationToWireframe();
            actors.push_back(polydataActor);
        }

        //sub polygons
        if (triangulation->mDebug)
        {
            //square
            auto squarePoints = triangulation->debugSquarePoints;
            TestUtil::AddPoint(squarePoints[0], 8, colors->GetColor3d("Crimson").GetData(), actors);
            TestUtil::AddPoint(squarePoints[1], 8, colors->GetColor3d("Cyan").GetData(), actors);
            TestUtil::AddPoint(squarePoints[2], 8, colors->GetColor3d("Blue").GetData(), actors);
            TestUtil::AddPoint(squarePoints[3], 8, colors->GetColor3d("RosyBrown").GetData(), actors);

            auto subPolygons = triangulation->debugSubPolygons;
            for (auto polygon: subPolygons)
                TestUtil::AddPolyData(polygon, colors->GetColor3d("DarkGreen").GetData(), actors, true);
        }
    }

    //rendering
    vtkNew<vtkRenderer> renderer;
    vtkNew<vtkRenderWindow> renderWindow;
    renderWindow->SetWindowName("Sphere");
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(1200, 800);

    vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
    renderWindowInteractor->SetRenderWindow(renderWindow);

    for (auto actor: actors)
        renderer->AddActor(actor);
    renderer->SetBackground(colors->GetColor3d("Silver").GetData());

    renderWindow->Render();
    renderWindowInteractor->Start();
}

int main(int argc, char* argv[])
{
    bool debug = false;

    auto start = std::chrono::high_resolution_clock::now();

    TestScanLineTriangulation(debug);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "TestScanLineTriangulation() takes " << duration.count() << " ms" << std::endl;
}