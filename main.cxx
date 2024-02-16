#include "Utility/ActorUtil.h"
#include "Utility/PolygonUtil.h"
#include "Utility/TestUtil.h"
#include "Algorithm/UniformPolygonTriangulation.h"

#include <vtkPolyData.h>
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


void TestUniformTriangulation(const std::vector<vtkVector3d>& polygonPoints, const std::vector<std::vector<vtkVector3d>>& holes, bool debug = false)
{
    double polygonNormal[3];
    Utility::ComputePolygonNormal(polygonPoints, polygonNormal);

    auto start = std::chrono::high_resolution_clock::now();

    Algorithm::UniformPolygonTriangulation triangulation;
    triangulation.mDebug = debug;
    triangulation.SetPolygonPoints(polygonPoints);
    triangulation.SetHoles(holes);
    triangulation.SetNormal(vtkVector3d(polygonNormal));
    triangulation.Triangulate();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "UniformPolygonTriangulation takes " << duration.count() << " ms" << std::endl;

    //visualization
    auto colors = vtkSmartPointer<vtkNamedColors>::New();
    std::vector<vtkSmartPointer<vtkActor>> actors;
    {
        //polygon
        auto polygonPolyData = Utility::GetPolygonPolyData(polygonPoints);
        TestUtil::AddPolyData(polygonPolyData, colors->GetColor3d("LightSteelBlue").GetData(), actors, true);
        for(int i = 0; i < polygonPoints.size(); ++i)
        {
            auto color = i == 0 ? colors->GetColor3d("DarkSalmon").GetData() : colors->GetColor3d("LightGreen").GetData();
            if(i == polygonPoints.size() - 1)
                color = colors->GetColor3d("Black").GetData();

            TestUtil::AddPoint(polygonPoints[i], 8, color, actors);
        }

        //holes
        for(int holeID = 0; holeID < holes.size(); ++holeID)
        {
            auto hole = holes[holeID];
            auto holePolyData = Utility::GetPolygonPolyData(hole);
            TestUtil::AddPolyData(holePolyData, colors->GetColor3d("Navy").GetData(), actors, true);

            auto middlePointsColor = holeID == 0 ? colors->GetColor3d("Thistle").GetData() : colors->GetColor3d("Indigo").GetData();
            for(int i = 0; i < hole.size(); ++i)
            {
                auto color = i == 0 ? colors->GetColor3d("DarkSalmon").GetData() : middlePointsColor;
                if(i == hole.size() - 1)
                    color = colors->GetColor3d("Black").GetData();
                TestUtil::AddPoint(hole[i], 8, color, actors);
            }
        }

        //triangulation result
        {
            auto axisX = triangulation.GetAxisX();
            auto offset = axisX * ((polygonPolyData->GetBounds()[1] - polygonPolyData->GetBounds()[0]) + 10);
            auto triangulatedPolygon = triangulation.GetOutPut();
            auto polyDataActor = Utility::GetPolyDataActor(triangulatedPolygon, colors->GetColor3d("DarkOliveGreen").GetData());
            polyDataActor->SetPosition(offset.GetData());
            polyDataActor->GetProperty()->SetRepresentationToWireframe();
            actors.push_back(polyDataActor);
        }
    }

    //rendering
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetWindowName("Sphere");
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(1200, 800);

    auto renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    for(const auto& actor: actors)
        renderer->AddActor(actor);
    renderer->SetBackground(colors->GetColor3d("Silver").GetData());

    renderWindow->Render();
    renderWindowInteractor->Start();
}

int main(int argc, char* argv[])
{
    bool debug = false;

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

    TestUniformTriangulation(polygonPoints, holes, debug);
}
