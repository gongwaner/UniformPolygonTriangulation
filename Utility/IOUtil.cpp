#include "IOUtil.h"

#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkOBJReader.h>
#include <vtkOBJWriter.h>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkPolyDataReader.h>
#include <vtkBYUReader.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>


namespace Utility
{
    vtkSmartPointer<vtkPolyData> ReadPolyData(const char* fileDir)
    {
        vtkSmartPointer<vtkPolyData> polyData;
        auto extension = std::filesystem::path(fileDir).extension().string();

        if (extension == ".ply")
        {
            vtkNew<vtkPLYReader> reader;
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if (extension == ".vtp")
        {
            vtkNew<vtkXMLPolyDataReader> reader;
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if (extension == ".obj")
        {
            vtkNew<vtkOBJReader> reader;
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if (extension == ".stl")
        {
            vtkNew<vtkSTLReader> reader;
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if (extension == ".vtk")
        {
            vtkNew<vtkPolyDataReader> reader;
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if (extension == ".g")
        {
            vtkNew<vtkBYUReader> reader;
            reader->SetGeometryFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else
        {
            std::cout << "unsupported file extension" << std::endl;
        }
        return polyData;
    }

    void WritePolyData(const char* fileDir, vtkSmartPointer<vtkPolyData> polyData)
    {
        auto extension = std::filesystem::path(fileDir).extension().string();

        if (extension == ".ply")
        {
            vtkNew<vtkPLYWriter> writer;
            writer->SetFileName(fileDir);
            writer->SetInputData(polyData);
            writer->Write();
        }
        else if (extension == ".obj")
        {
            vtkNew<vtkOBJWriter> writer;
            writer->SetFileName(fileDir);
            writer->SetInputData(polyData);
            writer->Write();
        }
        else if (extension == ".stl")
        {
            vtkNew<vtkSTLWriter> writer;
            writer->SetFileName(fileDir);
            writer->SetInputData(polyData);
            writer->Write();
        }
        else
        {
            std::cout << "unsupported file extension" << std::endl;
        }
    }

    std::vector<vtkVector3d> ReadVectorFromFile(const char* dir)
    {
        std::vector<vtkVector3d> vector;
        std::ifstream inFile(dir);
        std::string line;

        while (std::getline(inFile, line))
        {
            if (line.empty())
                continue;

            std::istringstream is(line);
            std::vector<double> row((std::istream_iterator<double>(is)), std::istream_iterator<double>());
            vector.push_back(vtkVector3d(row[0], row[1], row[2]));
        }

        return vector;
    }

    void WriteVectorToFile(const std::vector<vtkVector3d>& vector, const char* dir)
    {
        std::ofstream fs;
        fs.open(dir);
        for (const auto& value: vector)
        {
            fs << value[0] << " " << value[1] << " " << value[2] << std::endl;
        }
        fs.close();
    }
}
