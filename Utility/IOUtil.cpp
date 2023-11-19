#include "IOUtil.h"

#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkOBJReader.h>
#include <vtkOBJWriter.h>
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkPolyDataReader.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>


namespace Utility
{
    bool PathExist(const char* fileDir)
    {
        auto path = std::filesystem::path(fileDir);
        if(!std::filesystem::exists(path))
        {
            std::cerr << "ERROR: Directory " << path.string() << " does not exist!" << std::endl;
            return false;
        }

        return true;
    }

    bool IsValidDirectory(const char* fileDir)
    {
        auto path = std::filesystem::path(fileDir);
        auto parentDir = path.parent_path();
        if(!std::filesystem::is_directory(parentDir))
        {
            std::cerr << "ERROR: Directory " << parentDir.string() << " does not exist!" << std::endl;
            return false;
        }

        return true;
    }

    vtkSmartPointer<vtkPolyData> ReadPolyData(const char* fileDir)
    {
        if(!PathExist(fileDir))
            return vtkSmartPointer<vtkPolyData>::New();

        vtkSmartPointer<vtkPolyData> polyData;
        auto extension = std::filesystem::path(fileDir).extension().string();

        if(extension == ".ply")
        {
            auto reader = vtkSmartPointer<vtkPLYReader>::New();
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if(extension == ".obj")
        {
            auto reader = vtkSmartPointer<vtkOBJReader>::New();
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if(extension == ".stl")
        {
            auto reader = vtkSmartPointer<vtkSTLReader>::New();
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if(extension == ".vtk")
        {
            auto reader = vtkSmartPointer<vtkPolyDataReader>::New();
            reader->SetFileName(fileDir);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else
        {
            std::cerr << "ERROR: unsupported file extension" << std::endl;
        }
        return polyData;
    }

    void WritePolyData(const char* fileDir, vtkSmartPointer<vtkPolyData> polyData)
    {
        if(!IsValidDirectory(fileDir))
            return;

        auto extension = std::filesystem::path(fileDir).extension().string();

        if(extension == ".ply")
        {
            auto writer = vtkSmartPointer<vtkPLYWriter>::New();
            writer->SetFileName(fileDir);
            writer->SetInputData(polyData);
            writer->Write();
        }
        else if(extension == ".obj")
        {
            auto writer = vtkSmartPointer<vtkOBJWriter>::New();
            writer->SetFileName(fileDir);
            writer->SetInputData(polyData);
            writer->Write();
        }
        else if(extension == ".stl")
        {
            auto writer = vtkSmartPointer<vtkSTLWriter>::New();
            writer->SetFileName(fileDir);
            writer->SetInputData(polyData);
            writer->Write();
        }
        else
        {
            std::cerr << "ERROR: unsupported file extension" << std::endl;
        }
    }

    std::vector<vtkVector3d> ReadVectorFromFile(const char* fileDir)
    {
        if(!PathExist(fileDir))
            return {};

        std::vector<vtkVector3d> vector;
        std::ifstream inFile(fileDir);
        std::string line;

        while(std::getline(inFile, line))
        {
            if(line.empty())
                continue;

            std::istringstream is(line);
            std::vector<double> row((std::istream_iterator<double>(is)), std::istream_iterator<double>());
            vector.push_back(vtkVector3d(row[0], row[1], row[2]));
        }

        return vector;
    }

    void WriteVectorToFile(const std::vector<vtkVector3d>& vector, const char* fileDir)
    {
        if(!IsValidDirectory(fileDir))
            return;

        std::ofstream fs;
        fs.open(fileDir);
        for(const auto& value: vector)
        {
            fs << value[0] << " " << value[1] << " " << value[2] << std::endl;
        }
        fs.close();
    }
}
