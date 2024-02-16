#include "IOUtil.h"

#include <vtkPolyData.h>
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
        const auto path = std::filesystem::path(fileDir);
        if(!std::filesystem::exists(path))
        {
            std::cerr << "ERROR: Directory " << path.string() << " does not exist!" << std::endl;
            return false;
        }

        return true;
    }

    bool IsValidDirectory(const char* fileDir)
    {
        const auto path = std::filesystem::path(fileDir);
        const auto parentDir = path.parent_path();
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
            throw std::runtime_error("ReadPolyData(). Path does not exist!");

        const auto path = std::filesystem::path(fileDir);
        const auto extension = std::filesystem::path(fileDir).extension();
        std::cout << "Reading " << path << std::endl;

        vtkSmartPointer<vtkPolyData> polyData;
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
            throw std::runtime_error("ReadPolyData(). ERROR: unsupported file extension!");
        }

        printf("Poly data vertices cnt: %lld, cells cnt: %lld\n", polyData->GetNumberOfPoints(), polyData->GetNumberOfCells());

        return polyData;
    }

    void WritePolyData(const char* fileDir, vtkPolyData* polyData)
    {
        if(!IsValidDirectory(fileDir))
            return;

        const auto path = std::filesystem::path(fileDir);
        const auto extension = path.extension();

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
            return;
        }

        std::cout << "Poly data is written to " << path << std::endl;
    }

    void WriteColorPolyData(const char* fileDir, vtkPolyData* polyData)
    {
        if(!IsValidDirectory(fileDir))
            return;

        const auto path = std::filesystem::path(fileDir);
        const auto extension = path.extension();
        if(extension != ".ply")
        {
            std::cerr << "ERROR: wrong extension. Should be .ply" << std::endl;
            return;
        }

        auto plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
        plyWriter->SetFileName(fileDir);
        plyWriter->SetInputData(polyData);
        plyWriter->SetArrayName("Colors"); //set colors
        plyWriter->Write();

        std::cout << "Saved colored polydata to " << path << std::endl;
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
