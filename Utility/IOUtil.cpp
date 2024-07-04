#include "IOUtil.h"

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
