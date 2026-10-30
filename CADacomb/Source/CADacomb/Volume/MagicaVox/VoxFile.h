#pragma once

#include <array>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace MagicaVoxImporter
{
    class VoxFile
    {
      public:
        VoxFile(const std::string& filename);
        virtual ~VoxFile();

        static std::pair<std::vector<bool>, std::array<size_t, 3>> readBinary(const std::string& filename);
        static void                                                writeBinary(const std::vector<bool>& data, const std::array<size_t, 3>& size, const std::string& filename);

        std::vector<std::pair<std::vector<unsigned char>, std::array<size_t, 3>>> models;
        std::array<std::array<unsigned char, 4>, 256>                             colors;
        std::vector<std::map<std::string, std::string>>                           materials;
    };
}