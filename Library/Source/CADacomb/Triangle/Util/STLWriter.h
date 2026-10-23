#pragma once

#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace Library
{
    class STLWriter
    {
      public:
        static void write(const std::string& filename, const std::vector<glm::dvec3>&);
    };
}