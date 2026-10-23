#pragma once

#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace CADacomb
{
    class STLWriter
    {
      public:
        static void write(const std::string& filename, const std::vector<glm::dvec3>&);
    };
}