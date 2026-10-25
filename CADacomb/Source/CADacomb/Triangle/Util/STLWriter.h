#pragma once

#include <glm/glm.hpp>
#include <iosfwd>
#include <string>
#include <vector>

namespace CADacomb
{
    // Utility for writing triangle mesh geometry to STL streams and files.
    class STLWriter
    {
      public:
        STLWriter();
        virtual ~STLWriter();

        static void write(const std::string& filename, const std::vector<glm::dvec3>& data);
        static void write(std::ostream& stream, const std::vector<glm::dvec3>& data);
        static void writeAscii(const std::string& filename, const std::vector<glm::dvec3>& data);
        static void writeAscii(std::ostream& stream, const std::vector<glm::dvec3>& data);
    };
}
