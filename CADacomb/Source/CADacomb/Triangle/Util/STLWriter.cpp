#include "CADacomb/Triangle/Util/STLWriter.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>

namespace CADacomb
{
    STLWriter::STLWriter() {}

    STLWriter::~STLWriter() {}

    void STLWriter::write(const std::string& filename, const std::vector<glm::dvec3>& data)
    {
        std::ofstream file(filename, std::ios::binary);
        if (file.is_open())
        {
            write(file, data);
        }
    }

    void STLWriter::write(std::ostream& stream, const std::vector<glm::dvec3>& data)
    {
        char header[80];
        std::memset(header, 0, sizeof(header));
        std::strncpy(header, "CADacomb Binary STL Export", sizeof(header) - 1);
        stream.write(header, sizeof(header));

        uint32_t triangleCount = static_cast<uint32_t>(data.size() / 3);
        stream.write(reinterpret_cast<const char*>(&triangleCount), sizeof(uint32_t));

        for (size_t i = 0; i < data.size() / 3; i++)
        {
            glm::dvec3 a      = data[i * 3 + 0];
            glm::dvec3 b      = data[i * 3 + 1];
            glm::dvec3 c      = data[i * 3 + 2];
            glm::dvec3 normal = glm::cross(b - a, c - a);
            double     length = glm::length(normal);
            if (length > 1e-12)
            {
                normal /= length;
            }
            else
            {
                normal = glm::dvec3(0.0);
            }

            float normalFloats[3] = { static_cast<float>(normal.x), static_cast<float>(normal.y), static_cast<float>(normal.z) };
            stream.write(reinterpret_cast<const char*>(normalFloats), sizeof(normalFloats));

            float v1Floats[3] = { static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z) };
            stream.write(reinterpret_cast<const char*>(v1Floats), sizeof(v1Floats));

            float v2Floats[3] = { static_cast<float>(b.x), static_cast<float>(b.y), static_cast<float>(b.z) };
            stream.write(reinterpret_cast<const char*>(v2Floats), sizeof(v2Floats));

            float v3Floats[3] = { static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z) };
            stream.write(reinterpret_cast<const char*>(v3Floats), sizeof(v3Floats));

            uint16_t attributeByteCount = 0;
            stream.write(reinterpret_cast<const char*>(&attributeByteCount), sizeof(uint16_t));
        }
    }

    void STLWriter::writeAscii(const std::string& filename, const std::vector<glm::dvec3>& data)
    {
        std::ofstream file(filename);
        if (file.is_open())
        {
            writeAscii(file, data);
        }
    }

    void STLWriter::writeAscii(std::ostream& stream, const std::vector<glm::dvec3>& data)
    {
        stream << "solid\n";
        stream << std::setprecision(10);

        for (size_t i = 0; i < data.size() / 3; i++)
        {
            glm::dvec3 a      = data[i * 3 + 0];
            glm::dvec3 b      = data[i * 3 + 1];
            glm::dvec3 c      = data[i * 3 + 2];
            glm::dvec3 normal = glm::cross(b - a, c - a);
            double     length = glm::length(normal);
            if (length > 1e-12)
            {
                normal /= length;
            }
            else
            {
                normal = glm::dvec3(0.0);
            }

            stream << "facet normal " << normal.x << " " << normal.y << " " << normal.z << "\n";
            stream << "    outer loop\n";
            stream << "        vertex " << a.x << " " << a.y << " " << a.z << "\n";
            stream << "        vertex " << b.x << " " << b.y << " " << b.z << "\n";
            stream << "        vertex " << c.x << " " << c.y << " " << c.z << "\n";
            stream << "    endloop\n";
            stream << "endfacet\n";
        }

        stream << "endsolid\n";
    }
}
