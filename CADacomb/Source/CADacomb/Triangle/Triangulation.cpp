#include "CADacomb/Triangle/Triangulation.h"

#include "CADacomb/Triangle/Util/STLWriter.h"
#include "CADacomb/Triangle/Util/stl_reader.h"
#include "CADacomb/Util/Binary/base64.h"

#include <algorithm>
#include <fstream>
#include <limits>

namespace CADacomb
{
    Triangulation::Triangulation() {}
    Triangulation::~Triangulation() {}

    void Triangulation::saveAsSTL(const std::string& filename) const
    {
        std::vector<glm::dvec3> tri;
        tri.resize(indices.size());
        for (size_t i = 0; i < indices.size(); i++)
        {
            tri[i] = vertices[indices[i]];
        }
        STLWriter::write(filename, tri);
    }

    void Triangulation::saveAsSTL(std::ostream& stream) const
    {
        std::vector<glm::dvec3> tri;
        tri.resize(indices.size());
        for (size_t i = 0; i < indices.size(); i++)
        {
            tri[i] = vertices[indices[i]];
        }
        STLWriter::write(stream, tri);
    }

    std::unique_ptr<Triangulation> Triangulation::fromSTLFile(const std::string& filename)
    {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open())
        {
            return nullptr;
        }
        return fromSTLStream(file);
    }

    std::unique_ptr<Triangulation> Triangulation::fromSTLStream(std::istream& stream)
    {
        std::unique_ptr<Triangulation> result = std::make_unique<Triangulation>();

        std::vector<float>        coords;
        std::vector<float>        normals;
        std::vector<unsigned int> tris;
        std::vector<unsigned int> solidRanges;

        try
        {
            stl_reader::ReadStlStream(stream, coords, normals, tris, solidRanges);
        }
        catch (const std::exception& e)
        {
            return nullptr;
        }

        if (coords.empty() || tris.empty())
        {
            return nullptr;
        }

        result->vertices.reserve(coords.size() / 3);
        for (size_t i = 0; i < coords.size(); i += 3)
        {
            result->vertices.emplace_back(coords[i + 0], coords[i + 1], coords[i + 2]);
        }

        result->indices.reserve(tris.size());
        for (size_t i = 0; i < tris.size(); i++)
        {
            result->indices.push_back(static_cast<size_t>(tris[i]));
        }

        return result;
    }

    std::pair<glm::dvec3, glm::dvec3> Triangulation::getAABB() const
    {
        auto       inf = std::numeric_limits<double>::infinity();
        glm::dvec3 min = glm::dvec3(inf, inf, inf);
        glm::dvec3 max = glm::dvec3(-inf, -inf, -inf);

        for (const auto& x : vertices)
        {
            min.x = std::min(x.x, min.x);
            min.y = std::min(x.y, min.y);
            min.z = std::min(x.z, min.z);
            max.x = std::max(x.x, max.x);
            max.y = std::max(x.y, max.y);
            max.z = std::max(x.z, max.z);
        }
        glm::dvec3 position = glm::dvec3(min.x, min.y, min.z);
        glm::dvec3 size     = glm::dvec3(max.x - min.x, max.y - min.y, max.z - min.z);
        return std::make_pair(position, size);
    }

    std::map<std::pair<size_t, size_t>, std::vector<size_t>> Triangulation::getAdjacency() const
    {
        std::map<std::pair<size_t, size_t>, std::vector<size_t>> result;
        for (size_t i = 0; i < indices.size(); i += 3)
        {
            size_t t       = i / 3;
            auto   addEdge = [&](size_t a, size_t b)
            {
                if (a > b)
                    std::swap(a, b);
                result[{ a, b }].push_back(t);
            };
            addEdge(indices[i], indices[i + 1]);
            addEdge(indices[i + 1], indices[i + 2]);
            addEdge(indices[i + 2], indices[i]);
        }
        return result;
    }

    glm::dvec3 Triangulation::getFaceNormal(size_t faceIndex) const
    {
        size_t idx0 = indices[faceIndex * 3 + 0];
        size_t idx1 = indices[faceIndex * 3 + 1];
        size_t idx2 = indices[faceIndex * 3 + 2];

        const glm::dvec3& p0 = vertices[idx0];
        const glm::dvec3& p1 = vertices[idx1];
        const glm::dvec3& p2 = vertices[idx2];

        glm::dvec3 edge1  = p1 - p0;
        glm::dvec3 edge2  = p2 - p0;
        glm::dvec3 normal = glm::cross(edge1, edge2);

        double length = glm::length(normal);
        if (length > 1e-12)
        {
            return normal / length;
        }

        return glm::dvec3(0.0, 0.0, 0.0);
    }

    std::string Triangulation::toBase64() const
    {
        std::vector<unsigned char> buffer;

        size_t vSize = vertices.size();
        size_t iSize = indices.size();

        auto append = [&](const auto& data)
        {
            const unsigned char* ptr = reinterpret_cast<const unsigned char*>(&data);
            buffer.insert(buffer.end(), ptr, ptr + sizeof(data));
        };

        append(vSize);
        append(iSize);

        const unsigned char* vPtr = reinterpret_cast<const unsigned char*>(vertices.data());
        buffer.insert(buffer.end(), vPtr, vPtr + (vSize * sizeof(glm::dvec3)));

        const unsigned char* iPtr = reinterpret_cast<const unsigned char*>(indices.data());
        buffer.insert(buffer.end(), iPtr, iPtr + (iSize * sizeof(size_t)));

        return base64::base64_encode(buffer.data(), buffer.size());
    }

    Triangulation Triangulation::fromBase64(const std::string& base64Str)
    {
        auto                 buffer = base64::base64_decode(base64Str);
        const unsigned char* ptr    = buffer.data();

        Triangulation tri;

        size_t vSize = *reinterpret_cast<const size_t*>(ptr);
        ptr += sizeof(size_t);
        size_t iSize = *reinterpret_cast<const size_t*>(ptr);
        ptr += sizeof(size_t);

        tri.vertices.resize(vSize);
        std::copy(ptr, ptr + (vSize * sizeof(glm::dvec3)), reinterpret_cast<unsigned char*>(tri.vertices.data()));
        ptr += (vSize * sizeof(glm::dvec3));

        tri.indices.resize(iSize);
        std::copy(ptr, ptr + (iSize * sizeof(size_t)), reinterpret_cast<unsigned char*>(tri.indices.data()));

        return tri;
    }
}
