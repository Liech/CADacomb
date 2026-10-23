#include "SurfaceNets.h"

#include <algorithm>
#include <cmath>

namespace Library
{
    SurfaceNets::SurfaceNets() {}

    SurfaceNets::~SurfaceNets() {}

    std::unique_ptr<Triangulation> SurfaceNets::triangulate(const BinaryVolume& volume)
    {
        if (volume.dimension.x < 2 || volume.dimension.y < 2 || volume.dimension.z < 2)
            return std::make_unique<Triangulation>();

        glm::dvec3 voxelSize = volume.size / glm::dvec3(volume.dimension);
        return triangulate(volume.data, volume.origin, voxelSize, volume.dimension);
    }

    std::unique_ptr<Triangulation> SurfaceNets::triangulate(const std::vector<bool>& data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution)
    {
        if (resolution.x < 2 || resolution.y < 2 || resolution.z < 2 || data.size() < (size_t)resolution.x * (size_t)resolution.y * (size_t)resolution.z)
        {
            return std::make_unique<Triangulation>();
        }

        glm::u64vec3 cellDim(resolution.x - 1, resolution.y - 1, resolution.z - 1);
        size_t       totalCells = (size_t)cellDim.x * (size_t)cellDim.y * (size_t)cellDim.z;

        std::vector<int> cellVertexIndex(totalCells, -1);

        auto cellIndex = [&](size_t cx, size_t cy, size_t cz) -> size_t { return cx + cy * (size_t)cellDim.x + cz * (size_t)cellDim.x * (size_t)cellDim.y; };

        auto sampleIndex = [&](size_t x, size_t y, size_t z) -> size_t { return x + y * (size_t)resolution.x + z * (size_t)resolution.x * (size_t)resolution.y; };

        auto samplePos = [&](size_t x, size_t y, size_t z) -> glm::dvec3 { return origin + glm::dvec3((double)x * voxelSize.x, (double)y * voxelSize.y, (double)z * voxelSize.z); };

        auto result = std::make_unique<Triangulation>();

        static const int edges[12][2] = {
            { 0, 1 },
            { 2, 3 },
            { 4, 5 },
            { 6, 7 }, // X
            { 0, 2 },
            { 1, 3 },
            { 4, 6 },
            { 5, 7 }, // Y
            { 0, 4 },
            { 1, 5 },
            { 2, 6 },
            { 3, 7 }  // Z
        };

        // Pass 1: Generate 1 vertex per active cell (average of edge crossings)
        for (size_t cz = 0; cz < cellDim.z; ++cz)
        {
            for (size_t cy = 0; cy < cellDim.y; ++cy)
            {
                for (size_t cx = 0; cx < cellDim.x; ++cx)
                {
                    bool s[8] = { data[sampleIndex(cx, cy, cz)],     data[sampleIndex(cx + 1, cy, cz)],     data[sampleIndex(cx, cy + 1, cz)],     data[sampleIndex(cx + 1, cy + 1, cz)],
                                  data[sampleIndex(cx, cy, cz + 1)], data[sampleIndex(cx + 1, cy, cz + 1)], data[sampleIndex(cx, cy + 1, cz + 1)], data[sampleIndex(cx + 1, cy + 1, cz + 1)] };

                    bool allSame = true;
                    for (int i = 1; i < 8; ++i)
                    {
                        if (s[i] != s[0])
                        {
                            allSame = false;
                            break;
                        }
                    }
                    if (allSame)
                        continue;

                    glm::dvec3 sumPos(0.0);
                    int        count = 0;

                    for (int e = 0; e < 12; ++e)
                    {
                        int cA = edges[e][0];
                        int cB = edges[e][1];
                        if (s[cA] != s[cB])
                        {
                            size_t dxA = (cA & 1) ? 1 : 0;
                            size_t dyA = (cA & 2) ? 1 : 0;
                            size_t dzA = (cA & 4) ? 1 : 0;
                            size_t dxB = (cB & 1) ? 1 : 0;
                            size_t dyB = (cB & 2) ? 1 : 0;
                            size_t dzB = (cB & 4) ? 1 : 0;

                            glm::dvec3 pA = samplePos(cx + dxA, cy + dyA, cz + dzA);
                            glm::dvec3 pB = samplePos(cx + dxB, cy + dyB, cz + dzB);

                            sumPos += 0.5 * (pA + pB);
                            count++;
                        }
                    }

                    if (count > 0)
                    {
                        int vertIdx = static_cast<int>(result->vertices.size());
                        result->vertices.push_back(sumPos / (double)count);
                        cellVertexIndex[cellIndex(cx, cy, cz)] = vertIdx;
                    }
                }
            }
        }

        // Pass 2: Generate quads across internal crossing edges
        // X-edges: from (x, y, z) to (x+1, y, z)
        for (size_t z = 1; z < cellDim.z; ++z)
        {
            for (size_t y = 1; y < cellDim.y; ++y)
            {
                for (size_t x = 0; x < cellDim.x; ++x)
                {
                    bool s0 = data[sampleIndex(x, y, z)];
                    bool s1 = data[sampleIndex(x + 1, y, z)];
                    if (s0 == s1)
                        continue;

                    int iA = cellVertexIndex[cellIndex(x, y - 1, z - 1)];
                    int iB = cellVertexIndex[cellIndex(x, y, z - 1)];
                    int iC = cellVertexIndex[cellIndex(x, y, z)];
                    int iD = cellVertexIndex[cellIndex(x, y - 1, z)];

                    if (iA >= 0 && iB >= 0 && iC >= 0 && iD >= 0)
                    {
                        if (s0)
                        {
                            result->indices.push_back(iA);
                            result->indices.push_back(iB);
                            result->indices.push_back(iC);

                            result->indices.push_back(iA);
                            result->indices.push_back(iC);
                            result->indices.push_back(iD);
                        }
                        else
                        {
                            result->indices.push_back(iA);
                            result->indices.push_back(iD);
                            result->indices.push_back(iC);

                            result->indices.push_back(iA);
                            result->indices.push_back(iC);
                            result->indices.push_back(iB);
                        }
                    }
                }
            }
        }

        // Y-edges: from (x, y, z) to (x, y+1, z)
        for (size_t z = 1; z < cellDim.z; ++z)
        {
            for (size_t y = 0; y < cellDim.y; ++y)
            {
                for (size_t x = 1; x < cellDim.x; ++x)
                {
                    bool s0 = data[sampleIndex(x, y, z)];
                    bool s1 = data[sampleIndex(x, y + 1, z)];
                    if (s0 == s1)
                        continue;

                    int iA = cellVertexIndex[cellIndex(x - 1, y, z - 1)];
                    int iB = cellVertexIndex[cellIndex(x - 1, y, z)];
                    int iC = cellVertexIndex[cellIndex(x, y, z)];
                    int iD = cellVertexIndex[cellIndex(x, y, z - 1)];

                    if (iA >= 0 && iB >= 0 && iC >= 0 && iD >= 0)
                    {
                        if (s0)
                        {
                            result->indices.push_back(iA);
                            result->indices.push_back(iB);
                            result->indices.push_back(iC);

                            result->indices.push_back(iA);
                            result->indices.push_back(iC);
                            result->indices.push_back(iD);
                        }
                        else
                        {
                            result->indices.push_back(iA);
                            result->indices.push_back(iD);
                            result->indices.push_back(iC);

                            result->indices.push_back(iA);
                            result->indices.push_back(iC);
                            result->indices.push_back(iB);
                        }
                    }
                }
            }
        }

        // Z-edges: from (x, y, z) to (x, y, z+1)
        for (size_t z = 0; z < cellDim.z; ++z)
        {
            for (size_t y = 1; y < cellDim.y; ++y)
            {
                for (size_t x = 1; x < cellDim.x; ++x)
                {
                    bool s0 = data[sampleIndex(x, y, z)];
                    bool s1 = data[sampleIndex(x, y, z + 1)];
                    if (s0 == s1)
                        continue;

                    int iA = cellVertexIndex[cellIndex(x - 1, y - 1, z)];
                    int iB = cellVertexIndex[cellIndex(x, y - 1, z)];
                    int iC = cellVertexIndex[cellIndex(x, y, z)];
                    int iD = cellVertexIndex[cellIndex(x - 1, y, z)];

                    if (iA >= 0 && iB >= 0 && iC >= 0 && iD >= 0)
                    {
                        if (s0)
                        {
                            result->indices.push_back(iA);
                            result->indices.push_back(iB);
                            result->indices.push_back(iC);

                            result->indices.push_back(iA);
                            result->indices.push_back(iC);
                            result->indices.push_back(iD);
                        }
                        else
                        {
                            result->indices.push_back(iA);
                            result->indices.push_back(iD);
                            result->indices.push_back(iC);

                            result->indices.push_back(iA);
                            result->indices.push_back(iC);
                            result->indices.push_back(iB);
                        }
                    }
                }
            }
        }

        return result;
    }

    std::unique_ptr<Triangulation> SurfaceNets::triangulate(const bool* data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution)
    {
        size_t            total = (size_t)resolution.x * (size_t)resolution.y * (size_t)resolution.z;
        std::vector<bool> vec(data, data + total);
        return triangulate(vec, origin, voxelSize, resolution);
    }

    std::unique_ptr<Triangulation> SurfaceNets::triangulate(const std::vector<double>& data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution, double isovalue)
    {
        return triangulate(data.data(), origin, voxelSize, resolution, isovalue);
    }

    std::unique_ptr<Triangulation> SurfaceNets::triangulate(const double* data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution, double isovalue)
    {
        if (resolution.x < 2 || resolution.y < 2 || resolution.z < 2)
            return std::make_unique<Triangulation>();

        glm::u64vec3 cellDim(resolution.x - 1, resolution.y - 1, resolution.z - 1);
        size_t       totalCells = (size_t)cellDim.x * (size_t)cellDim.y * (size_t)cellDim.z;

        std::vector<int> cellVertexIndex(totalCells, -1);

        auto cellIndex = [&](size_t cx, size_t cy, size_t cz) -> size_t { return cx + cy * (size_t)cellDim.x + cz * (size_t)cellDim.x * (size_t)cellDim.y; };

        auto sampleIndex = [&](size_t x, size_t y, size_t z) -> size_t { return x + y * (size_t)resolution.x + z * (size_t)resolution.x * (size_t)resolution.y; };

        auto samplePos = [&](size_t x, size_t y, size_t z) -> glm::dvec3 { return origin + glm::dvec3((double)x * voxelSize.x, (double)y * voxelSize.y, (double)z * voxelSize.z); };

        auto result = std::make_unique<Triangulation>();

        static const int edges[12][2] = {
            { 0, 1 },
            { 2, 3 },
            { 4, 5 },
            { 6, 7 }, // X
            { 0, 2 },
            { 1, 3 },
            { 4, 6 },
            { 5, 7 }, // Y
            { 0, 4 },
            { 1, 5 },
            { 2, 6 },
            { 3, 7 }  // Z
        };

        // Pass 1: Generate 1 vertex per active cell (interpolated on crossing edges)
        for (size_t cz = 0; cz < cellDim.z; ++cz)
        {
            for (size_t cy = 0; cy < cellDim.y; ++cy)
            {
                for (size_t cx = 0; cx < cellDim.x; ++cx)
                {
                    double v[8] = { data[sampleIndex(cx, cy, cz)],     data[sampleIndex(cx + 1, cy, cz)],     data[sampleIndex(cx, cy + 1, cz)],     data[sampleIndex(cx + 1, cy + 1, cz)],
                                    data[sampleIndex(cx, cy, cz + 1)], data[sampleIndex(cx + 1, cy, cz + 1)], data[sampleIndex(cx, cy + 1, cz + 1)], data[sampleIndex(cx + 1, cy + 1, cz + 1)] };

                    bool s[8];
                    for (int i = 0; i < 8; ++i)
                        s[i] = (v[i] >= isovalue);

                    bool allSame = true;
                    for (int i = 1; i < 8; ++i)
                    {
                        if (s[i] != s[0])
                        {
                            allSame = false;
                            break;
                        }
                    }
                    if (allSame)
                        continue;

                    glm::dvec3 sumPos(0.0);
                    int        count = 0;

                    for (int e = 0; e < 12; ++e)
                    {
                        int cA = edges[e][0];
                        int cB = edges[e][1];
                        if (s[cA] != s[cB])
                        {
                            size_t dxA = (cA & 1) ? 1 : 0;
                            size_t dyA = (cA & 2) ? 1 : 0;
                            size_t dzA = (cA & 4) ? 1 : 0;
                            size_t dxB = (cB & 1) ? 1 : 0;
                            size_t dyB = (cB & 2) ? 1 : 0;
                            size_t dzB = (cB & 4) ? 1 : 0;

                            glm::dvec3 pA = samplePos(cx + dxA, cy + dyA, cz + dzA);
                            glm::dvec3 pB = samplePos(cx + dxB, cy + dyB, cz + dzB);

                            double valA  = v[cA];
                            double valB  = v[cB];
                            double denom = valB - valA;
                            double t     = 0.5;
                            if (std::abs(denom) > 1e-12)
                            {
                                t = (isovalue - valA) / denom;
                                t = std::clamp(t, 0.0, 1.0);
                            }

                            sumPos += pA + t * (pB - pA);
                            count++;
                        }
                    }

                    if (count > 0)
                    {
                        int vertIdx = static_cast<int>(result->vertices.size());
                        result->vertices.push_back(sumPos / (double)count);
                        cellVertexIndex[cellIndex(cx, cy, cz)] = vertIdx;
                    }
                }
            }
        }

        // Pass 2: Quads
        // X-edges
        for (size_t z = 1; z < cellDim.z; ++z)
        {
            for (size_t y = 1; y < cellDim.y; ++y)
            {
                for (size_t x = 0; x < cellDim.x; ++x)
                {
                    bool s0 = (data[sampleIndex(x, y, z)] >= isovalue);
                    bool s1 = (data[sampleIndex(x + 1, y, z)] >= isovalue);
                    if (s0 == s1)
                        continue;

                    int iA = cellVertexIndex[cellIndex(x, y - 1, z - 1)];
                    int iB = cellVertexIndex[cellIndex(x, y, z - 1)];
                    int iC = cellVertexIndex[cellIndex(x, y, z)];
                    int iD = cellVertexIndex[cellIndex(x, y - 1, z)];

                    if (iA >= 0 && iB >= 0 && iC >= 0 && iD >= 0)
                    {
                        if (s0)
                        {
                            result->indices.push_back(iA);
                            result->indices.push_back(iB);
                            result->indices.push_back(iC);

                            result->indices.push_back(iA);
                            result->indices.push_back(iC);
                            result->indices.push_back(iD);
                        }
                        else
                        {
                            result->indices.push_back(iA);
                            result->indices.push_back(iD);
                            result->indices.push_back(iC);

                            result->indices.push_back(iA);
                            result->indices.push_back(iC);
                            result->indices.push_back(iB);
                        }
                    }
                }
            }
        }

        // Y-edges
        for (size_t z = 1; z < cellDim.z; ++z)
        {
            for (size_t y = 0; y < cellDim.y; ++y)
            {
                for (size_t x = 1; x < cellDim.x; ++x)
                {
                    bool s0 = (data[sampleIndex(x, y, z)] >= isovalue);
                    bool s1 = (data[sampleIndex(x, y + 1, z)] >= isovalue);
                    if (s0 == s1)
                        continue;

                    int iA = cellVertexIndex[cellIndex(x - 1, y, z - 1)];
                    int iB = cellVertexIndex[cellIndex(x - 1, y, z)];
                    int iC = cellVertexIndex[cellIndex(x, y, z)];
                    int iD = cellVertexIndex[cellIndex(x, y, z - 1)];

                    if (iA >= 0 && iB >= 0 && iC >= 0 && iD >= 0)
                    {
                        if (s0)
                        {
                            result->indices.push_back(iA);
                            result->indices.push_back(iB);
                            result->indices.push_back(iC);

                            result->indices.push_back(iA);
                            result->indices.push_back(iC);
                            result->indices.push_back(iD);
                        }
                        else
                        {
                            result->indices.push_back(iA);
                            result->indices.push_back(iD);
                            result->indices.push_back(iC);

                            result->indices.push_back(iA);
                            result->indices.push_back(iC);
                            result->indices.push_back(iB);
                        }
                    }
                }
            }
        }

        // Z-edges
        for (size_t z = 0; z < cellDim.z; ++z)
        {
            for (size_t y = 1; y < cellDim.y; ++y)
            {
                for (size_t x = 1; x < cellDim.x; ++x)
                {
                    bool s0 = (data[sampleIndex(x, y, z)] >= isovalue);
                    bool s1 = (data[sampleIndex(x, y, z + 1)] >= isovalue);
                    if (s0 == s1)
                        continue;

                    int iA = cellVertexIndex[cellIndex(x - 1, y - 1, z)];
                    int iB = cellVertexIndex[cellIndex(x, y - 1, z)];
                    int iC = cellVertexIndex[cellIndex(x, y, z)];
                    int iD = cellVertexIndex[cellIndex(x - 1, y, z)];

                    if (iA >= 0 && iB >= 0 && iC >= 0 && iD >= 0)
                    {
                        if (s0)
                        {
                            result->indices.push_back(iA);
                            result->indices.push_back(iB);
                            result->indices.push_back(iC);

                            result->indices.push_back(iA);
                            result->indices.push_back(iC);
                            result->indices.push_back(iD);
                        }
                        else
                        {
                            result->indices.push_back(iA);
                            result->indices.push_back(iD);
                            result->indices.push_back(iC);

                            result->indices.push_back(iA);
                            result->indices.push_back(iC);
                            result->indices.push_back(iB);
                        }
                    }
                }
            }
        }

        return result;
    }

    std::vector<glm::dvec3> SurfaceNets::polygonize(const std::vector<bool>& data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution)
    {
        auto                    tri = triangulate(data, origin, voxelSize, resolution);
        std::vector<glm::dvec3> unindexed;
        unindexed.reserve(tri->indices.size());
        for (size_t idx : tri->indices)
        {
            unindexed.push_back(tri->vertices[idx]);
        }
        return unindexed;
    }

    std::vector<glm::dvec3> SurfaceNets::polygonize(const bool* data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution)
    {
        auto                    tri = triangulate(data, origin, voxelSize, resolution);
        std::vector<glm::dvec3> unindexed;
        unindexed.reserve(tri->indices.size());
        for (size_t idx : tri->indices)
        {
            unindexed.push_back(tri->vertices[idx]);
        }
        return unindexed;
    }

    std::vector<glm::dvec3> SurfaceNets::polygonize(const std::vector<double>& data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution, double isovalue)
    {
        auto                    tri = triangulate(data, origin, voxelSize, resolution, isovalue);
        std::vector<glm::dvec3> unindexed;
        unindexed.reserve(tri->indices.size());
        for (size_t idx : tri->indices)
        {
            unindexed.push_back(tri->vertices[idx]);
        }
        return unindexed;
    }

    std::vector<glm::dvec3> SurfaceNets::polygonize(const double* data, const glm::dvec3& origin, const glm::dvec3& voxelSize, const glm::u64vec3& resolution, double isovalue)
    {
        auto                    tri = triangulate(data, origin, voxelSize, resolution, isovalue);
        std::vector<glm::dvec3> unindexed;
        unindexed.reserve(tri->indices.size());
        for (size_t idx : tri->indices)
        {
            unindexed.push_back(tri->vertices[idx]);
        }
        return unindexed;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/Triangle/BaseShapeGenerator.h"
#include "CADacomb/Volume/Flat/Voxelizer.h"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("SurfaceNets/CubeWithHole")
{
    float              radius = 0.3f;
    auto               tri    = Library::BaseShapeGenerator::cubeWithHole(32, radius);
    Library::Voxelizer voxelizer;
    glm::dvec3         start(0.0, 0.0, 0.0);
    glm::dvec3         end(1.0, 1.0, 1.0);
    glm::u64vec3       resolution(16, 16, 16);

    auto volume = voxelizer.voxelize(tri.vertices, tri.indices, start, end, resolution);
    REQUIRE(volume != nullptr);

    auto snTri = Library::SurfaceNets::triangulate(*volume);
    REQUIRE(snTri != nullptr);
    CHECK(snTri->vertices.size() > 0);
    CHECK(snTri->indices.size() > 0);
    CHECK(snTri->indices.size() % 3 == 0);

    for (size_t idx : snTri->indices)
    {
        CHECK(idx < snTri->vertices.size());
    }

    glm::dvec3 voxelSize = volume->size / glm::dvec3(volume->dimension);
    auto       unindexed = Library::SurfaceNets::polygonize(volume->data, volume->origin, voxelSize, volume->dimension);
    CHECK(unindexed.size() == snTri->indices.size());
}

TEST_CASE("SurfaceNets/SingleVoxelCube")
{
    glm::u64vec3      resolution(4, 4, 4);
    std::vector<bool> data(4 * 4 * 4, false);
    data[1 + 1 * 4 + 1 * 16] = true;

    glm::dvec3 origin(0.0);
    glm::dvec3 voxelSize(1.0);

    auto tri = Library::SurfaceNets::triangulate(data, origin, voxelSize, resolution);
    REQUIRE(tri != nullptr);
    CHECK(tri->vertices.size() > 0);
    CHECK(tri->indices.size() > 0);
    CHECK(tri->indices.size() % 6 == 0);
}
#endif
