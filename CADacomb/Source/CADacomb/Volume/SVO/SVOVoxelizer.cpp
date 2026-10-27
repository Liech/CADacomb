#include "SVOVoxelizer.h"

#include <algorithm>
#include <cmath>

namespace CADacomb
{
    SVOVoxelizer::SVOVoxelizer() {}

    SVOVoxelizer::~SVOVoxelizer() {}

    void SVOVoxelizer::voxelizeBlocks(const Triangulation&                                                                                                   triangulation,
                                      const glm::dvec3&                                                                                                      start,
                                      const glm::dvec3&                                                                                                      end,
                                      uint32_t                                                                                                               dimension,
                                      uint32_t                                                                                                               blockSize,
                                      const std::function<void(const glm::ivec3& blockOffset, bool isSolidHomogeneous, const std::vector<bool>& blockData)>& onBlock) const
    {
        const auto& vertices = triangulation.vertices;
        const auto& indices  = triangulation.indices;
        glm::dvec3  span     = end - start;

        uint32_t numTilesX = (dimension + blockSize - 1) / blockSize;
        uint32_t numTilesY = (dimension + blockSize - 1) / blockSize;
        uint32_t numTilesZ = (dimension + blockSize - 1) / blockSize;

        std::vector<std::vector<size_t>> tileTriangleStack(numTilesX * numTilesY);
        spatialHashTrianglesToTiles(vertices, indices, start, end, numTilesX, numTilesY, tileTriangleStack);

        glm::dvec3        rayDirection(0.0, 0.0, 1.0);
        std::vector<bool> emptyVector;

#pragma omp parallel for schedule(dynamic)
        for (long long tileIdx = 0; tileIdx < static_cast<long long>(numTilesX * numTilesY); tileIdx++)
        {
            size_t      tileX         = static_cast<size_t>(tileIdx % numTilesX);
            size_t      tileY         = static_cast<size_t>(tileIdx / numTilesX);
            const auto& tileTriangles = tileTriangleStack[tileIdx];

            if (tileTriangles.empty())
            {
                continue;
            }

            double zMin = start.z;
            for (size_t tri : tileTriangles)
            {
                zMin = std::min(zMin, vertices[indices[tri + 0]].z);
                zMin = std::min(zMin, vertices[indices[tri + 1]].z);
                zMin = std::min(zMin, vertices[indices[tri + 2]].z);
            }

            std::vector<std::vector<int64_t>> rayZPositions(blockSize * blockSize);
            std::vector<double>               intersections;

            for (size_t ly = 0; ly < blockSize; ly++)
            {
                for (size_t lx = 0; lx < blockSize; lx++)
                {
                    size_t globalX = tileX * blockSize + lx;
                    size_t globalY = tileY * blockSize + ly;

                    if (globalX >= dimension || globalY >= dimension)
                    {
                        continue;
                    }

                    double     rx = (static_cast<double>(globalX) + 0.5) / static_cast<double>(dimension);
                    double     ry = (static_cast<double>(globalY) + 0.5) / static_cast<double>(dimension);
                    glm::dvec3 rayStart(start.x + span.x * rx, start.y + span.y * ry, zMin - 10.0);

                    intersections.clear();
                    for (size_t tri : tileTriangles)
                    {
                        const glm::dvec3& a = vertices[indices[tri]];
                        const glm::dvec3& b = vertices[indices[tri + 1]];
                        const glm::dvec3& c = vertices[indices[tri + 2]];

                        double minX = (a.x < b.x) ? ((a.x < c.x) ? a.x : c.x) : ((b.x < c.x) ? b.x : c.x);
                        double maxX = (a.x > b.x) ? ((a.x > c.x) ? a.x : c.x) : ((b.x > c.x) ? b.x : c.x);
                        if (rayStart.x < minX - 1e-5 || rayStart.x > maxX + 1e-5)
                        {
                            continue;
                        }

                        double minY = (a.y < b.y) ? ((a.y < c.y) ? a.y : c.y) : ((b.y < c.y) ? b.y : c.y);
                        double maxY = (a.y > b.y) ? ((a.y > c.y) ? a.y : c.y) : ((b.y > c.y) ? b.y : c.y);
                        if (rayStart.y < minY - 1e-5 || rayStart.y > maxY + 1e-5)
                        {
                            continue;
                        }

                        glm::dvec3 intersectionLocation;
                        if (intersectRayTriangle(rayStart, rayDirection, a, b, c, intersectionLocation))
                        {
                            intersections.push_back(intersectionLocation.z);
                        }
                    }

                    std::sort(intersections.begin(), intersections.end());
                    intersections.erase(std::unique(intersections.begin(), intersections.end(), [this](double v1, double v2) { return epsilonEqual(v1, v2); }), intersections.end());

                    size_t rayIdx = lx + ly * blockSize;
                    rayZPositions[rayIdx].reserve(intersections.size());

                    for (double zIntersection : intersections)
                    {
                        long long zPos = static_cast<long long>(std::round(((zIntersection - start.z) / span.z) * static_cast<double>(dimension)));
                        zPos           = std::max(0LL, zPos);
                        zPos           = std::min(static_cast<long long>(dimension), zPos);
                        rayZPositions[rayIdx].push_back(zPos);
                    }
                }
            }

            size_t            sliceSize = blockSize * blockSize;
            std::vector<bool> blockData(blockSize * blockSize * blockSize, false);

            for (uint32_t bz = 0; bz < numTilesZ; bz++)
            {
                int64_t blockZStart = static_cast<int64_t>(bz * blockSize);
                int64_t blockZEnd   = blockZStart + static_cast<int64_t>(blockSize);

                bool hasInternalFlips = false;
                bool allEmpty         = true;
                bool allSolid         = true;

                for (size_t rayIdx = 0; rayIdx < blockSize * blockSize; rayIdx++)
                {
                    const auto& zList = rayZPositions[rayIdx];

                    size_t countBefore = 0;
                    while (countBefore < zList.size() && zList[countBefore] < blockZStart)
                    {
                        countBefore++;
                    }
                    bool entryParity = (countBefore % 2 == 1);

                    if (countBefore < zList.size() && zList[countBefore] < blockZEnd)
                    {
                        hasInternalFlips = true;
                    }

                    if (entryParity)
                    {
                        allEmpty = false;
                    }
                    else
                    {
                        allSolid = false;
                    }
                }

                glm::ivec3 blockOffset(static_cast<int>(tileX * blockSize), static_cast<int>(tileY * blockSize), static_cast<int>(bz * blockSize));

                if (!hasInternalFlips && allEmpty)
                {
                    continue;
                }

                if (!hasInternalFlips && allSolid)
                {
#pragma omp critical
                    onBlock(blockOffset, true, emptyVector);
                    continue;
                }

                std::fill(blockData.begin(), blockData.end(), false);

                for (size_t ly = 0; ly < blockSize; ly++)
                {
                    for (size_t lx = 0; lx < blockSize; lx++)
                    {
                        size_t      rayIdx = lx + ly * blockSize;
                        const auto& zList  = rayZPositions[rayIdx];

                        size_t countBefore = 0;
                        while (countBefore < zList.size() && zList[countBefore] < blockZStart)
                        {
                            countBefore++;
                        }
                        bool entryParity = (countBefore % 2 == 1);

                        if (entryParity)
                        {
                            for (size_t lz = 0; lz < blockSize; lz++)
                            {
                                blockData[lx + ly * blockSize + lz * sliceSize] = true;
                            }
                        }

                        for (size_t i = countBefore; i < zList.size() && zList[i] < blockZEnd; i++)
                        {
                            size_t zLocal = static_cast<size_t>(zList[i] - blockZStart);
                            for (size_t lz = zLocal; lz < blockSize; lz++)
                            {
                                size_t memIdx     = lx + ly * blockSize + lz * sliceSize;
                                blockData[memIdx] = blockData[memIdx] ^ true;
                            }
                        }
                    }
                }

                size_t solidCount       = 0;
                size_t totalBlockVoxels = blockSize * blockSize * blockSize;
                for (size_t i = 0; i < totalBlockVoxels; i++)
                {
                    if (blockData[i])
                    {
                        solidCount++;
                    }
                }

                if (solidCount == 0)
                {
                    continue;
                }

                if (solidCount == totalBlockVoxels)
                {
#pragma omp critical
                    onBlock(blockOffset, true, emptyVector);
                }
                else
                {
#pragma omp critical
                    onBlock(blockOffset, false, blockData);
                }
            }
        }
    }

    void SVOVoxelizer::spatialHashTrianglesToTiles(const std::vector<glm::dvec3>&    vertices,
                                                   const std::vector<size_t>&        indices,
                                                   const glm::dvec3&                 start,
                                                   const glm::dvec3&                 end,
                                                   uint32_t                          numTilesX,
                                                   uint32_t                          numTilesY,
                                                   std::vector<std::vector<size_t>>& tileTriangleStack) const
    {
        size_t     amountTriangles = indices.size() / 3;
        glm::dvec3 span            = end - start;

        for (size_t i = 0; i < amountTriangles; i++)
        {
            size_t            address = i * 3;
            const glm::dvec3& a       = vertices[indices[address]];
            const glm::dvec3& b       = vertices[indices[address + 1]];
            const glm::dvec3& c       = vertices[indices[address + 2]];

            glm::dvec2 outStart;
            glm::dvec2 outEnd;
            box(a, b, c, outStart, outEnd);

            long long xStartLL = static_cast<long long>(std::floor(((outStart.x - start.x) / span.x) * static_cast<double>(numTilesX)));
            long long yStartLL = static_cast<long long>(std::floor(((outStart.y - start.y) / span.y) * static_cast<double>(numTilesY)));

            xStartLL = std::max(0LL, xStartLL);
            yStartLL = std::max(0LL, yStartLL);

            long long xEndLL = static_cast<long long>(std::ceil(((outEnd.x - start.x) / span.x) * static_cast<double>(numTilesX)));
            long long yEndLL = static_cast<long long>(std::ceil(((outEnd.y - start.y) / span.y) * static_cast<double>(numTilesY)));

            if (xEndLL <= xStartLL)
            {
                xEndLL = xStartLL + 1;
            }
            if (yEndLL <= yStartLL)
            {
                yEndLL = yStartLL + 1;
            }

            xEndLL = std::min(static_cast<long long>(numTilesX), xEndLL);
            yEndLL = std::min(static_cast<long long>(numTilesY), yEndLL);

            for (size_t tx = static_cast<size_t>(xStartLL); tx < static_cast<size_t>(xEndLL); tx++)
            {
                for (size_t ty = static_cast<size_t>(yStartLL); ty < static_cast<size_t>(yEndLL); ty++)
                {
                    tileTriangleStack[ty * numTilesX + tx].push_back(address);
                }
            }
        }
    }

    void SVOVoxelizer::box(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, glm::dvec2& start, glm::dvec2& end) const
    {
        double minX = (a.x < b.x) ? ((a.x < c.x) ? a.x : c.x) : ((b.x < c.x) ? b.x : c.x);
        double minY = (a.y < b.y) ? ((a.y < c.y) ? a.y : c.y) : ((b.y < c.y) ? b.y : c.y);
        double maxX = (a.x > b.x) ? ((a.x > c.x) ? a.x : c.x) : ((b.x > c.x) ? b.x : c.x);
        double maxY = (a.y > b.y) ? ((a.y > c.y) ? a.y : c.y) : ((b.y > c.y) ? b.y : c.y);
        start       = glm::dvec2(minX, minY);
        end         = glm::dvec2(maxX, maxY);
    }

    bool SVOVoxelizer::intersectRayTriangle(const glm::dvec3& rayOrigin, const glm::dvec3& rayVector, const glm::dvec3& vertex0, const glm::dvec3& vertex1, const glm::dvec3& vertex2, glm::dvec3& out)
      const
    {
        const double epsilon = 1e-5;
        glm::dvec3   edge1   = vertex1 - vertex0;
        glm::dvec3   edge2   = vertex2 - vertex0;
        glm::dvec3   h       = glm::cross(rayVector, edge2);
        double       a       = glm::dot(edge1, h);

        if (a > -epsilon && a < epsilon)
        {
            return false;
        }

        double     f = 1.0 / a;
        glm::dvec3 s = rayOrigin - vertex0;
        double     u = f * glm::dot(s, h);

        if (u < -epsilon || u > 1.0 + epsilon)
        {
            return false;
        }

        glm::dvec3 q = glm::cross(s, edge1);
        double     v = f * glm::dot(rayVector, q);

        if (v < -epsilon || u + v > 1.0 + epsilon)
        {
            return false;
        }

        double t = f * glm::dot(edge2, q);

        if (t > epsilon)
        {
            out = rayOrigin + rayVector * t;
            return true;
        }

        return false;
    }

    bool SVOVoxelizer::epsilonEqual(double a, double b) const
    {
        return std::abs(a - b) < 1e-5;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/Triangle/BaseShapeGenerator.h"
#include "CADacomb/Volume/Flat/Voxelizer.h"
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("SVOVoxelizer/EquivalenceWithVoxelizer")
    {
        float        radius = 0.3f;
        auto         tri    = CADacomb::BaseShapeGenerator::cubeWithHole(8, radius);
        glm::dvec3   start(0.0, 0.0, 0.0);
        glm::dvec3   end(1.0, 1.0, 1.0);
        glm::u64vec3 resolution(16, 16, 16);

        CADacomb::Voxelizer flatVoxelizer;
        auto                flat = flatVoxelizer.voxelize(tri.vertices, tri.indices, start, end, resolution);
        REQUIRE(flat != nullptr);

        SVOVoxelizer svoVoxelizer;
        auto         svo = svoVoxelizer.voxelize(tri, start, end, 16);
        REQUIRE(svo != nullptr);

        for (size_t z = 0; z < 16; z++)
        {
            for (size_t y = 0; y < 16; y++)
            {
                for (size_t x = 0; x < 16; x++)
                {
                    bool flatValue = flat->data[x + y * resolution.x + z * resolution.x * resolution.y];
                    bool svoValue  = svo->getVoxel(glm::ivec3(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)));
                    CHECK(flatValue == svoValue);
                }
            }
        }
    }

    TEST_CASE("SVOVoxelizer/UnitCube32")
    {
        auto         tri = CADacomb::BaseShapeGenerator::cube();
        SVOVoxelizer voxelizer;
        glm::dvec3   start(0.0, 0.0, 0.0);
        glm::dvec3   end(1.0, 1.0, 1.0);
        uint32_t     dimension = 32;

        auto svo = voxelizer.voxelize(tri, start, end, dimension);

        REQUIRE(svo != nullptr);
        CHECK(svo->getDimension() == dimension);

        for (size_t z = 2; z < 30; z++)
        {
            for (size_t y = 2; y < 30; y++)
            {
                for (size_t x = 2; x < 30; x++)
                {
                    CHECK(svo->getVoxel(glm::ivec3(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z))) == true);
                }
            }
        }
    }

    TEST_CASE("SVOVoxelizer/UnitCubeOffset")
    {
        auto tri = CADacomb::BaseShapeGenerator::cube();
        for (auto& v : tri.vertices)
        {
            v += glm::dvec3(50.0, 50.0, 338.0);
        }
        SVOVoxelizer voxelizer;
        glm::dvec3   start(50.0, 50.0, 338.0);
        glm::dvec3   end(51.0, 51.0, 339.0);
        uint32_t     dimension = 32;

        auto svo = voxelizer.voxelize(tri, start, end, dimension);

        REQUIRE(svo != nullptr);
        CHECK(svo->getDimension() == dimension);

        for (size_t z = 2; z < 30; z++)
        {
            for (size_t y = 2; y < 30; y++)
            {
                for (size_t x = 2; x < 30; x++)
                {
                    CHECK(svo->getVoxel(glm::ivec3(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z))) == true);
                }
            }
        }
    }
}
#endif
