#include "SVOVoxelizer.h"

#include "CADacomb/Util/ProgressBar.h"

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

        std::vector<bool> emptyVector;

        size_t      totalTiles = static_cast<size_t>(numTilesX * numTilesY);
        ProgressBar voxelizeBar(totalTiles, "Voxelizing mesh tiles");

#pragma omp parallel for schedule(dynamic) shared(voxelizeBar)
        for (long long tileIdx = 0; tileIdx < static_cast<long long>(totalTiles); tileIdx++)
        {
            if (voxelizeBar.isCanceled())
            {
                continue;
            }
            size_t      tileX         = static_cast<size_t>(tileIdx % numTilesX);
            size_t      tileY         = static_cast<size_t>(tileIdx / numTilesX);
            const auto& tileTriangles = tileTriangleStack[tileIdx];

            if (tileTriangles.empty())
            {
                voxelizeBar.addProgress(1);
                continue;
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

                    double rx   = (static_cast<double>(globalX) + 0.5) / static_cast<double>(dimension);
                    double ry   = (static_cast<double>(globalY) + 0.5) / static_cast<double>(dimension);
                    double rayX = start.x + span.x * rx;
                    double rayY = start.y + span.y * ry;

                    intersections.clear();
                    for (size_t tri : tileTriangles)
                    {
                        const glm::dvec3& a = vertices[indices[tri]];
                        const glm::dvec3& b = vertices[indices[tri + 1]];
                        const glm::dvec3& c = vertices[indices[tri + 2]];

                        double minX = (a.x < b.x) ? ((a.x < c.x) ? a.x : c.x) : ((b.x < c.x) ? b.x : c.x);
                        double maxX = (a.x > b.x) ? ((a.x > c.x) ? a.x : c.x) : ((b.x > c.x) ? b.x : c.x);
                        if (rayX < minX || rayX > maxX)
                        {
                            continue;
                        }

                        double minY = (a.y < b.y) ? ((a.y < c.y) ? a.y : c.y) : ((b.y < c.y) ? b.y : c.y);
                        double maxY = (a.y > b.y) ? ((a.y > c.y) ? a.y : c.y) : ((b.y > c.y) ? b.y : c.y);
                        if (rayY < minY || rayY > maxY)
                        {
                            continue;
                        }

                        double intersectionZ = 0.0;
                        if (intersectVerticalRayTriangle(rayX, rayY, a, b, c, intersectionZ))
                        {
                            intersections.push_back(intersectionZ);
                        }
                    }

                    std::sort(intersections.begin(), intersections.end());

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

            voxelizeBar.addProgress(1);
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

    int SVOVoxelizer::edgeSign(const glm::dvec3& a, const glm::dvec3& b, double px, double py, double& outCross) const
    {
        bool              swapped = false;
        const glm::dvec3* lo      = &a;
        const glm::dvec3* hi      = &b;

        if (a.x > b.x || (a.x == b.x && a.y > b.y))
        {
            lo      = &b;
            hi      = &a;
            swapped = true;
        }

        double ax    = lo->x - px;
        double ay    = lo->y - py;
        double bx    = hi->x - px;
        double by    = hi->y - py;
        double cross = ax * by - ay * bx;

        int sign = 0;
        if (cross > 0.0)
        {
            sign = 1;
        }
        else if (cross < 0.0)
        {
            sign = -1;
        }
        else if (lo->y != hi->y)
        {
            sign = (lo->y > hi->y) ? 1 : -1;
        }
        else if (hi->x != lo->x)
        {
            sign = (hi->x > lo->x) ? 1 : -1;
        }

        if (swapped)
        {
            outCross = -cross;
            return -sign;
        }

        outCross = cross;
        return sign;
    }

    bool SVOVoxelizer::intersectVerticalRayTriangle(double rayX, double rayY, const glm::dvec3& vertex0, const glm::dvec3& vertex1, const glm::dvec3& vertex2, double& outZ) const
    {
        double w2 = 0.0;
        int    s0 = edgeSign(vertex0, vertex1, rayX, rayY, w2);
        if (s0 == 0)
        {
            return false;
        }

        double w0 = 0.0;
        int    s1 = edgeSign(vertex1, vertex2, rayX, rayY, w0);
        if (s1 != s0)
        {
            return false;
        }

        double w1 = 0.0;
        int    s2 = edgeSign(vertex2, vertex0, rayX, rayY, w1);
        if (s2 != s0)
        {
            return false;
        }

        double wSum = w0 + w1 + w2;
        if (wSum == 0.0)
        {
            return false;
        }

        outZ = (w0 * vertex0.z + w1 * vertex1.z + w2 * vertex2.z) / wSum;
        return true;
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

    TEST_CASE("SVOVoxelizer/NoInvertedStrandsOnSilhouetteAndSharedEdges")
    {
        Triangulation tri;
        double        center = 16.5 / 32.0;
        double        radius = 8.0 / 32.0;
        double        zMid   = 0.5;
        double        zHalf  = 0.25;

        tri.vertices = {
            glm::dvec3(center, center, zMid + zHalf),
            glm::dvec3(center, center, zMid - zHalf),
            glm::dvec3(center + radius, center, zMid),
            glm::dvec3(center, center + radius, zMid),
            glm::dvec3(center - radius, center, zMid),
            glm::dvec3(center, center - radius, zMid)
        };
        tri.indices = {
            0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 2,
            1, 3, 2, 1, 4, 3, 1, 5, 4, 1, 2, 5
        };

        SVOVoxelizer voxelizer;
        auto         svo = voxelizer.voxelize(tri, glm::dvec3(0.0), glm::dvec3(1.0), 32);
        REQUIRE(svo != nullptr);

        CHECK(svo->getVoxel(glm::ivec3(16, 16, 16)) == true);

        for (int y = 0; y < 32; y++)
        {
            for (int x = 0; x < 32; x++)
            {
                CHECK(svo->getVoxel(glm::ivec3(x, y, 0)) == false);
                CHECK(svo->getVoxel(glm::ivec3(x, y, 31)) == false);
            }
        }
    }

    TEST_CASE("SVOVoxelizer/NoInvertedStrandsOnTinyXYAreaTriangles")
    {
        Triangulation tri;
        double        rayX = 16.5 / 32.0;
        double        rayY = 16.5 / 32.0;
        double        half = 1e-4;

        tri.vertices = {
            glm::dvec3(rayX - half, rayY - half, 0.25),
            glm::dvec3(rayX + half, rayY - half, 0.25),
            glm::dvec3(rayX + half, rayY + half, 0.25),
            glm::dvec3(rayX - half, rayY + half, 0.25),
            glm::dvec3(rayX - half, rayY - half, 0.75),
            glm::dvec3(rayX + half, rayY - half, 0.75),
            glm::dvec3(rayX + half, rayY + half, 0.75),
            glm::dvec3(rayX - half, rayY + half, 0.75)
        };
        tri.indices = {
            0, 2, 1, 0, 3, 2,
            4, 5, 6, 4, 6, 7,
            0, 1, 5, 0, 5, 4,
            1, 2, 6, 1, 6, 5,
            2, 3, 7, 2, 7, 6,
            3, 0, 4, 3, 4, 7
        };

        SVOVoxelizer voxelizer;
        auto         svo = voxelizer.voxelize(tri, glm::dvec3(0.0), glm::dvec3(1.0), 32);
        REQUIRE(svo != nullptr);

        CHECK(svo->getVoxel(glm::ivec3(16, 16, 16)) == true);
        CHECK(svo->getVoxel(glm::ivec3(16, 16, 0)) == false);
        CHECK(svo->getVoxel(glm::ivec3(16, 16, 31)) == false);
    }

    TEST_CASE("SVOVoxelizer/NoInvertedStrandsOnThinWedge")
    {
        Triangulation tri;
        tri.vertices = {
            glm::dvec3(0.2, 0.2, 0.5),
            glm::dvec3(0.8, 0.2, 0.5),
            glm::dvec3(0.8, 0.8, 0.5),
            glm::dvec3(0.2, 0.8, 0.5),
            glm::dvec3(0.2, 0.2, 0.5 + 1e-6),
            glm::dvec3(0.8, 0.2, 0.5 + 1e-6),
            glm::dvec3(0.8, 0.8, 0.5 + 1e-6),
            glm::dvec3(0.2, 0.8, 0.5 + 1e-6)
        };
        tri.indices = {
            0, 2, 1, 0, 3, 2,
            4, 5, 6, 4, 6, 7,
            0, 1, 5, 0, 5, 4,
            1, 2, 6, 1, 6, 5,
            2, 3, 7, 2, 7, 6,
            3, 0, 4, 3, 4, 7
        };

        SVOVoxelizer voxelizer;
        auto         svo = voxelizer.voxelize(tri, glm::dvec3(0.0), glm::dvec3(1.0), 32);
        REQUIRE(svo != nullptr);

        for (int y = 0; y < 32; y++)
        {
            for (int x = 0; x < 32; x++)
            {
                CHECK(svo->getVoxel(glm::ivec3(x, y, 0)) == false);
                CHECK(svo->getVoxel(glm::ivec3(x, y, 31)) == false);
            }
        }
    }
}
#endif
