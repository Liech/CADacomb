#include "Voxelizer.h"
#include <algorithm>
#include <cassert>
#include <cmath>

namespace CADacomb
{
    Voxelizer::Voxelizer() {}

    Voxelizer::~Voxelizer() {}

    std::unique_ptr<BinaryVolume> Voxelizer::voxelize(const std::vector<glm::dvec3>& vertices,
                                                      const std::vector<size_t>&     indices,
                                                      const glm::dvec3&              start,
                                                      const glm::dvec3&              end,
                                                      const glm::u64vec3&            resolution)
    {
        assert(resolution.z % 8 == 0);
        size_t     memorySize = static_cast<size_t>(resolution.x) * resolution.y * resolution.z;
        glm::dvec3 span       = end - start;

        std::vector<bool> rawResult(memorySize, false);

        std::vector<std::vector<size_t>> triangleStack;
        triangleStack.resize(resolution.x * resolution.y);

        spatialHashTriangles(vertices, indices, start, end, resolution, triangleStack);
        applyTrianglesToVolume(rawResult, triangleStack, vertices, indices, start, end, resolution);

        auto volume       = std::make_unique<BinaryVolume>();
        volume->dimension = resolution;
        volume->origin    = start;
        volume->size      = span;
        volume->data.resize(memorySize, false);

        for (size_t z = 0; z < resolution.z; ++z)
        {
            for (size_t y = 0; y < resolution.y; ++y)
            {
                for (size_t x = 0; x < resolution.x; ++x)
                {
                    size_t rawIdx        = z + y * resolution.z + x * resolution.y * resolution.z;
                    size_t stdIdx        = x + y * resolution.x + z * resolution.x * resolution.y;
                    volume->data[stdIdx] = rawResult[rawIdx];
                }
            }
        }

        return volume;
    }

    std::unique_ptr<BinaryVolume> Voxelizer::voxelize(const std::vector<glm::dvec3>& vertices, const glm::dvec3& start, const glm::dvec3& end, const glm::u64vec3& resolution)
    {
        std::vector<size_t> indices(vertices.size());
        for (size_t i = 0; i < indices.size(); i++)
        {
            indices[i] = i;
        }

        return voxelize(vertices, indices, start, end, resolution);
    }

    void Voxelizer::applyTrianglesToVolume(std::vector<bool>&                      data,
                                           const std::vector<std::vector<size_t>>& triangleStack,
                                           const std::vector<glm::dvec3>&          vertices,
                                           const std::vector<size_t>&              indices,
                                           const glm::dvec3&                       start,
                                           const glm::dvec3&                       end,
                                           const glm::u64vec3&                     resolution)
    {
        glm::dvec3 span = end - start;

#pragma omp parallel for
        for (long long x = 0; x < static_cast<long long>(resolution.x); x++)
        {
            std::vector<double> intersections;

            for (size_t y = 0; y < resolution.y; y++)
            {
                size_t stackAddress = x + resolution.x * y;
                size_t resultOffset = resolution.z * resolution.y * x + resolution.z * y;
                auto&  stack        = triangleStack[stackAddress];

                if (stack.empty())
                {
                    continue;
                }

                double rx   = (static_cast<double>(x) + 0.5) / static_cast<double>(resolution.x);
                double ry   = (static_cast<double>(y) + 0.5) / static_cast<double>(resolution.y);
                double rayX = start.x + span.x * rx;
                double rayY = start.y + span.y * ry;

                intersections.clear();
                for (auto& tri : stack)
                {
                    const glm::dvec3& a             = vertices[indices[tri]];
                    const glm::dvec3& b             = vertices[indices[tri + 1]];
                    const glm::dvec3& c             = vertices[indices[tri + 2]];
                    double            intersectionZ = 0.0;
                    if (intersectVerticalRayTriangle(rayX, rayY, a, b, c, intersectionZ))
                    {
                        intersections.push_back(intersectionZ);
                    }
                }
                std::sort(intersections.begin(), intersections.end());

                for (auto& zIntersection : intersections)
                {
                    long long zPos = std::round(((zIntersection - start.z) / span.z) * static_cast<double>(resolution.z));
                    zPos           = std::max(0LL, zPos);
                    zPos           = std::min(static_cast<long long>(resolution.z), zPos);

                    size_t memoryAddress = zPos + resultOffset;
                    size_t max           = resultOffset + resolution.z;
                    for (size_t i = memoryAddress; i < max; i++)
                    {
                        data[i] = data[i] ^ true;
                    }
                }
            }
        }
    }

    void Voxelizer::spatialHashTriangles(const std::vector<glm::dvec3>&    vertices,
                                         const std::vector<size_t>&        indices,
                                         const glm::dvec3&                 start,
                                         const glm::dvec3&                 end,
                                         const glm::u64vec3&               resolution,
                                         std::vector<std::vector<size_t>>& triangleStack)
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

            long long xStartLL = std::floor(((outStart.x - start.x) / span.x) * static_cast<double>(resolution.x));
            long long yStartLL = std::floor(((outStart.y - start.y) / span.y) * static_cast<double>(resolution.y));

            xStartLL = std::max(0LL, xStartLL);
            yStartLL = std::max(0LL, yStartLL);

            size_t xStart = static_cast<size_t>(xStartLL);
            size_t yStart = static_cast<size_t>(yStartLL);

            long long xEndLL = std::ceil(((outEnd.x - start.x) / span.x) * static_cast<double>(resolution.x));
            long long yEndLL = std::ceil(((outEnd.y - start.y) / span.y) * static_cast<double>(resolution.y));

            if (xEndLL <= xStartLL)
            {
                xEndLL = xStartLL + 1;
            }
            if (yEndLL <= yStartLL)
            {
                yEndLL = yStartLL + 1;
            }

            xEndLL = std::min(static_cast<long long>(resolution.x), xEndLL);
            yEndLL = std::min(static_cast<long long>(resolution.y), yEndLL);

            size_t xEnd = static_cast<size_t>(xEndLL);
            size_t yEnd = static_cast<size_t>(yEndLL);

            for (size_t x = xStart; x < xEnd; x++)
            {
                for (size_t y = yStart; y < yEnd; y++)
                {
                    triangleStack[y * resolution.x + x].push_back(address);
                }
            }
        }
    }

    void Voxelizer::box(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, glm::dvec2& start, glm::dvec2& end)
    {
        double minX = (a.x < b.x) ? ((a.x < c.x) ? a.x : c.x) : ((b.x < c.x) ? b.x : c.x);
        double minY = (a.y < b.y) ? ((a.y < c.y) ? a.y : c.y) : ((b.y < c.y) ? b.y : c.y);
        double maxX = (a.x > b.x) ? ((a.x > c.x) ? a.x : c.x) : ((b.x > c.x) ? b.x : c.x);
        double maxY = (a.y > b.y) ? ((a.y > c.y) ? a.y : c.y) : ((b.y > c.y) ? b.y : c.y);
        start       = glm::dvec2(minX, minY);
        end         = glm::dvec2(maxX, maxY);
    }

    int Voxelizer::edgeSign(const glm::dvec3& a, const glm::dvec3& b, double px, double py, double& outCross) const
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

    bool Voxelizer::intersectVerticalRayTriangle(double rayX, double rayY, const glm::dvec3& vertex0, const glm::dvec3& vertex1, const glm::dvec3& vertex2, double& outZ) const
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
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Voxelizer/CubeWithHole16")
{
    float               radius = 0.3f;
    auto                tri    = CADacomb::BaseShapeGenerator::cubeWithHole(32, radius);
    CADacomb::Voxelizer voxelizer;
    glm::dvec3          start(0.0, 0.0, 0.0);
    glm::dvec3          end(1.0, 1.0, 1.0);
    glm::u64vec3        resolution(16, 16, 16);

    auto volume = voxelizer.voxelize(tri.vertices, tri.indices, start, end, resolution);

    REQUIRE(volume != nullptr);
    REQUIRE(volume->dimension == resolution);
    REQUIRE(volume->origin == start);
    REQUIRE(volume->size == (end - start));
    REQUIRE(volume->data.size() == 16 * 16 * 16);

    auto getVoxel = [&](size_t x, size_t y, size_t z) -> bool { return volume->data[x + y * resolution.x + z * resolution.x * resolution.y]; };

    size_t solidCount = 0;
    for (size_t x = 0; x < resolution.x; ++x)
    {
        for (size_t y = 0; y < resolution.y; ++y)
        {
            for (size_t z = 0; z < resolution.z; ++z)
            {
                if (getVoxel(x, y, z))
                {
                    solidCount++;
                }
            }
        }
    }

    for (size_t z = 0; z < resolution.z; ++z)
    {
        for (size_t x = 6; x <= 9; ++x)
        {
            for (size_t y = 6; y <= 9; ++y)
            {
                double cx   = (static_cast<double>(x) + 0.5) / static_cast<double>(resolution.x);
                double cy   = (static_cast<double>(y) + 0.5) / static_cast<double>(resolution.y);
                double dist = std::sqrt((cx - 0.5) * (cx - 0.5) + (cy - 0.5) * (cy - 0.5));
                if (dist < radius - 0.05)
                {
                    INFO("Hole voxel at (" << x << ", " << y << ", " << z << ") dist=" << dist << " should be false");
                    CHECK(getVoxel(x, y, z) == false);
                }
            }
        }
    }

    for (size_t z = 1; z < resolution.z - 1; ++z)
    {
        for (size_t x : { static_cast<size_t>(1), static_cast<size_t>(14) })
        {
            for (size_t y : { static_cast<size_t>(1), static_cast<size_t>(14) })
            {
                INFO("Corner voxel at (" << x << ", " << y << ", " << z << ") should be true");
                CHECK(getVoxel(x, y, z) == true);
            }
        }
    }

    CHECK(solidCount > 2000);
    CHECK(solidCount < 3500);
}

TEST_CASE("Voxelizer/CubeOffset")
{
    auto tri = CADacomb::BaseShapeGenerator::cube();
    for (auto& v : tri.vertices)
    {
        v += glm::dvec3(50.0, 50.0, 338.0);
    }
    CADacomb::Voxelizer voxelizer;
    glm::dvec3          start(50.0, 50.0, 338.0);
    glm::dvec3          end(51.0, 51.0, 339.0);
    glm::u64vec3        resolution(16, 16, 16);

    auto volume = voxelizer.voxelize(tri.vertices, tri.indices, start, end, resolution);

    REQUIRE(volume != nullptr);
    size_t solidCount = 0;
    for (size_t i = 0; i < volume->data.size(); i++)
    {
        if (volume->data[i])
        {
            solidCount++;
        }
    }
    CHECK(solidCount > 0);
}
#endif
