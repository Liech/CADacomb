#include "SVOLandscapeGenerator.h"

#include <algorithm>
#include <cmath>

namespace CADacomb
{
    SVOLandscapeGenerator::SVOLandscapeGenerator() {}

    SVOLandscapeGenerator::~SVOLandscapeGenerator() {}

    int SVOLandscapeGenerator::fastFloor(double x)
    {
        int i = static_cast<int>(x);
        return (x < static_cast<double>(i)) ? (i - 1) : i;
    }

    uint32_t SVOLandscapeGenerator::hash3D(int x, int y, int z, uint32_t seed)
    {
        uint32_t h = seed ^ (static_cast<uint32_t>(x) * 0x73856093u) ^ (static_cast<uint32_t>(y) * 0x19379363u) ^ (static_cast<uint32_t>(z) * 0x834923f9u);
        h ^= h >> 16;
        h *= 0x85ebca6bu;
        h ^= h >> 13;
        h *= 0xc2b2ae35u;
        h ^= h >> 16;
        return h;
    }

    double SVOLandscapeGenerator::grad(uint32_t hash, double x, double y, double z)
    {
        switch (hash % 12)
        {
            case 0:
                return x + y;
            case 1:
                return -x + y;
            case 2:
                return x - y;
            case 3:
                return -x - y;
            case 4:
                return x + z;
            case 5:
                return -x + z;
            case 6:
                return x - z;
            case 7:
                return -x - z;
            case 8:
                return y + z;
            case 9:
                return -y + z;
            case 10:
                return y - z;
            case 11:
                return -y - z;
            default:
                return 0.0;
        }
    }

    double SVOLandscapeGenerator::fade(double t)
    {
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    }

    double SVOLandscapeGenerator::lerp(double a, double b, double t)
    {
        return a + t * (b - a);
    }

    double SVOLandscapeGenerator::perlinNoise3D(const glm::dvec3& pos, uint32_t seed)
    {
        int x0 = fastFloor(pos.x);
        int y0 = fastFloor(pos.y);
        int z0 = fastFloor(pos.z);

        int x1 = x0 + 1;
        int y1 = y0 + 1;
        int z1 = z0 + 1;

        double dx = pos.x - static_cast<double>(x0);
        double dy = pos.y - static_cast<double>(y0);
        double dz = pos.z - static_cast<double>(z0);

        double u = fade(dx);
        double v = fade(dy);
        double w = fade(dz);

        double g000 = grad(hash3D(x0, y0, z0, seed), dx, dy, dz);
        double g100 = grad(hash3D(x1, y0, z0, seed), dx - 1.0, dy, dz);
        double g010 = grad(hash3D(x0, y1, z0, seed), dx, dy - 1.0, dz);
        double g110 = grad(hash3D(x1, y1, z0, seed), dx - 1.0, dy - 1.0, dz);

        double g001 = grad(hash3D(x0, y0, z1, seed), dx, dy, dz - 1.0);
        double g101 = grad(hash3D(x1, y0, z1, seed), dx - 1.0, dy, dz - 1.0);
        double g011 = grad(hash3D(x0, y1, z1, seed), dx, dy - 1.0, dz - 1.0);
        double g111 = grad(hash3D(x1, y1, z1, seed), dx - 1.0, dy - 1.0, dz - 1.0);

        double x00 = lerp(g000, g100, u);
        double x10 = lerp(g010, g110, u);
        double x01 = lerp(g001, g101, u);
        double x11 = lerp(g011, g111, u);

        double y0Val = lerp(x00, x10, v);
        double y1Val = lerp(x01, x11, v);

        return lerp(y0Val, y1Val, w);
    }

    double SVOLandscapeGenerator::fbm3D(const glm::dvec3& pos, int octaves, double frequency, double persistence, double lacunarity, uint32_t seed)
    {
        double sum    = 0.0;
        double amp    = 1.0;
        double maxAmp = 0.0;
        double freq   = frequency;

        for (int i = 0; i < octaves; i++)
        {
            sum += amp * perlinNoise3D(pos * freq, seed + static_cast<uint32_t>(i * 31));
            maxAmp += amp;
            freq *= lacunarity;
            amp *= persistence;
        }

        return (maxAmp > 0.0) ? (sum / maxAmp) : 0.0;
    }

    double SVOLandscapeGenerator::evaluateDensity(const glm::dvec3& localPos, double volumeHeight, const LandscapeConfig& config)
    {
        if (config.customDensity)
        {
            return config.customDensity(localPos);
        }

        double h          = (volumeHeight > 0.0) ? (localPos.y / volumeHeight) : 0.0;
        double heightGrad = (config.baseHeightRatio - h) * 2.0 * config.heightScale;

        double terrainNoise = fbm3D(localPos, config.octaves, config.frequency, config.persistence, config.lacunarity, config.seed);

        double density = heightGrad + terrainNoise;

        if (config.enableCaves && h < 0.9)
        {
            double caveNoise = fbm3D(localPos, 2, config.caveFrequency, 0.5, 2.0, config.seed + 9999);

            if (std::abs(caveNoise) < config.caveThreshold)
            {
                density = -1.0;
            }
        }

        return density;
    }

    std::pair<double, double> SVOLandscapeGenerator::estimateDensityBounds(const glm::ivec3&      boxMin,
                                                                           const glm::ivec3&      boxSize,
                                                                           double                 volumeHeight,
                                                                           const LandscapeConfig& config,
                                                                           const glm::dvec3&      worldOffset)
    {
        if (config.customDensity)
        {
            return { -1e9, 1e9 };
        }

        if (volumeHeight <= 0.0 || config.heightScale <= 0.0)
        {
            return { -1.0, 1.0 };
        }

        double yMin = static_cast<double>(boxMin.y) + worldOffset.y;
        double yMax = static_cast<double>(boxMin.y + std::max(1, boxSize.y) - 1) + worldOffset.y;

        double hMin = yMin / volumeHeight;
        double hMax = yMax / volumeHeight;

        double minHeightGrad = (config.baseHeightRatio - hMax) * 2.0 * config.heightScale;
        double maxHeightGrad = (config.baseHeightRatio - hMin) * 2.0 * config.heightScale;

        glm::dvec3 pMin   = glm::dvec3(boxMin) + worldOffset;
        glm::dvec3 pMax   = glm::dvec3(boxMin + glm::max(glm::ivec3(1), boxSize) - 1) + worldOffset;
        glm::dvec3 center = 0.5 * (pMin + pMax);
        double     radius = 0.5 * glm::length(pMax - pMin);

        constexpr double perlinLipschitz = 2.6;

        double sumAmp     = 0.0;
        double currentAmp = 1.0;
        for (int i = 0; i < config.octaves; i++)
        {
            sumAmp += currentAmp;
            currentAmp *= config.persistence;
        }

        double minTerrainNoise = -1.0;
        double maxTerrainNoise = 1.0;

        if (sumAmp > 0.0)
        {
            double boundMaxSum = 0.0;
            double boundMinSum = 0.0;
            double octaveAmp   = 1.0;
            double octaveFreq  = config.frequency;

            for (int i = 0; i < config.octaves; i++)
            {
                double delta = octaveFreq * perlinLipschitz * radius;
                if (delta < 2.0)
                {
                    double n = perlinNoise3D(center * octaveFreq, config.seed + static_cast<uint32_t>(i * 31));
                    boundMaxSum += octaveAmp * std::min(1.0, n + delta);
                    boundMinSum += octaveAmp * std::max(-1.0, n - delta);
                }
                else
                {
                    boundMaxSum += octaveAmp * 1.0;
                    boundMinSum += octaveAmp * (-1.0);
                }
                octaveFreq *= config.lacunarity;
                octaveAmp *= config.persistence;
            }

            maxTerrainNoise = boundMaxSum / sumAmp;
            minTerrainNoise = boundMinSum / sumAmp;
        }

        return { minHeightGrad + minTerrainNoise, maxHeightGrad + maxTerrainNoise };
    }

    bool SVOLandscapeGenerator::canCarveCaves(const glm::ivec3& boxMin, const glm::ivec3& boxSize, double volumeHeight, const LandscapeConfig& config, const glm::dvec3& worldOffset)
    {
        if (!config.enableCaves)
        {
            return false;
        }

        double yMin = static_cast<double>(boxMin.y) + worldOffset.y;
        double hMin = (volumeHeight > 0.0) ? (yMin / volumeHeight) : 0.0;
        if (hMin >= 0.9)
        {
            return false;
        }

        glm::dvec3 pMin   = glm::dvec3(boxMin) + worldOffset;
        glm::dvec3 pMax   = glm::dvec3(boxMin + glm::max(glm::ivec3(1), boxSize) - 1) + worldOffset;
        glm::dvec3 center = 0.5 * (pMin + pMax);
        double     radius = 0.5 * glm::length(pMax - pMin);

        constexpr double perlinLipschitz = 2.6;

        double sumAmp     = 1.0 + 0.5;
        double octaveAmp  = 1.0;
        double octaveFreq = config.caveFrequency;

        double boundMaxSum = 0.0;
        double boundMinSum = 0.0;
        for (int i = 0; i < 2; i++)
        {
            double delta = octaveFreq * perlinLipschitz * radius;
            if (delta < 2.0)
            {
                double n = perlinNoise3D(center * octaveFreq, config.seed + 9999 + static_cast<uint32_t>(i * 31));
                boundMaxSum += octaveAmp * std::min(1.0, n + delta);
                boundMinSum += octaveAmp * std::max(-1.0, n - delta);
            }
            else
            {
                boundMaxSum += octaveAmp * 1.0;
                boundMinSum += octaveAmp * (-1.0);
            }
            octaveFreq *= 2.0;
            octaveAmp *= 0.5;
        }

        double minCaveNoise = boundMinSum / sumAmp;
        double maxCaveNoise = boundMaxSum / sumAmp;

        if (minCaveNoise > config.caveThreshold || maxCaveNoise < -config.caveThreshold)
        {
            return false;
        }

        return true;
    }

    CADacomb::BinaryVolume SVOLandscapeGenerator::generateBinaryVolume(const glm::u64vec3& dimension, const LandscapeConfig& config, const glm::dvec3& worldOffset, const glm::dvec3& worldSize)
    {
        CADacomb::BinaryVolume volume;
        volume.dimension = dimension;
        volume.origin    = worldOffset;
        volume.size      = worldSize;

        size_t totalVoxels = dimension.x * dimension.y * dimension.z;
        if (totalVoxels == 0)
        {
            return volume;
        }

        std::vector<uint8_t> temp(totalVoxels, 0);
        double               volumeHeight = static_cast<double>(dimension.y);

        const int64_t dimZ = static_cast<int64_t>(dimension.z);
        const int64_t dimY = static_cast<int64_t>(dimension.y);
        const int64_t dimX = static_cast<int64_t>(dimension.x);

        int64_t maxY   = dimY;
        int64_t solidY = -1;

        if (!config.customDensity && volumeHeight > 0.0 && config.heightScale > 0.0)
        {
            double minSolidRatio = config.baseHeightRatio - (config.solidThreshold + 1.0) / (2.0 * config.heightScale);
            if (!config.enableCaves && minSolidRatio > 0.0)
            {
                double solidWorldY = minSolidRatio * volumeHeight - worldOffset.y;
                if (solidWorldY > 0.0)
                {
                    solidY = std::min(dimY, static_cast<int64_t>(std::floor(solidWorldY)));
                }
            }

            double maxRatio  = config.baseHeightRatio + (1.0 - config.solidThreshold) / (2.0 * config.heightScale);
            double maxWorldY = maxRatio * volumeHeight - worldOffset.y;
            if (maxWorldY >= 0.0)
            {
                maxY = std::min(dimY, static_cast<int64_t>(std::ceil(maxWorldY)) + 1);
            }
            else
            {
                maxY = 0;
            }
        }

#pragma omp parallel for schedule(dynamic)
        for (int64_t z = 0; z < dimZ; z++)
        {
            for (int64_t y = 0; y < maxY; y++)
            {
                if (y < solidY)
                {
                    size_t rowStart = static_cast<size_t>(y * dimX + z * dimX * dimY);
                    std::fill_n(&temp[rowStart], dimX, uint8_t(1));
                    continue;
                }

                for (int64_t x = 0; x < dimX; x++)
                {
                    glm::dvec3 pos     = glm::dvec3(x, y, z) + worldOffset;
                    double     density = evaluateDensity(pos, volumeHeight, config);
                    if (density > config.solidThreshold)
                    {
                        size_t idx = static_cast<size_t>(x + y * dimX + z * dimX * dimY);
                        temp[idx]  = 1;
                    }
                }
            }
        }

        volume.data.assign(temp.begin(), temp.end());
        return volume;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/Volume/SVO/Blocky/SVOBlockyTriangulation.h"
#include <catch2/catch_test_macros.hpp>
#include <chrono>

namespace CADacomb
{
    TEST_CASE("SVOLandscape/PerlinNoiseBoundsAndDeterminism", "[SVO][Landscape][Noise]")
    {
        for (int i = 0; i < 100; i++)
        {
            glm::dvec3 pos(i * 0.37, (i % 7) * 1.83, (i % 11) * 0.91);
            double     val = SVOLandscapeGenerator::perlinNoise3D(pos, 42);
            CHECK(val >= -1.0);
            CHECK(val <= 1.0);
        }

        glm::dvec3 testPos(12.345, 67.891, 23.456);
        double     val1 = SVOLandscapeGenerator::perlinNoise3D(testPos, 1337);
        double     val2 = SVOLandscapeGenerator::perlinNoise3D(testPos, 1337);
        CHECK(val1 == val2);

        double valDiffSeed = SVOLandscapeGenerator::perlinNoise3D(testPos, 9999);
        CHECK(val1 != valDiffSeed);

        double valNear = SVOLandscapeGenerator::perlinNoise3D(testPos + glm::dvec3(0.001, 0.0, 0.0), 1337);
        CHECK(std::abs(val1 - valNear) < 0.02);
    }

    TEST_CASE("SVOLandscape/FBMNoiseEvaluation", "[SVO][Landscape][Noise]")
    {
        glm::dvec3 pos(5.0, 10.0, 15.0);
        double     fbmVal = SVOLandscapeGenerator::fbm3D(pos, 4, 0.05, 0.5, 2.0, 123);
        CHECK(fbmVal >= -1.0);
        CHECK(fbmVal <= 1.0);
    }

    TEST_CASE("SVOLandscape/DensityHeightGradient", "[SVO][Landscape]")
    {
        LandscapeConfig config;
        config.baseHeightRatio = 0.5;
        config.heightScale     = 1.0;
        config.seed            = 12345;

        double volumeHeight = 64.0;

        double deepDensity = SVOLandscapeGenerator::evaluateDensity(glm::dvec3(32.0, 4.0, 32.0), volumeHeight, config);
        CHECK(deepDensity > 0.0);

        double skyDensity = SVOLandscapeGenerator::evaluateDensity(glm::dvec3(32.0, 60.0, 32.0), volumeHeight, config);
        CHECK(skyDensity < 0.0);
    }

    TEST_CASE("SVOLandscape/CaveCarving", "[SVO][Landscape]")
    {
        LandscapeConfig configNoCaves;
        configNoCaves.enableCaves     = false;
        configNoCaves.baseHeightRatio = 0.5;
        configNoCaves.seed            = 777;

        LandscapeConfig configWithCaves = configNoCaves;
        configWithCaves.enableCaves     = true;
        configWithCaves.caveThreshold   = 0.99;

        double     volumeHeight = 64.0;
        glm::dvec3 midPos(32.0, 16.0, 32.0);

        double densityWithCaves = SVOLandscapeGenerator::evaluateDensity(midPos, volumeHeight, configWithCaves);
        CHECK(densityWithCaves == -1.0);
    }

    TEST_CASE("SVOLandscape/BinaryVolumeGeneration", "[SVO][Landscape]")
    {
        LandscapeConfig config;
        config.seed            = 42;
        config.baseHeightRatio = 0.5;

        glm::u64vec3 dim(16, 16, 16);
        BinaryVolume vol = SVOLandscapeGenerator::generateBinaryVolume(dim, config);

        REQUIRE(vol.dimension == dim);
        REQUIRE(vol.data.size() == 16 * 16 * 16);

        size_t solidCount = 0;
        for (bool b : vol.data)
        {
            if (b)
            {
                solidCount++;
            }
        }

        CHECK(solidCount > 0);
        CHECK(solidCount < vol.data.size());
    }

    TEST_CASE("SVOLandscape/SVOGenerationAndTriangulation", "[SVO][Landscape]")
    {
        LandscapeConfig config;
        config.seed            = 99;
        config.octaves         = 1;
        config.baseHeightRatio = 0.5;

        auto svo = SVOLandscapeGenerator::generateSVO<bool>(32, config);
        REQUIRE(svo != nullptr);
        CHECK(svo->getDimension() == 32);

        SVOBlockyOptions options;
        options.targetDepth = 5;
        options.origin      = glm::dvec3(0.0);
        options.size        = glm::dvec3(32.0);

        auto mesh = SVOBlockyTriangulation::triangulate(*svo, options);
        REQUIRE(mesh != nullptr);
        CHECK(!mesh->vertices.empty());
        CHECK(!mesh->indices.empty());
        CHECK(mesh->indices.size() % 3 == 0);

        auto [minB, sz] = mesh->getAABB();
        CHECK(minB.x >= 0.0);
        CHECK(minB.y >= 0.0);
        CHECK(minB.z >= 0.0);
        CHECK(minB.x + sz.x <= 32.0 + 1e-4);
        CHECK(minB.y + sz.y <= 32.0 + 1e-4);
        CHECK(minB.z + sz.z <= 32.0 + 1e-4);
    }

    TEST_CASE("SVOLandscape/FillSVOChunk", "[SVO][Landscape]")
    {
        SVO<bool> svo(64, false, MostPrevalentExceptMergeStrategy<bool, false>::instance());

        LandscapeConfig config;
        config.seed            = 101;
        config.octaves         = 1;
        config.baseHeightRatio = 0.5;

        SVOLandscapeGenerator::fillSVOChunk(svo, glm::ivec3(0, 0, 0), glm::ivec3(32, 32, 32), 64.0, config);

        bool hasSolidInA = false;
        for (int z = 0; z < 32 && !hasSolidInA; z++)
        {
            for (int y = 0; y < 16 && !hasSolidInA; y++)
            {
                for (int x = 0; x < 32 && !hasSolidInA; x++)
                {
                    if (svo.getVoxel(glm::ivec3(x, y, z)))
                    {
                        hasSolidInA = true;
                    }
                }
            }
        }
        CHECK(hasSolidInA);

        bool hasSolidInUnfilled = false;
        for (int z = 32; z < 48 && !hasSolidInUnfilled; z++)
        {
            for (int y = 48; y < 64 && !hasSolidInUnfilled; y++)
            {
                for (int x = 32; x < 48 && !hasSolidInUnfilled; x++)
                {
                    if (svo.getVoxel(glm::ivec3(x, y, z)))
                    {
                        hasSolidInUnfilled = true;
                    }
                }
            }
        }
        CHECK_FALSE(hasSolidInUnfilled);
    }

    TEST_CASE("SVOLandscape/EstimateDensityBounds", "[SVO][Landscape]")
    {
        LandscapeConfig config;
        config.seed            = 42;
        config.octaves         = 4;
        config.baseHeightRatio = 0.5;
        config.heightScale     = 1.2;

        double volumeHeight = 128.0;

        glm::ivec3 skyBoxMin(0, 120, 0);
        glm::ivec3 skyBoxSize(8, 8, 8);
        auto [skyMin, skyMax] = SVOLandscapeGenerator::estimateDensityBounds(skyBoxMin, skyBoxSize, volumeHeight, config);
        CHECK(skyMax < 0.0);

        glm::ivec3 deepBoxMin(0, 0, 0);
        glm::ivec3 deepBoxSize(8, 8, 8);
        auto [deepMin, deepMax] = SVOLandscapeGenerator::estimateDensityBounds(deepBoxMin, deepBoxSize, volumeHeight, config);
        CHECK(deepMin > 0.0);
    }

    TEST_CASE("SVOLandscape/PerformanceBenchmark", "[SVO][Landscape][.benchmark]")
    {
        LandscapeConfig config;
        config.octaves = 4;
        config.seed    = 1337;

        for (uint32_t res : { 64u, 128u, 256u })
        {
            auto start   = std::chrono::high_resolution_clock::now();
            auto svo     = SVOLandscapeGenerator::generateSVO<bool>(res, config);
            auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();
            REQUIRE(svo != nullptr);
            REQUIRE(svo->getRaw().size() > 0);
            WARN("Generated " << res << "^3 landscape in " << elapsed << " ms (nodes: " << svo->getRaw().size() << ")");
        }
    }
}
#endif
