#pragma once

#include "CADacomb/Util/ProgressBar.h"
#include "CADacomb/Volume/Flat/BinaryVolume.h"
#include "CADacomb/Volume/MergeStrategy/MostPrevalentExceptMergeStrategy.h"
#include "CADacomb/Volume/SVO/BinaryVolumeToSVO.h"
#include "CADacomb/Volume/SVO/SVO.h"
#include "LandscapeConfig.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace CADacomb
{
    class SVOLandscapeGenerator
    {
      public:
        SVOLandscapeGenerator();
        virtual ~SVOLandscapeGenerator();

        static double perlinNoise3D(const glm::dvec3& pos, uint32_t seed = 0);
        static double fbm3D(const glm::dvec3& pos, int octaves, double frequency, double persistence, double lacunarity, uint32_t seed = 0);
        static double evaluateDensity(const glm::dvec3& localPos, double volumeHeight, const LandscapeConfig& config);

        static std::pair<double, double> estimateDensityBounds(const glm::ivec3&      boxMin,
                                                               const glm::ivec3&      boxSize,
                                                               double                 volumeHeight,
                                                               const LandscapeConfig& config,
                                                               const glm::dvec3&      worldOffset = glm::dvec3(0.0));

        static bool canCarveCaves(const glm::ivec3& boxMin, const glm::ivec3& boxSize, double volumeHeight, const LandscapeConfig& config, const glm::dvec3& worldOffset = glm::dvec3(0.0));

        static CADacomb::BinaryVolume generateBinaryVolume(const glm::u64vec3&    dimension,
                                                           const LandscapeConfig& config      = {},
                                                           const glm::dvec3&      worldOffset = glm::dvec3(0.0),
                                                           const glm::dvec3&      worldSize   = glm::dvec3(1.0));

        static void evaluateDensityBlock(const glm::ivec3&      blockMin,
                                         uint32_t               blockSize,
                                         double                 volumeHeight,
                                         const LandscapeConfig& config,
                                         const glm::dvec3&      worldOffset,
                                         double*                outDensities);

        template<typename T = bool>
        static std::unique_ptr<SVO<T>> generateSVO(uint32_t                dimension,
                                                   const LandscapeConfig&  config      = {},
                                                   const glm::dvec3&       worldOffset = glm::dvec3(0.0),
                                                   T                       solidValue  = static_cast<T>(true),
                                                   T                       emptyValue  = static_cast<T>(false),
                                                   uint32_t                chunkSize   = 32,
                                                   const MergeStrategy<T>& strategy    = MostPrevalentExceptMergeStrategy<T, false>::instance());

        template<typename T>
        static void fillSVOChunk(SVO<T>&                svo,
                                 const glm::ivec3&      chunkMin,
                                 const glm::ivec3&      chunkSize,
                                 double                 totalVolumeHeight,
                                 const LandscapeConfig& config,
                                 const glm::dvec3&      worldOffset = glm::dvec3(0.0),
                                 T                      solidValue  = static_cast<T>(true));

      private:
        static int      fastFloor(double x);
        static uint32_t hash3D(int x, int y, int z, uint32_t seed);
        static double   grad(uint32_t hash, double x, double y, double z);
        static double   fade(double t);
        static double   lerp(double a, double b, double t);
        static void     addPerlinOctaveBlock(const glm::ivec3& blockMin,
                                             uint32_t          blockSize,
                                             const glm::dvec3& worldOffset,
                                             double            freq,
                                             double            weight,
                                             uint32_t          seed,
                                             double*           outValues);

        template<typename T>
        static void fillChunkGridRecursive(std::vector<T>&        grid,
                                           const glm::ivec3&      localMin,
                                           int                    size,
                                           const glm::ivec3&      chunkMin,
                                           int                    chunkStep,
                                           double                 totalVolumeHeight,
                                           const LandscapeConfig& config,
                                           const glm::dvec3&      worldOffset,
                                           T                      solidValue,
                                           T                      emptyValue);
    };

    template<typename T>
    void SVOLandscapeGenerator::fillChunkGridRecursive(std::vector<T>&        grid,
                                                       const glm::ivec3&      localMin,
                                                       int                    size,
                                                       const glm::ivec3&      chunkMin,
                                                       int                    chunkStep,
                                                       double                 totalVolumeHeight,
                                                       const LandscapeConfig& config,
                                                       const glm::dvec3&      worldOffset,
                                                       T                      solidValue,
                                                       T                      emptyValue)
    {
        glm::ivec3 boxMin = chunkMin + localMin;
        glm::ivec3 boxSize(size);

        auto [minDensity, maxDensity] = estimateDensityBounds(boxMin, boxSize, totalVolumeHeight, config, worldOffset);

        if (maxDensity <= config.solidThreshold)
        {
            return;
        }

        if (minDensity > config.solidThreshold && !canCarveCaves(boxMin, boxSize, totalVolumeHeight, config, worldOffset))
        {
            for (int lz = 0; lz < size; lz++)
            {
                int z = localMin.z + lz;
                for (int ly = 0; ly < size; ly++)
                {
                    int    y        = localMin.y + ly;
                    size_t rowStart = static_cast<size_t>(localMin.x + y * chunkStep + z * chunkStep * chunkStep);
                    std::fill_n(grid.begin() + rowStart, size, solidValue);
                }
            }
            return;
        }

        if (size > 4)
        {
            int half = size / 2;
            for (int cz = 0; cz < 2; cz++)
            {
                for (int cy = 0; cy < 2; cy++)
                {
                    for (int cx = 0; cx < 2; cx++)
                    {
                        glm::ivec3 childLocalMin = localMin + glm::ivec3(cx * half, cy * half, cz * half);
                        fillChunkGridRecursive(grid, childLocalMin, half, chunkMin, chunkStep, totalVolumeHeight, config, worldOffset, solidValue, emptyValue);
                    }
                }
            }
            return;
        }

        double densities[64];
        evaluateDensityBlock(boxMin, static_cast<uint32_t>(size), totalVolumeHeight, config, worldOffset, densities);

        for (int lz = 0; lz < size; lz++)
        {
            int z = localMin.z + lz;
            for (int ly = 0; ly < size; ly++)
            {
                int y = localMin.y + ly;
                for (int lx = 0; lx < size; lx++)
                {
                    size_t blockIdx = static_cast<size_t>(lx + ly * size + lz * size * size);
                    if (densities[blockIdx] > config.solidThreshold)
                    {
                        int    x   = localMin.x + lx;
                        size_t idx = static_cast<size_t>(x + y * chunkStep + z * chunkStep * chunkStep);
                        grid[idx]  = solidValue;
                    }
                }
            }
        }
    }

    template<typename T>
    std::unique_ptr<SVO<T>> SVOLandscapeGenerator::generateSVO(uint32_t                dimension,
                                                               const LandscapeConfig&  config,
                                                               const glm::dvec3&       worldOffset,
                                                               T                       solidValue,
                                                               T                       emptyValue,
                                                               uint32_t                chunkSize,
                                                               const MergeStrategy<T>& strategy)
    {
        (void)chunkSize;
        assert(dimension > 0 && (dimension & (dimension - 1)) == 0);

        double totalVolumeHeight = static_cast<double>(dimension);

        auto classifyBox = [&](const glm::ivec3& boxMin, uint32_t boxSize, T& outUniformVal) -> bool
        {
            glm::ivec3 sizeVec(static_cast<int>(boxSize));
            auto [minDensity, maxDensity] = estimateDensityBounds(boxMin, sizeVec, totalVolumeHeight, config, worldOffset);

            if (maxDensity <= config.solidThreshold)
            {
                outUniformVal = emptyValue;
                return true;
            }

            if (minDensity > config.solidThreshold && !canCarveCaves(boxMin, sizeVec, totalVolumeHeight, config, worldOffset))
            {
                outUniformVal = solidValue;
                return true;
            }

            return false;
        };

        auto evalBlock = [&](const glm::ivec3& blockMin, uint32_t blockSize, T* outBlock) -> void
        {
            double densities[64];
            evaluateDensityBlock(blockMin, blockSize, totalVolumeHeight, config, worldOffset, densities);
            size_t count = static_cast<size_t>(blockSize) * blockSize * blockSize;
            for (size_t i = 0; i < count; i++)
            {
                outBlock[i] = (densities[i] > config.solidThreshold) ? solidValue : emptyValue;
            }
        };

        return SVO<T>::buildTopDown(dimension, emptyValue, strategy, classifyBox, evalBlock, "Generating landscape SVO");
    }

    template<typename T>
    void SVOLandscapeGenerator::fillSVOChunk(SVO<T>&                svo,
                                             const glm::ivec3&      chunkMin,
                                             const glm::ivec3&      chunkSize,
                                             double                 totalVolumeHeight,
                                             const LandscapeConfig& config,
                                             const glm::dvec3&      worldOffset,
                                             T                      solidValue)
    {
        if (chunkSize.x <= 0 || chunkSize.y <= 0 || chunkSize.z <= 0)
        {
            return;
        }

        T emptyValue = static_cast<T>(false);

        auto [minDensity, maxDensity] = estimateDensityBounds(chunkMin, chunkSize, totalVolumeHeight, config, worldOffset);
        if (maxDensity <= config.solidThreshold)
        {
            return;
        }

        bool isAlignedCube = (chunkSize.x == chunkSize.y && chunkSize.y == chunkSize.z && (chunkSize.x & (chunkSize.x - 1)) == 0 && (chunkMin.x % chunkSize.x == 0) &&
                              (chunkMin.y % chunkSize.x == 0) && (chunkMin.z % chunkSize.x == 0));

        uint32_t step = static_cast<uint32_t>(chunkSize.x);

        if (isAlignedCube && minDensity > config.solidThreshold && !canCarveCaves(chunkMin, chunkSize, totalVolumeHeight, config, worldOffset))
        {
            SVO<T> solidChunk(step, solidValue, svo.getMergeStrategy());
            svo.setSubSVO(chunkMin, solidChunk);
            return;
        }

        if (isAlignedCube)
        {
            size_t         totalChunkVoxels = static_cast<size_t>(step) * step * step;
            std::vector<T> grid(totalChunkVoxels, emptyValue);

            fillChunkGridRecursive(grid, glm::ivec3(0), static_cast<int>(step), chunkMin, static_cast<int>(step), totalVolumeHeight, config, worldOffset, solidValue, emptyValue);

            bool hasSolid = false;
            bool hasEmpty = false;
            for (size_t v = 0; v < totalChunkVoxels; v++)
            {
                if (grid[v] == solidValue)
                {
                    hasSolid = true;
                    if (hasEmpty)
                    {
                        break;
                    }
                }
                else
                {
                    hasEmpty = true;
                    if (hasSolid)
                    {
                        break;
                    }
                }
            }

            if (!hasSolid)
            {
                return;
            }

            if (!hasEmpty)
            {
                SVO<T> solidChunk(step, solidValue, svo.getMergeStrategy());
                svo.setSubSVO(chunkMin, solidChunk);
                return;
            }

            SVO<T> chunk(grid, step, svo.getMergeStrategy());
            svo.setSubSVO(chunkMin, chunk);
        }
        else
        {
            const int svoDim = static_cast<int>(svo.getDimension());
            for (int z = 0; z < chunkSize.z; z++)
            {
                int pz = chunkMin.z + z;
                if (pz < 0 || pz >= svoDim)
                {
                    continue;
                }

                for (int y = 0; y < chunkSize.y; y++)
                {
                    int py = chunkMin.y + y;
                    if (py < 0 || py >= svoDim)
                    {
                        continue;
                    }

                    for (int x = 0; x < chunkSize.x; x++)
                    {
                        int px = chunkMin.x + x;
                        if (px < 0 || px >= svoDim)
                        {
                            continue;
                        }

                        glm::ivec3 pos(px, py, pz);
                        glm::dvec3 evalPos = glm::dvec3(pos) + worldOffset;
                        double     density = evaluateDensity(evalPos, totalVolumeHeight, config);
                        if (density > config.solidThreshold)
                        {
                            svo.setVoxel(pos, solidValue);
                        }
                    }
                }
            }
        }
    }
}
