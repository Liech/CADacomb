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

        if (size > 2)
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

        for (int lz = 0; lz < size; lz++)
        {
            int z  = localMin.z + lz;
            int pz = chunkMin.z + z;
            for (int ly = 0; ly < size; ly++)
            {
                int y  = localMin.y + ly;
                int py = chunkMin.y + y;
                for (int lx = 0; lx < size; lx++)
                {
                    int x  = localMin.x + lx;
                    int px = chunkMin.x + x;

                    glm::dvec3 evalPos = glm::dvec3(px, py, pz) + worldOffset;
                    double     density = evaluateDensity(evalPos, totalVolumeHeight, config);
                    if (density > config.solidThreshold)
                    {
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
        assert(dimension > 0 && (dimension & (dimension - 1)) == 0);

        ProgressBar planningBar(1, "Planning landscape chunks...");

        auto svo = std::make_unique<SVO<T>>(dimension, emptyValue, strategy);

        uint32_t step    = (chunkSize > 0) ? std::min(chunkSize, dimension) : std::min(32u, dimension);
        uint32_t potStep = 1;
        while (potStep * 2 <= step)
        {
            potStep *= 2;
        }
        step = std::max(1u, potStep);

        double totalVolumeHeight = static_cast<double>(dimension);

        struct ChunkTask
        {
            glm::ivec3 min;
            bool       isGuaranteedSolid = false;
        };

        std::vector<ChunkTask> tasks;
        for (uint32_t z = 0; z < dimension; z += step)
        {
            for (uint32_t y = 0; y < dimension; y += step)
            {
                for (uint32_t x = 0; x < dimension; x += step)
                {
                    glm::ivec3 chunkMin(static_cast<int>(x), static_cast<int>(y), static_cast<int>(z));
                    auto [minDensity, maxDensity] = estimateDensityBounds(chunkMin, glm::ivec3(step), totalVolumeHeight, config, worldOffset);

                    if (maxDensity <= config.solidThreshold)
                    {
                        continue;
                    }

                    ChunkTask task;
                    task.min               = chunkMin;
                    task.isGuaranteedSolid = (minDensity > config.solidThreshold && !canCarveCaves(chunkMin, glm::ivec3(step), totalVolumeHeight, config, worldOffset));
                    tasks.push_back(task);
                }
            }
        }

        planningBar.addProgress(1);

        if (tasks.empty())
        {
            return svo;
        }

        std::vector<std::unique_ptr<SVO<T>>> chunkSVOs(tasks.size());

        {
            ProgressBar evalBar(tasks.size(), "Evaluating terrain chunks");

#pragma omp parallel for schedule(dynamic)
            for (int i = 0; i < static_cast<int>(tasks.size()); i++)
            {
                if (evalBar.isCanceled())
                {
                    continue;
                }

                const auto& task = tasks[i];
                if (task.isGuaranteedSolid)
                {
                    chunkSVOs[i] = std::make_unique<SVO<T>>(step, solidValue, svo->getMergeStrategy());
                }
                else
                {
                    size_t         totalChunkVoxels = static_cast<size_t>(step) * step * step;
                    std::vector<T> grid(totalChunkVoxels, emptyValue);

                    fillChunkGridRecursive(grid, glm::ivec3(0), static_cast<int>(step), task.min, static_cast<int>(step), totalVolumeHeight, config, worldOffset, solidValue, emptyValue);

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
                        chunkSVOs[i] = nullptr;
                    }
                    else if (!hasEmpty)
                    {
                        chunkSVOs[i] = std::make_unique<SVO<T>>(step, solidValue, svo->getMergeStrategy());
                    }
                    else
                    {
                        chunkSVOs[i] = std::make_unique<SVO<T>>(grid, step, svo->getMergeStrategy());
                    }
                }

                evalBar.addProgress(1);
            }

            if (evalBar.isCanceled())
            {
                return nullptr;
            }
        }

        {
            ProgressBar assembleBar(tasks.size(), "Assembling octree");
            for (size_t i = 0; i < tasks.size(); i++)
            {
                if (assembleBar.isCanceled())
                {
                    return nullptr;
                }

                if (chunkSVOs[i])
                {
                    svo->setSubSVO(tasks[i].min, *chunkSVOs[i]);
                }
                assembleBar.addProgress(1);
            }
        }

        return svo;
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
