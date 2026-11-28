#include "CADacomb/SDF/SdfToSVOConverter.h"

#include "CADacomb/Formula/FormulaLuaEvaluator.h"
#include "CADacomb/Volume/MergeStrategy/MostPrevalentExceptMergeStrategy.h"

#include <algorithm>
#include <cmath>
#include <vector>

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace CADacomb
{
    SdfToSVOConverter::SdfToSVOConverter() {}

    SdfToSVOConverter::~SdfToSVOConverter() {}

    std::unique_ptr<SVO<bool>> SdfToSVOConverter::convert(const Sdf& sdf, uint32_t dimension, std::string& outError)
    {
        outError.clear();
        if (dimension == 0 || (dimension & (dimension - 1)) != 0)
        {
            outError = "Dimension must be a positive power of two.";
            return nullptr;
        }

        int threadCount = 1;
#if defined(_OPENMP)
        threadCount = std::max(1, omp_get_max_threads());
#endif

        std::vector<FormulaLuaEvaluator> evaluators(static_cast<size_t>(threadCount));
        for (size_t i = 0; i < evaluators.size(); i++)
        {
            if (!evaluators[i].compile(sdf.getFormula(), outError))
            {
                return nullptr;
            }
        }

        const glm::dvec3 aabbMin   = sdf.getAABB().getMin();
        const glm::dvec3 aabbSize  = sdf.getAABB().getSize();
        const glm::dvec3 voxelSize = aabbSize / static_cast<double>(dimension);
        const double     diagScale = 0.5 * glm::length(voxelSize);

        auto getThreadIndex = []() -> size_t
        {
#if defined(_OPENMP)
            return static_cast<size_t>(omp_get_thread_num());
#else
            return 0;
#endif
        };

        auto classifyBox = [&](const glm::ivec3& boxMin, uint32_t boxSize, bool& outUniformVal) -> bool
        {
            size_t     tid    = getThreadIndex();
            glm::dvec3 center = aabbMin + (glm::dvec3(boxMin) + glm::dvec3(0.5 * static_cast<double>(boxSize))) * voxelSize;
            double     radius = static_cast<double>(boxSize) * diagScale;

            std::vector<float> res = evaluators[tid].evaluate({ static_cast<float>(center.x), static_cast<float>(center.y), static_cast<float>(center.z) });
            if (res.empty())
            {
                return false;
            }

            double dist = static_cast<double>(res[0]);
            if (dist > radius)
            {
                outUniformVal = false;
                return true;
            }
            if (dist < -radius)
            {
                outUniformVal = true;
                return true;
            }
            return false;
        };

        auto evalBlock = [&](const glm::ivec3& blockMin, uint32_t blockSize, bool* outBlock) -> void
        {
            size_t tid    = getThreadIndex();
            size_t outIdx = 0;

            for (uint32_t lz = 0; lz < blockSize; lz++)
            {
                double pz = aabbMin.z + (static_cast<double>(blockMin.z + static_cast<int>(lz)) + 0.5) * voxelSize.z;
                for (uint32_t ly = 0; ly < blockSize; ly++)
                {
                    double py = aabbMin.y + (static_cast<double>(blockMin.y + static_cast<int>(ly)) + 0.5) * voxelSize.y;
                    for (uint32_t lx = 0; lx < blockSize; lx++)
                    {
                        double             px  = aabbMin.x + (static_cast<double>(blockMin.x + static_cast<int>(lx)) + 0.5) * voxelSize.x;
                        std::vector<float> res = evaluators[tid].evaluate({ static_cast<float>(px), static_cast<float>(py), static_cast<float>(pz) });
                        outBlock[outIdx]       = !res.empty() && res[0] < 0.0f;
                        outIdx++;
                    }
                }
            }
        };

        return SVO<bool>::buildTopDown(dimension, false, MostPrevalentExceptMergeStrategy<bool, false>::instance(), classifyBox, evalBlock, "Converting SDF to SVO");
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("SdfToSVOConverter/ConvertsSphereSdf")
{
    CADacomb::Sdf sdf;
    std::string   error;
    auto          svo = CADacomb::SdfToSVOConverter::convert(sdf, 16, error);

    REQUIRE(svo != nullptr);
    CHECK(error.empty());
    CHECK(svo->getVoxel({ 8, 8, 8 }) == true);
    CHECK(svo->getVoxel({ 0, 0, 0 }) == false);
}
#endif
