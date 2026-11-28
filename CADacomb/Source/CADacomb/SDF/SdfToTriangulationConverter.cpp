#include "CADacomb/SDF/SdfToTriangulationConverter.h"

#include "CADacomb/Formula/FormulaLuaEvaluator.h"
#include "CADacomb/Volume/Flat/SurfaceNets.h"

#include <algorithm>
#include <vector>

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace CADacomb
{
    SdfToTriangulationConverter::SdfToTriangulationConverter() {}

    SdfToTriangulationConverter::~SdfToTriangulationConverter() {}

    std::unique_ptr<Triangulation> SdfToTriangulationConverter::convert(const Sdf& sdf, uint32_t resolution, std::string& outError)
    {
        outError.clear();
        if (resolution < 2)
        {
            outError = "Resolution must be at least 2.";
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

        const uint64_t   gridDim   = static_cast<uint64_t>(resolution) + 1;
        const glm::dvec3 origin    = sdf.getAABB().getMin();
        const glm::dvec3 voxelSize = sdf.getAABB().getSize() / static_cast<double>(resolution);

        std::vector<double> field(static_cast<size_t>(gridDim * gridDim * gridDim), 1.0);

#if defined(_OPENMP)
#pragma omp parallel for schedule(dynamic)
#endif
        for (int64_t z = 0; z < static_cast<int64_t>(gridDim); z++)
        {
            size_t tid = 0;
#if defined(_OPENMP)
            tid = static_cast<size_t>(omp_get_thread_num());
#endif
            double pz = origin.z + static_cast<double>(z) * voxelSize.z;
            for (uint64_t y = 0; y < gridDim; y++)
            {
                double py = origin.y + static_cast<double>(y) * voxelSize.y;
                for (uint64_t x = 0; x < gridDim; x++)
                {
                    double             px  = origin.x + static_cast<double>(x) * voxelSize.x;
                    std::vector<float> res = evaluators[tid].evaluate({ static_cast<float>(px), static_cast<float>(py), static_cast<float>(pz) });
                    size_t             idx = static_cast<size_t>((static_cast<uint64_t>(z) * gridDim + y) * gridDim + x);
                    field[idx]             = res.empty() ? 1.0 : static_cast<double>(res[0]);
                }
            }
        }

        return SurfaceNets::triangulate(field, origin, voxelSize, glm::u64vec3(gridDim, gridDim, gridDim), 0.0);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("SdfToTriangulationConverter/ConvertsSphereSdfToMesh")
{
    CADacomb::Sdf sdf;
    std::string   error;
    auto          mesh = CADacomb::SdfToTriangulationConverter::convert(sdf, 16, error);

    REQUIRE(mesh != nullptr);
    CHECK(error.empty());
    CHECK(mesh->vertices.size() > 0);
    CHECK(mesh->indices.size() > 0);
}
#endif
