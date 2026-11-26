#include "CADacomb/Texture/Mapping/TriangulationUVs.h"

namespace CADacomb::Texture
{
    TriangulationUVs::TriangulationUVs() {}

    TriangulationUVs::TriangulationUVs(size_t cornerCount)
      : cornerUVs(cornerCount, glm::dvec2(0.0, 0.0))
    {
    }

    TriangulationUVs::~TriangulationUVs() {}

    size_t TriangulationUVs::getCornerCount() const
    {
        return cornerUVs.size();
    }

    bool TriangulationUVs::isEmpty() const
    {
        return cornerUVs.empty();
    }

    void TriangulationUVs::resize(size_t cornerCount)
    {
        cornerUVs.resize(cornerCount, glm::dvec2(0.0, 0.0));
    }

    const glm::dvec2& TriangulationUVs::getUV(size_t cornerIndex) const
    {
        return cornerUVs[cornerIndex];
    }

    glm::dvec2& TriangulationUVs::getUV(size_t cornerIndex)
    {
        return cornerUVs[cornerIndex];
    }

    void TriangulationUVs::setUV(size_t cornerIndex, const glm::dvec2& uv)
    {
        cornerUVs[cornerIndex] = uv;
    }

    const std::vector<glm::dvec2>& TriangulationUVs::getAllUVs() const
    {
        return cornerUVs;
    }

    void TriangulationUVs::flipV()
    {
        for (auto& uv : cornerUVs)
        {
            uv.y = 1.0 - uv.y;
        }
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("TriangulationUVs/CornerAccessAndFlipV")
{
    CADacomb::Texture::TriangulationUVs uvs(3);
    CHECK(uvs.getCornerCount() == 3);
    CHECK_FALSE(uvs.isEmpty());

    uvs.setUV(0, glm::dvec2(0.0, 0.25));
    uvs.setUV(1, glm::dvec2(1.0, 0.75));
    uvs.setUV(2, glm::dvec2(0.5, 1.0));

    uvs.flipV();
    CHECK(uvs.getUV(0).y == Catch::Approx(0.75));
    CHECK(uvs.getUV(1).y == Catch::Approx(0.25));
    CHECK(uvs.getUV(2).y == Catch::Approx(0.0));
}
#endif
