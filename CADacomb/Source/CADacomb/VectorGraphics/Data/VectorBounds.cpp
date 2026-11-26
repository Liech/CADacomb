#include "CADacomb/VectorGraphics/Data/VectorBounds.h"

#include <algorithm>

namespace CADacomb::VectorGraphics
{
    VectorBounds::VectorBounds() {}

    VectorBounds::VectorBounds(const glm::dvec2& minInput, const glm::dvec2& maxInput)
      : minPoint(minInput)
      , maxPoint(maxInput)
      , valid(true)
    {
    }

    VectorBounds::~VectorBounds() {}

    bool VectorBounds::isValid() const
    {
        return valid;
    }

    const glm::dvec2& VectorBounds::getMin() const
    {
        return minPoint;
    }

    const glm::dvec2& VectorBounds::getMax() const
    {
        return maxPoint;
    }

    glm::dvec2 VectorBounds::getSize() const
    {
        if (!valid)
        {
            return glm::dvec2(0.0, 0.0);
        }
        return maxPoint - minPoint;
    }

    glm::dvec2 VectorBounds::getCenter() const
    {
        if (!valid)
        {
            return glm::dvec2(0.0, 0.0);
        }
        return (minPoint + maxPoint) * 0.5;
    }

    void VectorBounds::expand(const glm::dvec2& point)
    {
        if (!valid)
        {
            minPoint = point;
            maxPoint = point;
            valid    = true;
            return;
        }

        minPoint.x = std::min(minPoint.x, point.x);
        minPoint.y = std::min(minPoint.y, point.y);
        maxPoint.x = std::max(maxPoint.x, point.x);
        maxPoint.y = std::max(maxPoint.y, point.y);
    }

    void VectorBounds::expand(const VectorBounds& other)
    {
        if (!other.valid)
        {
            return;
        }
        expand(other.minPoint);
        expand(other.maxPoint);
    }

    VectorBounds VectorBounds::transformed(const glm::dmat3& matrix) const
    {
        if (!valid)
        {
            return VectorBounds();
        }

        VectorBounds result;
        glm::dvec2   corners[4] = { glm::dvec2(minPoint.x, minPoint.y), glm::dvec2(maxPoint.x, minPoint.y), glm::dvec2(maxPoint.x, maxPoint.y), glm::dvec2(minPoint.x, maxPoint.y) };

        for (int i = 0; i < 4; i++)
        {
            glm::dvec3 p = matrix * glm::dvec3(corners[i], 1.0);
            result.expand(glm::dvec2(p.x, p.y));
        }

        return result;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("VectorBounds/EmptyAndExpand")
{
    CADacomb::VectorGraphics::VectorBounds bounds;
    CHECK_FALSE(bounds.isValid());
    CHECK(bounds.getSize() == glm::dvec2(0.0, 0.0));

    bounds.expand(glm::dvec2(2.0, -3.0));
    bounds.expand(glm::dvec2(10.0, 5.0));
    CHECK(bounds.isValid());
    CHECK(bounds.getMin() == glm::dvec2(2.0, -3.0));
    CHECK(bounds.getMax() == glm::dvec2(10.0, 5.0));
    CHECK(bounds.getSize() == glm::dvec2(8.0, 8.0));
    CHECK(bounds.getCenter() == glm::dvec2(6.0, 1.0));
}

TEST_CASE("VectorBounds/Transformed")
{
    CADacomb::VectorGraphics::VectorBounds bounds(glm::dvec2(0.0, 0.0), glm::dvec2(10.0, 20.0));
    glm::dmat3                             matrix(1.0);
    matrix[0][0] = 2.0;
    matrix[1][1] = 3.0;
    matrix[2][0] = 5.0;
    matrix[2][1] = -4.0;

    CADacomb::VectorGraphics::VectorBounds transformed = bounds.transformed(matrix);
    CHECK(transformed.isValid());
    CHECK(transformed.getMin().x == Catch::Approx(5.0));
    CHECK(transformed.getMin().y == Catch::Approx(-4.0));
    CHECK(transformed.getMax().x == Catch::Approx(25.0));
    CHECK(transformed.getMax().y == Catch::Approx(56.0));
}
#endif
