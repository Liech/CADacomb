#include "AABB3.h"
#include <algorithm>

namespace CADacomb
{
    AABB3::AABB3() {}

    AABB3::AABB3(const glm::dvec3& minPoint, const glm::dvec3& maxPoint)
      : minPoint(minPoint)
      , maxPoint(maxPoint)
    {
    }

    AABB3::~AABB3() {}

    const glm::dvec3& AABB3::getMin() const
    {
        return minPoint;
    }

    const glm::dvec3& AABB3::getMax() const
    {
        return maxPoint;
    }

    void AABB3::setMin(const glm::dvec3& newMinPoint)
    {
        minPoint = newMinPoint;
    }

    void AABB3::setMax(const glm::dvec3& newMaxPoint)
    {
        maxPoint = newMaxPoint;
    }

    glm::dvec3 AABB3::getSize() const
    {
        return maxPoint - minPoint;
    }

    glm::dvec3 AABB3::getCenter() const
    {
        return (minPoint + maxPoint) * 0.5;
    }

    bool AABB3::contains(const glm::dvec3& point) const
    {
        return point.x >= minPoint.x && point.x <= maxPoint.x && point.y >= minPoint.y && point.y <= maxPoint.y && point.z >= minPoint.z && point.z <= maxPoint.z;
    }

    bool AABB3::contains(const AABB3& other) const
    {
        return other.minPoint.x >= minPoint.x && other.maxPoint.x <= maxPoint.x && other.minPoint.y >= minPoint.y && other.maxPoint.y <= maxPoint.y && other.minPoint.z >= minPoint.z &&
               other.maxPoint.z <= maxPoint.z;
    }

    bool AABB3::intersects(const AABB3& other) const
    {
        return minPoint.x <= other.maxPoint.x && maxPoint.x >= other.minPoint.x && minPoint.y <= other.maxPoint.y && maxPoint.y >= other.minPoint.y && minPoint.z <= other.maxPoint.z &&
               maxPoint.z >= other.minPoint.z;
    }

    void AABB3::expand(const glm::dvec3& point)
    {
        minPoint = glm::min(minPoint, point);
        maxPoint = glm::max(maxPoint, point);
    }

    void AABB3::expand(const AABB3& other)
    {
        minPoint = glm::min(minPoint, other.minPoint);
        maxPoint = glm::max(maxPoint, other.maxPoint);
    }

    glm::dvec3 AABB3::closestPoint(const glm::dvec3& point) const
    {
        return glm::clamp(point, minPoint, maxPoint);
    }

    double AABB3::distanceTo(const glm::dvec3& point) const
    {
        return glm::length(point - closestPoint(point));
    }

    double AABB3::squaredDistanceTo(const glm::dvec3& point) const
    {
        glm::dvec3 diff = point - closestPoint(point);
        return glm::dot(diff, diff);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("AABB3/ConstructionAndGetters", "[AABB3]")
    {
        AABB3 box(glm::dvec3(0.0, 1.0, 2.0), glm::dvec3(10.0, 11.0, 12.0));

        CHECK(box.getMin() == glm::dvec3(0.0, 1.0, 2.0));
        CHECK(box.getMax() == glm::dvec3(10.0, 11.0, 12.0));
        CHECK(box.getSize() == glm::dvec3(10.0, 10.0, 10.0));
        CHECK(box.getCenter() == glm::dvec3(5.0, 6.0, 7.0));

        box.setMin(glm::dvec3(-1.0, -1.0, -1.0));
        box.setMax(glm::dvec3(1.0, 1.0, 1.0));
        CHECK(box.getSize() == glm::dvec3(2.0, 2.0, 2.0));
        CHECK(box.getCenter() == glm::dvec3(0.0, 0.0, 0.0));
    }

    TEST_CASE("AABB3/ContainsAndIntersects", "[AABB3]")
    {
        AABB3 box(glm::dvec3(0.0), glm::dvec3(10.0));

        CHECK(box.contains(glm::dvec3(5.0)));
        CHECK(box.contains(glm::dvec3(0.0)));
        CHECK(box.contains(glm::dvec3(10.0)));
        CHECK_FALSE(box.contains(glm::dvec3(-0.1, 5.0, 5.0)));
        CHECK_FALSE(box.contains(glm::dvec3(10.1, 5.0, 5.0)));

        AABB3 insideBox(glm::dvec3(2.0), glm::dvec3(8.0));
        AABB3 overlappingBox(glm::dvec3(8.0), glm::dvec3(15.0));
        AABB3 separateBox(glm::dvec3(20.0), glm::dvec3(30.0));

        CHECK(box.contains(insideBox));
        CHECK_FALSE(box.contains(overlappingBox));
        CHECK(box.intersects(overlappingBox));
        CHECK_FALSE(box.intersects(separateBox));
    }

    TEST_CASE("AABB3/Expand", "[AABB3]")
    {
        AABB3 box(glm::dvec3(0.0), glm::dvec3(5.0));

        box.expand(glm::dvec3(-2.0, 3.0, 8.0));
        CHECK(box.getMin() == glm::dvec3(-2.0, 0.0, 0.0));
        CHECK(box.getMax() == glm::dvec3(5.0, 5.0, 8.0));

        AABB3 other(glm::dvec3(-5.0, -1.0, 2.0), glm::dvec3(6.0, 7.0, 10.0));
        box.expand(other);
        CHECK(box.getMin() == glm::dvec3(-5.0, -1.0, 0.0));
        CHECK(box.getMax() == glm::dvec3(6.0, 7.0, 10.0));
    }

    TEST_CASE("AABB3/ClosestPointAndDistance", "[AABB3]")
    {
        AABB3 box(glm::dvec3(0.0), glm::dvec3(10.0));

        glm::dvec3 insidePoint(5.0, 5.0, 5.0);
        CHECK(box.closestPoint(insidePoint) == insidePoint);
        CHECK(box.distanceTo(insidePoint) == 0.0);
        CHECK(box.squaredDistanceTo(insidePoint) == 0.0);

        glm::dvec3 outsidePoint(15.0, 5.0, 5.0);
        CHECK(box.closestPoint(outsidePoint) == glm::dvec3(10.0, 5.0, 5.0));
        CHECK(box.distanceTo(outsidePoint) == 5.0);
        CHECK(box.squaredDistanceTo(outsidePoint) == 25.0);
    }
}
#endif
