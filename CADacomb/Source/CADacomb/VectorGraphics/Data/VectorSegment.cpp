#include "CADacomb/VectorGraphics/Data/VectorSegment.h"

namespace CADacomb::VectorGraphics
{
    VectorSegment::VectorSegment() {}

    VectorSegment::~VectorSegment() {}

    VectorSegment VectorSegment::makeLine(const glm::dvec2& endPointInput)
    {
        VectorSegment segment;
        segment.type     = VectorSegmentType::Line;
        segment.endPoint = endPointInput;
        return segment;
    }

    VectorSegment VectorSegment::makeCubic(const glm::dvec2& controlPoint1Input, const glm::dvec2& controlPoint2Input, const glm::dvec2& endPointInput)
    {
        VectorSegment segment;
        segment.type          = VectorSegmentType::CubicBezier;
        segment.controlPoint1 = controlPoint1Input;
        segment.controlPoint2 = controlPoint2Input;
        segment.endPoint      = endPointInput;
        return segment;
    }

    VectorSegment VectorSegment::makeQuadratic(const glm::dvec2& startPoint, const glm::dvec2& controlPoint, const glm::dvec2& endPointInput)
    {
        constexpr double twoThirds = 2.0 / 3.0;
        glm::dvec2       cp1       = startPoint + twoThirds * (controlPoint - startPoint);
        glm::dvec2       cp2       = endPointInput + twoThirds * (controlPoint - endPointInput);
        return makeCubic(cp1, cp2, endPointInput);
    }

    VectorSegmentType VectorSegment::getType() const
    {
        return type;
    }

    const glm::dvec2& VectorSegment::getControlPoint1() const
    {
        return controlPoint1;
    }

    const glm::dvec2& VectorSegment::getControlPoint2() const
    {
        return controlPoint2;
    }

    const glm::dvec2& VectorSegment::getEndPoint() const
    {
        return endPoint;
    }

    VectorSegment VectorSegment::transformed(const glm::dmat3& matrix) const
    {
        if (type == VectorSegmentType::Line)
        {
            return makeLine(transformPoint(matrix, endPoint));
        }
        return makeCubic(transformPoint(matrix, controlPoint1), transformPoint(matrix, controlPoint2), transformPoint(matrix, endPoint));
    }

    glm::dvec2 VectorSegment::transformPoint(const glm::dmat3& matrix, const glm::dvec2& point)
    {
        glm::dvec3 p = matrix * glm::dvec3(point, 1.0);
        return glm::dvec2(p.x, p.y);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("VectorSegment/QuadraticElevation")
{
    CADacomb::VectorGraphics::VectorSegment segment = CADacomb::VectorGraphics::VectorSegment::makeQuadratic(glm::dvec2(0.0, 0.0), glm::dvec2(3.0, 6.0), glm::dvec2(6.0, 0.0));
    CHECK(segment.getType() == CADacomb::VectorGraphics::VectorSegmentType::CubicBezier);
    CHECK(segment.getControlPoint1().x == Catch::Approx(2.0));
    CHECK(segment.getControlPoint1().y == Catch::Approx(4.0));
    CHECK(segment.getControlPoint2().x == Catch::Approx(4.0));
    CHECK(segment.getControlPoint2().y == Catch::Approx(4.0));
    CHECK(segment.getEndPoint() == glm::dvec2(6.0, 0.0));
}

TEST_CASE("VectorSegment/TransformLine")
{
    CADacomb::VectorGraphics::VectorSegment segment = CADacomb::VectorGraphics::VectorSegment::makeLine(glm::dvec2(4.0, 5.0));
    glm::dmat3                              matrix(1.0);
    matrix[2][0] = 10.0;
    matrix[2][1] = -2.0;

    CADacomb::VectorGraphics::VectorSegment transformed = segment.transformed(matrix);
    CHECK(transformed.getType() == CADacomb::VectorGraphics::VectorSegmentType::Line);
    CHECK(transformed.getEndPoint().x == Catch::Approx(14.0));
    CHECK(transformed.getEndPoint().y == Catch::Approx(3.0));
}
#endif
