#include "CADacomb/VectorGraphics/Data/VectorSubpath.h"

#include <algorithm>

namespace CADacomb::VectorGraphics
{
    VectorSubpath::VectorSubpath() {}

    VectorSubpath::VectorSubpath(const glm::dvec2& startPointInput)
      : startPoint(startPointInput)
    {
    }

    VectorSubpath::~VectorSubpath() {}

    const glm::dvec2& VectorSubpath::getStartPoint() const
    {
        return startPoint;
    }

    void VectorSubpath::setStartPoint(const glm::dvec2& point)
    {
        startPoint = point;
    }

    const std::vector<VectorSegment>& VectorSubpath::getSegments() const
    {
        return segments;
    }

    bool VectorSubpath::isClosed() const
    {
        return closed;
    }

    void VectorSubpath::setClosed(bool value)
    {
        closed = value;
    }

    bool VectorSubpath::isEmpty() const
    {
        return segments.empty();
    }

    glm::dvec2 VectorSubpath::getCurrentPoint() const
    {
        if (segments.empty())
        {
            return startPoint;
        }
        return segments.back().getEndPoint();
    }

    void VectorSubpath::lineTo(const glm::dvec2& endPoint)
    {
        segments.push_back(VectorSegment::makeLine(endPoint));
    }

    void VectorSubpath::cubicTo(const glm::dvec2& controlPoint1, const glm::dvec2& controlPoint2, const glm::dvec2& endPoint)
    {
        segments.push_back(VectorSegment::makeCubic(controlPoint1, controlPoint2, endPoint));
    }

    void VectorSubpath::quadraticTo(const glm::dvec2& controlPoint, const glm::dvec2& endPoint)
    {
        segments.push_back(VectorSegment::makeQuadratic(getCurrentPoint(), controlPoint, endPoint));
    }

    void VectorSubpath::close()
    {
        closed = true;
    }

    std::vector<glm::dvec2> VectorSubpath::flatten(double tolerance) const
    {
        std::vector<glm::dvec2> points;
        points.push_back(startPoint);

        double     safeTolerance = std::max(tolerance, 1e-6);
        double     toleranceSq   = safeTolerance * safeTolerance;
        glm::dvec2 current       = startPoint;

        for (const VectorSegment& segment : segments)
        {
            if (segment.getType() == VectorSegmentType::Line)
            {
                points.push_back(segment.getEndPoint());
            }
            else
            {
                flattenCubic(current, segment.getControlPoint1(), segment.getControlPoint2(), segment.getEndPoint(), toleranceSq, 0, points);
            }
            current = segment.getEndPoint();
        }

        return points;
    }

    VectorSubpath VectorSubpath::transformed(const glm::dmat3& matrix) const
    {
        glm::dvec3    transformedStart = matrix * glm::dvec3(startPoint, 1.0);
        VectorSubpath result(glm::dvec2(transformedStart.x, transformedStart.y));
        result.closed = closed;
        result.segments.reserve(segments.size());

        for (const VectorSegment& segment : segments)
        {
            result.segments.push_back(segment.transformed(matrix));
        }

        return result;
    }

    VectorBounds VectorSubpath::computeBounds(double tolerance) const
    {
        VectorBounds            bounds;
        std::vector<glm::dvec2> points = flatten(tolerance);
        for (const glm::dvec2& point : points)
        {
            bounds.expand(point);
        }
        return bounds;
    }

    void VectorSubpath::flattenCubic(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, double toleranceSq, int depth, std::vector<glm::dvec2>& output)
    {
        constexpr int maxDepth = 12;

        glm::dvec2 d       = p3 - p0;
        double     len2    = glm::dot(d, d);
        double     dist1Sq = 0.0;
        double     dist2Sq = 0.0;

        if (len2 > 1e-12)
        {
            double cross1 = d.x * (p1.y - p0.y) - d.y * (p1.x - p0.x);
            double cross2 = d.x * (p2.y - p0.y) - d.y * (p2.x - p0.x);
            dist1Sq       = (cross1 * cross1) / len2;
            dist2Sq       = (cross2 * cross2) / len2;
        }
        else
        {
            glm::dvec2 v1 = p1 - p0;
            glm::dvec2 v2 = p2 - p0;
            dist1Sq       = glm::dot(v1, v1);
            dist2Sq       = glm::dot(v2, v2);
        }

        if ((dist1Sq <= toleranceSq && dist2Sq <= toleranceSq) || depth >= maxDepth)
        {
            output.push_back(p3);
            return;
        }

        glm::dvec2 p01   = (p0 + p1) * 0.5;
        glm::dvec2 p12   = (p1 + p2) * 0.5;
        glm::dvec2 p23   = (p2 + p3) * 0.5;
        glm::dvec2 p012  = (p01 + p12) * 0.5;
        glm::dvec2 p123  = (p12 + p23) * 0.5;
        glm::dvec2 p0123 = (p012 + p123) * 0.5;

        flattenCubic(p0, p01, p012, p0123, toleranceSq, depth + 1, output);
        flattenCubic(p0123, p123, p23, p3, toleranceSq, depth + 1, output);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("VectorSubpath/LineFlattenAndBounds")
{
    CADacomb::VectorGraphics::VectorSubpath subpath(glm::dvec2(0.0, 0.0));
    subpath.lineTo(glm::dvec2(10.0, 0.0));
    subpath.lineTo(glm::dvec2(10.0, 5.0));
    subpath.close();

    CHECK(subpath.isClosed());
    CHECK(subpath.getSegments().size() == 2);

    std::vector<glm::dvec2> flat = subpath.flatten(0.1);
    CHECK(flat.size() == 3);
    CHECK(flat[0] == glm::dvec2(0.0, 0.0));
    CHECK(flat[2] == glm::dvec2(10.0, 5.0));

    CADacomb::VectorGraphics::VectorBounds bounds = subpath.computeBounds(0.1);
    CHECK(bounds.isValid());
    CHECK(bounds.getMin() == glm::dvec2(0.0, 0.0));
    CHECK(bounds.getMax() == glm::dvec2(10.0, 5.0));
}

TEST_CASE("VectorSubpath/CubicFlattenSubdivides")
{
    CADacomb::VectorGraphics::VectorSubpath subpath(glm::dvec2(0.0, 0.0));
    subpath.cubicTo(glm::dvec2(0.0, 10.0), glm::dvec2(10.0, 10.0), glm::dvec2(10.0, 0.0));

    std::vector<glm::dvec2> flat = subpath.flatten(0.1);
    CHECK(flat.size() > 4);
    CHECK(flat.front() == glm::dvec2(0.0, 0.0));
    CHECK(flat.back() == glm::dvec2(10.0, 0.0));

    CADacomb::VectorGraphics::VectorBounds bounds = subpath.computeBounds(0.05);
    CHECK(bounds.getMax().y == Catch::Approx(7.5).margin(0.1));
}
#endif
