#pragma once

#include "CADacomb/VectorGraphics/Data/VectorSegmentType.h"

#include <glm/glm.hpp>

namespace CADacomb::VectorGraphics
{
    // Single 2D curve segment (line or cubic Bezier) within a VectorSubpath.
    class VectorSegment
    {
      public:
        VectorSegment();
        virtual ~VectorSegment();

        static VectorSegment makeLine(const glm::dvec2& endPoint);
        static VectorSegment makeCubic(const glm::dvec2& controlPoint1, const glm::dvec2& controlPoint2, const glm::dvec2& endPoint);
        static VectorSegment makeQuadratic(const glm::dvec2& startPoint, const glm::dvec2& controlPoint, const glm::dvec2& endPoint);

        VectorSegmentType getType() const;
        const glm::dvec2& getControlPoint1() const;
        const glm::dvec2& getControlPoint2() const;
        const glm::dvec2& getEndPoint() const;

        VectorSegment transformed(const glm::dmat3& matrix) const;

      private:
        static glm::dvec2 transformPoint(const glm::dmat3& matrix, const glm::dvec2& point);

        VectorSegmentType type          = VectorSegmentType::Line;
        glm::dvec2        controlPoint1 = glm::dvec2(0.0, 0.0);
        glm::dvec2        controlPoint2 = glm::dvec2(0.0, 0.0);
        glm::dvec2        endPoint      = glm::dvec2(0.0, 0.0);
    };
}
