#pragma once

#include "CADacomb/VectorGraphics/Data/VectorBounds.h"
#include "CADacomb/VectorGraphics/Data/VectorSegment.h"

#include <glm/glm.hpp>
#include <vector>

namespace CADacomb::VectorGraphics
{
    // Contiguous 2D contour made of line and cubic Bezier segments, either open or closed.
    class VectorSubpath
    {
      public:
        VectorSubpath();
        explicit VectorSubpath(const glm::dvec2& startPoint);
        virtual ~VectorSubpath();

        const glm::dvec2& getStartPoint() const;
        void              setStartPoint(const glm::dvec2& point);

        const std::vector<VectorSegment>& getSegments() const;
        bool                              isClosed() const;
        void                              setClosed(bool value);
        bool                              isEmpty() const;
        glm::dvec2                        getCurrentPoint() const;

        void lineTo(const glm::dvec2& endPoint);
        void cubicTo(const glm::dvec2& controlPoint1, const glm::dvec2& controlPoint2, const glm::dvec2& endPoint);
        void quadraticTo(const glm::dvec2& controlPoint, const glm::dvec2& endPoint);
        void close();

        std::vector<glm::dvec2> flatten(double tolerance = 0.25) const;
        VectorSubpath           transformed(const glm::dmat3& matrix) const;
        VectorBounds            computeBounds(double tolerance = 0.25) const;

      private:
        static void flattenCubic(const glm::dvec2& p0, const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, double toleranceSq, int depth, std::vector<glm::dvec2>& output);

        glm::dvec2                 startPoint = glm::dvec2(0.0, 0.0);
        std::vector<VectorSegment> segments;
        bool                       closed = false;
    };
}
