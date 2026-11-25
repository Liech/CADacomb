#pragma once

#include <glm/glm.hpp>

namespace CADacomb::VectorGraphics
{
    // Axis-aligned 2D bounding box in vector document space.
    class VectorBounds
    {
      public:
        VectorBounds();
        VectorBounds(const glm::dvec2& minPoint, const glm::dvec2& maxPoint);
        virtual ~VectorBounds();

        bool isValid() const;

        const glm::dvec2& getMin() const;
        const glm::dvec2& getMax() const;
        glm::dvec2        getSize() const;
        glm::dvec2        getCenter() const;

        void         expand(const glm::dvec2& point);
        void         expand(const VectorBounds& other);
        VectorBounds transformed(const glm::dmat3& matrix) const;

      private:
        glm::dvec2 minPoint = glm::dvec2(0.0, 0.0);
        glm::dvec2 maxPoint = glm::dvec2(0.0, 0.0);
        bool       valid    = false;
    };
}
