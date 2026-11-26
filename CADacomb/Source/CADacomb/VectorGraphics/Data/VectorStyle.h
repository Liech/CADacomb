#pragma once

#include "CADacomb/VectorGraphics/Data/VectorFillRule.h"

#include <glm/glm.hpp>

namespace CADacomb::VectorGraphics
{
    // Visual style properties (fill, stroke, fill rule, opacity) for a vector element.
    class VectorStyle
    {
      public:
        VectorStyle();
        virtual ~VectorStyle();

        bool hasFill() const;
        void setHasFill(bool enabled);

        const glm::u8vec4& getFillColor() const;
        void               setFillColor(const glm::u8vec4& color);

        bool hasStroke() const;
        void setHasStroke(bool enabled);

        const glm::u8vec4& getStrokeColor() const;
        void               setStrokeColor(const glm::u8vec4& color);

        double getStrokeWidth() const;
        void   setStrokeWidth(double width);

        VectorFillRule getFillRule() const;
        void           setFillRule(VectorFillRule rule);

        double getOpacity() const;
        void   setOpacity(double value);

      private:
        glm::u8vec4    fillColor     = glm::u8vec4(0, 0, 0, 255);
        glm::u8vec4    strokeColor   = glm::u8vec4(0, 0, 0, 255);
        double         strokeWidth   = 1.0;
        double         opacity       = 1.0;
        VectorFillRule fillRule      = VectorFillRule::NonZero;
        bool           fillEnabled   = true;
        bool           strokeEnabled = false;
    };
}
