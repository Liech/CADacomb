#pragma once

#include "CADacomb/VectorGraphics/Data/VectorElement.h"

#include <glm/glm.hpp>
#include <string>

namespace CADacomb::VectorGraphics
{
    // Vector element storing positioned 2D text metadata for preservation and future glyph outline conversion.
    class VectorText : public VectorElement
    {
      public:
        VectorText();
        virtual ~VectorText();

        VectorElementType              getType() const override;
        VectorBounds                   computeBounds() const override;
        std::shared_ptr<VectorElement> clone() const override;
        size_t                         getByteSize() const override;

        const std::string& getText() const;
        void               setText(const std::string& value);

        const glm::dvec2& getPosition() const;
        void              setPosition(const glm::dvec2& pos);

        const std::string& getFontFamily() const;
        void               setFontFamily(const std::string& family);

        double getFontSize() const;
        void   setFontSize(double size);

      private:
        std::string text;
        std::string fontFamily;
        glm::dvec2  position = glm::dvec2(0.0, 0.0);
        double      fontSize = 16.0;
    };
}
