#pragma once

#include "CADacomb/VectorGraphics/Data/VectorBounds.h"
#include "CADacomb/VectorGraphics/Data/VectorElementType.h"
#include "CADacomb/VectorGraphics/Data/VectorStyle.h"

#include <cstddef>
#include <glm/glm.hpp>
#include <memory>
#include <string>

namespace CADacomb::VectorGraphics
{
    // Abstract base class for 2D vector scene elements (paths, groups, text).
    class VectorElement
    {
      public:
        VectorElement();
        virtual ~VectorElement();

        virtual VectorElementType              getType() const       = 0;
        virtual VectorBounds                   computeBounds() const = 0;
        virtual std::shared_ptr<VectorElement> clone() const         = 0;
        virtual size_t                         getByteSize() const   = 0;

        const std::string& getId() const;
        void               setId(const std::string& idValue);

        const glm::dmat3& getTransform() const;
        void              setTransform(const glm::dmat3& transformMatrix);

        const VectorStyle& getStyle() const;
        VectorStyle&       getStyle();
        void               setStyle(const VectorStyle& styleValue);

      private:
        std::string id;
        glm::dmat3  transform = glm::dmat3(1.0);
        VectorStyle style;
    };
}
