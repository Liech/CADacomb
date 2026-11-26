#pragma once

#include "CADacomb/VectorGraphics/Data/VectorBounds.h"
#include "CADacomb/VectorGraphics/Data/VectorElement.h"

#include <cstddef>
#include <glm/glm.hpp>
#include <memory>
#include <vector>

namespace CADacomb::VectorGraphics
{
    // Headless 2D vector graphic document in Y-down coordinates containing ordered vector elements.
    class VectorGraphics
    {
      public:
        VectorGraphics();
        virtual ~VectorGraphics();

        bool              hasViewBox() const;
        const glm::dvec2& getViewBoxOrigin() const;
        const glm::dvec2& getViewBoxSize() const;
        void              setViewBox(const glm::dvec2& origin, const glm::dvec2& size);
        void              clearViewBox();

        const std::vector<std::shared_ptr<VectorElement>>& getElements() const;
        void                                               addElement(std::shared_ptr<VectorElement> element);
        void                                               clear();
        bool                                               isEmpty() const;

        VectorBounds computeBounds() const;
        size_t       getByteSize() const;

      private:
        std::vector<std::shared_ptr<VectorElement>> elements;
        glm::dvec2                                  viewBoxOrigin  = glm::dvec2(0.0, 0.0);
        glm::dvec2                                  viewBoxSize    = glm::dvec2(0.0, 0.0);
        bool                                        viewBoxDefined = false;
    };
}
