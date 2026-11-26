#pragma once

#include "CADacomb/VectorGraphics/Data/VectorElement.h"

#include <memory>
#include <vector>

namespace CADacomb::VectorGraphics
{
    // Hierarchical container grouping child VectorElement instances with a shared transform and style.
    class VectorGroup : public VectorElement
    {
      public:
        VectorGroup();
        virtual ~VectorGroup();

        VectorElementType              getType() const override;
        VectorBounds                   computeBounds() const override;
        std::shared_ptr<VectorElement> clone() const override;
        size_t                         getByteSize() const override;

        const std::vector<std::shared_ptr<VectorElement>>& getChildren() const;
        void                                               addChild(std::shared_ptr<VectorElement> child);
        bool                                               isEmpty() const;

      private:
        std::vector<std::shared_ptr<VectorElement>> children;
    };
}
