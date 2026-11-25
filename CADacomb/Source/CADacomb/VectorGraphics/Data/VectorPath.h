#pragma once

#include "CADacomb/VectorGraphics/Data/VectorElement.h"
#include "CADacomb/VectorGraphics/Data/VectorSubpath.h"

#include <vector>

namespace CADacomb::VectorGraphics
{
    // Vector element holding one or more 2D subpaths (outer contours and holes).
    class VectorPath : public VectorElement
    {
      public:
        VectorPath();
        virtual ~VectorPath();

        VectorElementType              getType() const override;
        VectorBounds                   computeBounds() const override;
        std::shared_ptr<VectorElement> clone() const override;
        size_t                         getByteSize() const override;

        const std::vector<VectorSubpath>& getSubpaths() const;
        std::vector<VectorSubpath>&       getSubpaths();
        void                              addSubpath(const VectorSubpath& subpath);
        bool                              isEmpty() const;

      private:
        std::vector<VectorSubpath> subpaths;
    };
}
