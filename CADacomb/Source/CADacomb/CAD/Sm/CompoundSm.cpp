#include "CompoundSm.h"

#include "SolidSm.h"
#include <algorithm>

namespace CADacomb::CAD
{
    CompoundSm::CompoundSm(const std::vector<std::shared_ptr<SmBrep>>& brepsInput)
      : breps(brepsInput)
    {
    }

    CompoundSm::~CompoundSm() {}

    ShapeType CompoundSm::getShapeType() const
    {
        return ShapeType::Compound;
    }

    AABB3 CompoundSm::getAABB() const
    {
        std::vector<std::unique_ptr<Shape>> shapes = getShapes();
        if (shapes.empty())
        {
            return AABB3();
        }
        AABB3 combinedBounds = shapes[0]->getAABB();
        for (size_t index = 1; index < shapes.size(); index++)
        {
            AABB3      shapeBounds = shapes[index]->getAABB();
            glm::dvec3 minCorner   = glm::min(combinedBounds.getMin(), shapeBounds.getMin());
            glm::dvec3 maxCorner   = glm::max(combinedBounds.getMax(), shapeBounds.getMax());
            combinedBounds         = AABB3(minCorner, maxCorner);
        }
        return combinedBounds;
    }

    std::vector<std::unique_ptr<Shape>> CompoundSm::getShapes() const
    {
        std::vector<std::unique_ptr<Shape>> shapes;
        shapes.reserve(breps.size());
        for (size_t index = 0; index < breps.size(); index++)
        {
            shapes.push_back(std::make_unique<SolidSm>(breps[index]));
        }
        return shapes;
    }

    std::vector<std::unique_ptr<Shape>> CompoundSm::getChildren() const
    {
        return getShapes();
    }

    const std::vector<std::shared_ptr<SmBrep>>& CompoundSm::getBreps() const
    {
        return breps;
    }
}
