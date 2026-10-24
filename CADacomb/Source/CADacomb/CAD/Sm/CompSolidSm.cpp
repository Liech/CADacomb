#include "CompSolidSm.h"

#include "SolidSm.h"
#include <algorithm>

namespace CADacomb::CAD
{
    CompSolidSm::CompSolidSm(const std::vector<std::shared_ptr<SmBrep>>& brepsInput)
      : breps(brepsInput)
    {
    }

    CompSolidSm::~CompSolidSm() {}

    ShapeType CompSolidSm::getShapeType() const
    {
        return ShapeType::CompSolid;
    }

    AABB3 CompSolidSm::getAABB() const
    {
        std::vector<std::unique_ptr<Solid>> solids = getSolids();
        if (solids.empty())
        {
            return AABB3();
        }
        AABB3 combinedBounds = solids[0]->getAABB();
        for (size_t index = 1; index < solids.size(); index++)
        {
            AABB3      solidBounds = solids[index]->getAABB();
            glm::dvec3 minCorner   = glm::min(combinedBounds.getMin(), solidBounds.getMin());
            glm::dvec3 maxCorner   = glm::max(combinedBounds.getMax(), solidBounds.getMax());
            combinedBounds         = AABB3(minCorner, maxCorner);
        }
        return combinedBounds;
    }

    std::vector<std::unique_ptr<Solid>> CompSolidSm::getSolids() const
    {
        std::vector<std::unique_ptr<Solid>> solids;
        solids.reserve(breps.size());
        for (size_t index = 0; index < breps.size(); index++)
        {
            solids.push_back(std::make_unique<SolidSm>(breps[index]));
        }
        return solids;
    }

    std::vector<std::unique_ptr<Shape>> CompSolidSm::getChildren() const
    {
        std::vector<std::unique_ptr<Solid>> solids = getSolids();
        std::vector<std::unique_ptr<Shape>> children;
        children.reserve(solids.size());
        for (size_t index = 0; index < solids.size(); index++)
        {
            children.push_back(std::move(solids[index]));
        }
        return children;
    }

    const std::vector<std::shared_ptr<SmBrep>>& CompSolidSm::getBreps() const
    {
        return breps;
    }
}
