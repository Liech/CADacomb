#include "CompoundSm.h"

#include "BrepTransformSm.h"
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

    void CompoundSm::transform(const glm::dmat4& transformationMatrix)
    {
        for (size_t index = 0; index < breps.size(); index++)
        {
            BrepTransformSm::transform(breps[index].get(), transformationMatrix);
        }
    }

    void CompoundSm::translate(const glm::dvec3& translationVector)
    {
        for (size_t index = 0; index < breps.size(); index++)
        {
            BrepTransformSm::translate(breps[index].get(), translationVector);
        }
    }

    void CompoundSm::rotate(const glm::dvec3& axisOrigin, const glm::dvec3& axisDirection, double angleRadians)
    {
        for (size_t index = 0; index < breps.size(); index++)
        {
            BrepTransformSm::rotate(breps[index].get(), axisOrigin, axisDirection, angleRadians);
        }
    }

    void CompoundSm::scale(const glm::dvec3& centerPoint, double scaleFactor)
    {
        for (size_t index = 0; index < breps.size(); index++)
        {
            BrepTransformSm::scale(breps[index].get(), centerPoint, scaleFactor);
        }
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
