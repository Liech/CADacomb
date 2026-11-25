#include "CADacomb/VectorGraphics/Data/VectorGroup.h"

namespace CADacomb::VectorGraphics
{
    VectorGroup::VectorGroup() {}

    VectorGroup::~VectorGroup() {}

    VectorElementType VectorGroup::getType() const
    {
        return VectorElementType::Group;
    }

    VectorBounds VectorGroup::computeBounds() const
    {
        VectorBounds childBounds;
        for (const std::shared_ptr<VectorElement>& child : children)
        {
            childBounds.expand(child->computeBounds());
        }
        return childBounds.transformed(getTransform());
    }

    std::shared_ptr<VectorElement> VectorGroup::clone() const
    {
        auto copy = std::make_shared<VectorGroup>();
        copy->setId(getId());
        copy->setTransform(getTransform());
        copy->setStyle(getStyle());
        copy->children.reserve(children.size());
        for (const std::shared_ptr<VectorElement>& child : children)
        {
            copy->children.push_back(child->clone());
        }
        return copy;
    }

    size_t VectorGroup::getByteSize() const
    {
        size_t bytes = sizeof(VectorGroup) + getId().size();
        for (const std::shared_ptr<VectorElement>& child : children)
        {
            bytes += child->getByteSize();
        }
        return bytes;
    }

    const std::vector<std::shared_ptr<VectorElement>>& VectorGroup::getChildren() const
    {
        return children;
    }

    void VectorGroup::addChild(std::shared_ptr<VectorElement> child)
    {
        children.push_back(std::move(child));
    }

    bool VectorGroup::isEmpty() const
    {
        return children.empty();
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/VectorGraphics/Data/VectorPath.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("VectorGroup/HierarchyBoundsAndClone")
{
    auto                                    path = std::make_shared<CADacomb::VectorGraphics::VectorPath>();
    CADacomb::VectorGraphics::VectorSubpath line(glm::dvec2(1.0, 2.0));
    line.lineTo(glm::dvec2(5.0, 6.0));
    path->addSubpath(line);

    CADacomb::VectorGraphics::VectorGroup group;
    group.setId("layer1");
    glm::dmat3 groupTransform(1.0);
    groupTransform[2][0] = 10.0;
    groupTransform[2][1] = 20.0;
    group.setTransform(groupTransform);
    group.addChild(path);

    CADacomb::VectorGraphics::VectorBounds bounds = group.computeBounds();
    CHECK(bounds.isValid());
    CHECK(bounds.getMin().x == Catch::Approx(11.0));
    CHECK(bounds.getMin().y == Catch::Approx(22.0));
    CHECK(bounds.getMax().x == Catch::Approx(15.0));
    CHECK(bounds.getMax().y == Catch::Approx(26.0));

    std::shared_ptr<CADacomb::VectorGraphics::VectorElement> cloned = group.clone();
    CHECK(cloned->getType() == CADacomb::VectorGraphics::VectorElementType::Group);
    auto clonedGroup = std::dynamic_pointer_cast<CADacomb::VectorGraphics::VectorGroup>(cloned);
    REQUIRE(clonedGroup != nullptr);
    CHECK(clonedGroup->getChildren().size() == 1);
    CHECK(clonedGroup->getChildren()[0] != path);
}
#endif
