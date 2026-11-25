#include "CADacomb/VectorGraphics/Data/VectorPath.h"

namespace CADacomb::VectorGraphics
{
    VectorPath::VectorPath() {}

    VectorPath::~VectorPath() {}

    VectorElementType VectorPath::getType() const
    {
        return VectorElementType::Path;
    }

    VectorBounds VectorPath::computeBounds() const
    {
        VectorBounds      bounds;
        const glm::dmat3& transform = getTransform();
        for (const VectorSubpath& subpath : subpaths)
        {
            bounds.expand(subpath.transformed(transform).computeBounds());
        }
        return bounds;
    }

    std::shared_ptr<VectorElement> VectorPath::clone() const
    {
        auto copy = std::make_shared<VectorPath>();
        copy->setId(getId());
        copy->setTransform(getTransform());
        copy->setStyle(getStyle());
        copy->subpaths = subpaths;
        return copy;
    }

    size_t VectorPath::getByteSize() const
    {
        size_t bytes = sizeof(VectorPath) + getId().size();
        for (const VectorSubpath& subpath : subpaths)
        {
            bytes += sizeof(VectorSubpath) + subpath.getSegments().size() * sizeof(VectorSegment);
        }
        return bytes;
    }

    const std::vector<VectorSubpath>& VectorPath::getSubpaths() const
    {
        return subpaths;
    }

    std::vector<VectorSubpath>& VectorPath::getSubpaths()
    {
        return subpaths;
    }

    void VectorPath::addSubpath(const VectorSubpath& subpath)
    {
        subpaths.push_back(subpath);
    }

    bool VectorPath::isEmpty() const
    {
        return subpaths.empty();
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("VectorPath/BoundsAndClone")
{
    CADacomb::VectorGraphics::VectorPath path;
    path.setId("anvil");

    CADacomb::VectorGraphics::VectorSubpath rect(glm::dvec2(0.0, 0.0));
    rect.lineTo(glm::dvec2(10.0, 0.0));
    rect.lineTo(glm::dvec2(10.0, 4.0));
    rect.lineTo(glm::dvec2(0.0, 4.0));
    rect.close();
    path.addSubpath(rect);

    glm::dmat3 transform(1.0);
    transform[2][0] = 5.0;
    transform[2][1] = 2.0;
    path.setTransform(transform);

    CADacomb::VectorGraphics::VectorBounds bounds = path.computeBounds();
    CHECK(bounds.isValid());
    CHECK(bounds.getMin().x == Catch::Approx(5.0));
    CHECK(bounds.getMin().y == Catch::Approx(2.0));
    CHECK(bounds.getMax().x == Catch::Approx(15.0));
    CHECK(bounds.getMax().y == Catch::Approx(6.0));

    std::shared_ptr<CADacomb::VectorGraphics::VectorElement> cloned = path.clone();
    CHECK(cloned->getType() == CADacomb::VectorGraphics::VectorElementType::Path);
    CHECK(cloned->getId() == "anvil");
    CHECK(cloned->computeBounds().getMax().x == Catch::Approx(15.0));
}
#endif
