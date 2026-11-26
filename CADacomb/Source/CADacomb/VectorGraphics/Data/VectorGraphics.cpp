#include "CADacomb/VectorGraphics/Data/VectorGraphics.h"

namespace CADacomb::VectorGraphics
{
    VectorGraphics::VectorGraphics() {}

    VectorGraphics::~VectorGraphics() {}

    bool VectorGraphics::hasViewBox() const
    {
        return viewBoxDefined;
    }

    const glm::dvec2& VectorGraphics::getViewBoxOrigin() const
    {
        return viewBoxOrigin;
    }

    const glm::dvec2& VectorGraphics::getViewBoxSize() const
    {
        return viewBoxSize;
    }

    void VectorGraphics::setViewBox(const glm::dvec2& origin, const glm::dvec2& size)
    {
        viewBoxOrigin  = origin;
        viewBoxSize    = size;
        viewBoxDefined = true;
    }

    void VectorGraphics::clearViewBox()
    {
        viewBoxOrigin  = glm::dvec2(0.0, 0.0);
        viewBoxSize    = glm::dvec2(0.0, 0.0);
        viewBoxDefined = false;
    }

    const std::vector<std::shared_ptr<VectorElement>>& VectorGraphics::getElements() const
    {
        return elements;
    }

    void VectorGraphics::addElement(std::shared_ptr<VectorElement> element)
    {
        elements.push_back(std::move(element));
    }

    void VectorGraphics::clear()
    {
        elements.clear();
        clearViewBox();
    }

    bool VectorGraphics::isEmpty() const
    {
        return elements.empty();
    }

    VectorBounds VectorGraphics::computeBounds() const
    {
        VectorBounds bounds;
        for (const std::shared_ptr<VectorElement>& element : elements)
        {
            bounds.expand(element->computeBounds());
        }
        return bounds;
    }

    size_t VectorGraphics::getByteSize() const
    {
        size_t bytes = sizeof(VectorGraphics);
        for (const std::shared_ptr<VectorElement>& element : elements)
        {
            bytes += element->getByteSize();
        }
        return bytes;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/VectorGraphics/Data/VectorPath.h"
#include "CADacomb/VectorGraphics/Data/VectorText.h"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("VectorGraphics/ElementsAndViewBox")
{
    CADacomb::VectorGraphics::VectorGraphics doc;
    CHECK(doc.isEmpty());
    CHECK_FALSE(doc.hasViewBox());

    doc.setViewBox(glm::dvec2(0.0, 0.0), glm::dvec2(100.0, 50.0));
    CHECK(doc.hasViewBox());
    CHECK(doc.getViewBoxSize() == glm::dvec2(100.0, 50.0));

    auto                                    path = std::make_shared<CADacomb::VectorGraphics::VectorPath>();
    CADacomb::VectorGraphics::VectorSubpath tri(glm::dvec2(0.0, 0.0));
    tri.lineTo(glm::dvec2(20.0, 0.0));
    tri.lineTo(glm::dvec2(10.0, 30.0));
    tri.close();
    path->addSubpath(tri);

    auto text = std::make_shared<CADacomb::VectorGraphics::VectorText>();
    text->setText("Hi");
    text->setPosition(glm::dvec2(5.0, 5.0));

    doc.addElement(path);
    doc.addElement(text);

    CHECK_FALSE(doc.isEmpty());
    CHECK(doc.getElements().size() == 2);
    CHECK(doc.getByteSize() > sizeof(CADacomb::VectorGraphics::VectorGraphics));

    CADacomb::VectorGraphics::VectorBounds bounds = doc.computeBounds();
    CHECK(bounds.isValid());
    CHECK(bounds.getMin() == glm::dvec2(0.0, 0.0));
    CHECK(bounds.getMax() == glm::dvec2(20.0, 30.0));
}
#endif
