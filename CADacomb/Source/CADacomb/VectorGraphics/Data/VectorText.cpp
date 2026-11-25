#include "CADacomb/VectorGraphics/Data/VectorText.h"

namespace CADacomb::VectorGraphics
{
    VectorText::VectorText() {}

    VectorText::~VectorText() {}

    VectorElementType VectorText::getType() const
    {
        return VectorElementType::Text;
    }

    VectorBounds VectorText::computeBounds() const
    {
        glm::dvec3   p = getTransform() * glm::dvec3(position, 1.0);
        VectorBounds bounds;
        bounds.expand(glm::dvec2(p.x, p.y));
        return bounds;
    }

    std::shared_ptr<VectorElement> VectorText::clone() const
    {
        auto copy = std::make_shared<VectorText>();
        copy->setId(getId());
        copy->setTransform(getTransform());
        copy->setStyle(getStyle());
        copy->text       = text;
        copy->fontFamily = fontFamily;
        copy->position   = position;
        copy->fontSize   = fontSize;
        return copy;
    }

    size_t VectorText::getByteSize() const
    {
        return sizeof(VectorText) + getId().size() + text.size() + fontFamily.size();
    }

    const std::string& VectorText::getText() const
    {
        return text;
    }

    void VectorText::setText(const std::string& value)
    {
        text = value;
    }

    const glm::dvec2& VectorText::getPosition() const
    {
        return position;
    }

    void VectorText::setPosition(const glm::dvec2& pos)
    {
        position = pos;
    }

    const std::string& VectorText::getFontFamily() const
    {
        return fontFamily;
    }

    void VectorText::setFontFamily(const std::string& family)
    {
        fontFamily = family;
    }

    double VectorText::getFontSize() const
    {
        return fontSize;
    }

    void VectorText::setFontSize(double size)
    {
        fontSize = size;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("VectorText/AccessorsAndClone")
{
    CADacomb::VectorGraphics::VectorText textElement;
    textElement.setId("label");
    textElement.setText("Forged");
    textElement.setPosition(glm::dvec2(12.0, 24.0));
    textElement.setFontFamily("serif");
    textElement.setFontSize(18.0);

    CHECK(textElement.getType() == CADacomb::VectorGraphics::VectorElementType::Text);
    CHECK(textElement.getText() == "Forged");
    CHECK(textElement.getPosition() == glm::dvec2(12.0, 24.0));
    CHECK(textElement.getFontFamily() == "serif");
    CHECK(textElement.getFontSize() == 18.0);

    std::shared_ptr<CADacomb::VectorGraphics::VectorElement> cloned     = textElement.clone();
    auto                                                     clonedText = std::dynamic_pointer_cast<CADacomb::VectorGraphics::VectorText>(cloned);
    REQUIRE(clonedText != nullptr);
    CHECK(clonedText->getText() == "Forged");
    CHECK(clonedText->getFontFamily() == "serif");
    CHECK(clonedText->getFontSize() == 18.0);
}
#endif
