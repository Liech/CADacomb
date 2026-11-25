#include "CADacomb/VectorGraphics/Data/VectorStyle.h"

namespace CADacomb::VectorGraphics
{
    VectorStyle::VectorStyle() {}

    VectorStyle::~VectorStyle() {}

    bool VectorStyle::hasFill() const
    {
        return fillEnabled;
    }

    void VectorStyle::setHasFill(bool enabled)
    {
        fillEnabled = enabled;
    }

    const glm::u8vec4& VectorStyle::getFillColor() const
    {
        return fillColor;
    }

    void VectorStyle::setFillColor(const glm::u8vec4& color)
    {
        fillColor = color;
    }

    bool VectorStyle::hasStroke() const
    {
        return strokeEnabled;
    }

    void VectorStyle::setHasStroke(bool enabled)
    {
        strokeEnabled = enabled;
    }

    const glm::u8vec4& VectorStyle::getStrokeColor() const
    {
        return strokeColor;
    }

    void VectorStyle::setStrokeColor(const glm::u8vec4& color)
    {
        strokeColor = color;
    }

    double VectorStyle::getStrokeWidth() const
    {
        return strokeWidth;
    }

    void VectorStyle::setStrokeWidth(double width)
    {
        strokeWidth = width;
    }

    VectorFillRule VectorStyle::getFillRule() const
    {
        return fillRule;
    }

    void VectorStyle::setFillRule(VectorFillRule rule)
    {
        fillRule = rule;
    }

    double VectorStyle::getOpacity() const
    {
        return opacity;
    }

    void VectorStyle::setOpacity(double value)
    {
        opacity = value;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("VectorStyle/DefaultsAndAccessors")
{
    CADacomb::VectorGraphics::VectorStyle style;
    CHECK(style.hasFill());
    CHECK_FALSE(style.hasStroke());
    CHECK(style.getFillColor() == glm::u8vec4(0, 0, 0, 255));
    CHECK(style.getFillRule() == CADacomb::VectorGraphics::VectorFillRule::NonZero);
    CHECK(style.getOpacity() == 1.0);

    style.setHasStroke(true);
    style.setStrokeColor(glm::u8vec4(255, 0, 0, 255));
    style.setStrokeWidth(2.5);
    style.setFillRule(CADacomb::VectorGraphics::VectorFillRule::EvenOdd);
    style.setOpacity(0.5);

    CHECK(style.hasStroke());
    CHECK(style.getStrokeColor() == glm::u8vec4(255, 0, 0, 255));
    CHECK(style.getStrokeWidth() == 2.5);
    CHECK(style.getFillRule() == CADacomb::VectorGraphics::VectorFillRule::EvenOdd);
    CHECK(style.getOpacity() == 0.5);
}
#endif
