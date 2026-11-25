#include "CADacomb/VectorGraphics/Data/VectorElement.h"

namespace CADacomb::VectorGraphics
{
    VectorElement::VectorElement() {}

    VectorElement::~VectorElement() {}

    const std::string& VectorElement::getId() const
    {
        return id;
    }

    void VectorElement::setId(const std::string& idValue)
    {
        id = idValue;
    }

    const glm::dmat3& VectorElement::getTransform() const
    {
        return transform;
    }

    void VectorElement::setTransform(const glm::dmat3& transformMatrix)
    {
        transform = transformMatrix;
    }

    const VectorStyle& VectorElement::getStyle() const
    {
        return style;
    }

    VectorStyle& VectorElement::getStyle()
    {
        return style;
    }

    void VectorElement::setStyle(const VectorStyle& styleValue)
    {
        style = styleValue;
    }
}
