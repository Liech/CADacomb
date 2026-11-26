#include "CADacomb/VectorGraphics/Svg/SvgXmlNode.h"

namespace CADacomb::VectorGraphics
{
    SvgXmlNode::SvgXmlNode() {}

    SvgXmlNode::~SvgXmlNode() {}

    const std::string& SvgXmlNode::getTagName() const
    {
        return tagName;
    }

    void SvgXmlNode::setTagName(const std::string& name)
    {
        tagName = name;
    }

    const std::string& SvgXmlNode::getTextContent() const
    {
        return textContent;
    }

    void SvgXmlNode::appendTextContent(const std::string& text)
    {
        textContent += text;
    }

    bool SvgXmlNode::hasAttribute(const std::string& name) const
    {
        return attributes.find(name) != attributes.end();
    }

    std::string SvgXmlNode::getAttribute(const std::string& name, const std::string& defaultValue) const
    {
        auto it = attributes.find(name);
        if (it == attributes.end())
        {
            return defaultValue;
        }
        return it->second;
    }

    void SvgXmlNode::setAttribute(const std::string& name, const std::string& value)
    {
        attributes[name] = value;
    }

    const std::map<std::string, std::string>& SvgXmlNode::getAttributes() const
    {
        return attributes;
    }

    const std::vector<std::shared_ptr<SvgXmlNode>>& SvgXmlNode::getChildren() const
    {
        return children;
    }

    void SvgXmlNode::addChild(std::shared_ptr<SvgXmlNode> child)
    {
        children.push_back(std::move(child));
    }
}
