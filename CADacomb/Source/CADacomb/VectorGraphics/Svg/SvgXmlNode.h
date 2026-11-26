#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace CADacomb::VectorGraphics
{
    // Minimal XML element node used for zero-dependency SVG parsing.
    class SvgXmlNode
    {
      public:
        SvgXmlNode();
        virtual ~SvgXmlNode();

        const std::string& getTagName() const;
        void               setTagName(const std::string& name);

        const std::string& getTextContent() const;
        void               appendTextContent(const std::string& text);

        bool        hasAttribute(const std::string& name) const;
        std::string getAttribute(const std::string& name, const std::string& defaultValue = "") const;
        void        setAttribute(const std::string& name, const std::string& value);

        const std::map<std::string, std::string>& getAttributes() const;

        const std::vector<std::shared_ptr<SvgXmlNode>>& getChildren() const;
        void                                            addChild(std::shared_ptr<SvgXmlNode> child);

      private:
        std::string                              tagName;
        std::string                              textContent;
        std::map<std::string, std::string>       attributes;
        std::vector<std::shared_ptr<SvgXmlNode>> children;
    };
}
