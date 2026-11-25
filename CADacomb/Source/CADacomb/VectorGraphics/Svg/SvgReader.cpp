#include "CADacomb/VectorGraphics/Svg/SvgReader.h"

#include "CADacomb/VectorGraphics/Data/VectorGroup.h"
#include "CADacomb/VectorGraphics/Data/VectorText.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace CADacomb::VectorGraphics
{
    SvgReader::SvgReader() {}

    SvgReader::~SvgReader() {}

    VectorGraphics SvgReader::loadFromFile(const std::string& filePath)
    {
        std::ifstream file(filePath, std::ios::binary);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open SVG file: " + filePath);
        }

        std::ostringstream buffer;
        buffer << file.rdbuf();
        return loadFromString(buffer.str());
    }

    VectorGraphics SvgReader::loadFromString(const std::string& svgContent)
    {
        VectorGraphics              document;
        std::shared_ptr<SvgXmlNode> root = parseXmlRoot(svgContent);
        if (!root)
        {
            return document;
        }

        if (root->hasAttribute("viewBox"))
        {
            std::vector<double> vb = parseNumberList(root->getAttribute("viewBox"));
            if (vb.size() >= 4 && vb[2] > 0.0 && vb[3] > 0.0)
            {
                document.setViewBox(glm::dvec2(vb[0], vb[1]), glm::dvec2(vb[2], vb[3]));
            }
        }
        else if (root->hasAttribute("width") && root->hasAttribute("height"))
        {
            double w = parseLength(root->getAttribute("width"), 0.0);
            double h = parseLength(root->getAttribute("height"), 0.0);
            if (w > 0.0 && h > 0.0)
            {
                document.setViewBox(glm::dvec2(0.0, 0.0), glm::dvec2(w, h));
            }
        }

        VectorStyle rootStyle = parseStyle(*root, VectorStyle());
        for (const std::shared_ptr<SvgXmlNode>& child : root->getChildren())
        {
            std::shared_ptr<VectorElement> element = convertNode(*child, rootStyle);
            if (element)
            {
                document.addElement(element);
            }
        }

        return document;
    }

    std::shared_ptr<SvgXmlNode> SvgReader::parseXmlRoot(const std::string& content)
    {
        size_t pos = 0;
        if (content.size() >= 3 && static_cast<unsigned char>(content[0]) == 0xEF && static_cast<unsigned char>(content[1]) == 0xBB && static_cast<unsigned char>(content[2]) == 0xBF)
        {
            pos = 3;
        }

        while (pos < content.size())
        {
            skipWhitespaceAndComments(content, pos);
            if (pos >= content.size())
            {
                break;
            }
            if (content[pos] == '<')
            {
                if (pos + 1 < content.size() && (content[pos + 1] == '?' || content[pos + 1] == '!'))
                {
                    skipWhitespaceAndComments(content, pos);
                    if (pos < content.size() && content[pos] == '<')
                    {
                        size_t closePos = content.find('>', pos);
                        if (closePos == std::string::npos)
                        {
                            break;
                        }
                        pos = closePos + 1;
                    }
                    continue;
                }
                return parseXmlElement(content, pos);
            }
            pos++;
        }

        return nullptr;
    }

    std::shared_ptr<SvgXmlNode> SvgReader::parseXmlElement(const std::string& content, size_t& pos)
    {
        if (pos >= content.size() || content[pos] != '<')
        {
            return nullptr;
        }
        pos++;

        if (pos >= content.size() || content[pos] == '/' || content[pos] == '?' || content[pos] == '!')
        {
            return nullptr;
        }

        size_t nameStart = pos;
        while (pos < content.size() && !std::isspace(static_cast<unsigned char>(content[pos])) && content[pos] != '>' && content[pos] != '/')
        {
            pos++;
        }

        auto node = std::make_shared<SvgXmlNode>();
        node->setTagName(stripNamespacePrefix(content.substr(nameStart, pos - nameStart)));

        bool selfClosing = false;
        while (pos < content.size())
        {
            while (pos < content.size() && std::isspace(static_cast<unsigned char>(content[pos])))
            {
                pos++;
            }
            if (pos >= content.size())
            {
                break;
            }
            if (content[pos] == '>')
            {
                pos++;
                break;
            }
            if (content[pos] == '/' && pos + 1 < content.size() && content[pos + 1] == '>')
            {
                selfClosing = true;
                pos += 2;
                break;
            }

            size_t attrNameStart = pos;
            while (pos < content.size() && !std::isspace(static_cast<unsigned char>(content[pos])) && content[pos] != '=' && content[pos] != '>' && content[pos] != '/')
            {
                pos++;
            }
            std::string attrName = content.substr(attrNameStart, pos - attrNameStart);

            while (pos < content.size() && std::isspace(static_cast<unsigned char>(content[pos])))
            {
                pos++;
            }

            std::string attrValue;
            if (pos < content.size() && content[pos] == '=')
            {
                pos++;
                while (pos < content.size() && std::isspace(static_cast<unsigned char>(content[pos])))
                {
                    pos++;
                }
                if (pos < content.size() && (content[pos] == '"' || content[pos] == '\''))
                {
                    char   quote    = content[pos++];
                    size_t valStart = pos;
                    while (pos < content.size() && content[pos] != quote)
                    {
                        pos++;
                    }
                    attrValue = unescapeXml(content.substr(valStart, pos - valStart));
                    if (pos < content.size() && content[pos] == quote)
                    {
                        pos++;
                    }
                }
            }

            if (!attrName.empty())
            {
                node->setAttribute(attrName, attrValue);
            }
        }

        if (selfClosing)
        {
            return node;
        }

        while (pos < content.size())
        {
            if (content[pos] == '<')
            {
                if (pos + 1 < content.size() && content[pos + 1] == '/')
                {
                    size_t closePos = content.find('>', pos);
                    if (closePos == std::string::npos)
                    {
                        pos = content.size();
                    }
                    else
                    {
                        pos = closePos + 1;
                    }
                    break;
                }
                if (content.compare(pos, 4, "<!--") == 0)
                {
                    size_t endComment = content.find("-->", pos + 4);
                    pos               = (endComment == std::string::npos) ? content.size() : endComment + 3;
                    continue;
                }
                if (content.compare(pos, 9, "<![CDATA[") == 0)
                {
                    size_t endCdata = content.find("]]>", pos + 9);
                    if (endCdata == std::string::npos)
                    {
                        node->appendTextContent(content.substr(pos + 9));
                        pos = content.size();
                    }
                    else
                    {
                        node->appendTextContent(content.substr(pos + 9, endCdata - (pos + 9)));
                        pos = endCdata + 3;
                    }
                    continue;
                }
                if (pos + 1 < content.size() && (content[pos + 1] == '?' || content[pos + 1] == '!'))
                {
                    size_t closePos = content.find('>', pos);
                    pos             = (closePos == std::string::npos) ? content.size() : closePos + 1;
                    continue;
                }

                std::shared_ptr<SvgXmlNode> child = parseXmlElement(content, pos);
                if (child)
                {
                    node->addChild(child);
                }
                else
                {
                    pos++;
                }
            }
            else
            {
                size_t textStart = pos;
                while (pos < content.size() && content[pos] != '<')
                {
                    pos++;
                }
                node->appendTextContent(unescapeXml(content.substr(textStart, pos - textStart)));
            }
        }

        return node;
    }

    void SvgReader::skipWhitespaceAndComments(const std::string& content, size_t& pos)
    {
        while (pos < content.size())
        {
            while (pos < content.size() && std::isspace(static_cast<unsigned char>(content[pos])))
            {
                pos++;
            }
            if (content.compare(pos, 4, "<!--") == 0)
            {
                size_t endComment = content.find("-->", pos + 4);
                pos               = (endComment == std::string::npos) ? content.size() : endComment + 3;
                continue;
            }
            if (content.compare(pos, 2, "<?") == 0)
            {
                size_t endPi = content.find("?>", pos + 2);
                pos          = (endPi == std::string::npos) ? content.size() : endPi + 2;
                continue;
            }
            if (content.compare(pos, 2, "<!") == 0)
            {
                size_t endDecl = content.find('>', pos + 2);
                pos            = (endDecl == std::string::npos) ? content.size() : endDecl + 1;
                continue;
            }
            break;
        }
    }

    std::string SvgReader::unescapeXml(const std::string& text)
    {
        std::string result;
        result.reserve(text.size());
        for (size_t i = 0; i < text.size(); i++)
        {
            if (text[i] == '&')
            {
                if (text.compare(i, 5, "&amp;") == 0)
                {
                    result += '&';
                    i += 4;
                }
                else if (text.compare(i, 4, "&lt;") == 0)
                {
                    result += '<';
                    i += 3;
                }
                else if (text.compare(i, 4, "&gt;") == 0)
                {
                    result += '>';
                    i += 3;
                }
                else if (text.compare(i, 6, "&quot;") == 0)
                {
                    result += '"';
                    i += 5;
                }
                else if (text.compare(i, 6, "&apos;") == 0)
                {
                    result += '\'';
                    i += 5;
                }
                else
                {
                    result += text[i];
                }
            }
            else
            {
                result += text[i];
            }
        }
        return result;
    }

    std::string SvgReader::stripNamespacePrefix(const std::string& tagName)
    {
        size_t colonPos = tagName.find(':');
        if (colonPos != std::string::npos)
        {
            return tagName.substr(colonPos + 1);
        }
        return tagName;
    }

    std::shared_ptr<VectorElement> SvgReader::convertNode(const SvgXmlNode& node, const VectorStyle& inheritedStyle)
    {
        std::string tag = toLower(node.getTagName());
        if (tag == "defs" || tag == "metadata" || tag == "title" || tag == "desc" || tag == "style" || tag == "filter" || tag == "lineargradient" || tag == "radialgradient" || tag == "clippath" ||
            tag == "mask" || tag == "namedview")
        {
            return nullptr;
        }

        VectorStyle style = parseStyle(node, inheritedStyle);
        glm::dmat3  transform(1.0);
        if (node.hasAttribute("transform"))
        {
            transform = parseTransform(node.getAttribute("transform"));
        }

        std::shared_ptr<VectorElement> element;
        if (tag == "g" || tag == "svg")
        {
            auto group = std::make_shared<VectorGroup>();
            group->setId(node.getAttribute("id"));
            group->setTransform(transform);
            group->setStyle(style);
            for (const std::shared_ptr<SvgXmlNode>& child : node.getChildren())
            {
                std::shared_ptr<VectorElement> childElem = convertNode(*child, style);
                if (childElem)
                {
                    group->addChild(childElem);
                }
            }
            if (!group->isEmpty())
            {
                element = group;
            }
        }
        else if (tag == "path")
        {
            element = parsePathElement(node, style);
        }
        else if (tag == "rect")
        {
            element = parseRectElement(node, style);
        }
        else if (tag == "circle")
        {
            element = parseCircleElement(node, style);
        }
        else if (tag == "ellipse")
        {
            element = parseEllipseElement(node, style);
        }
        else if (tag == "line")
        {
            element = parseLineElement(node, style);
        }
        else if (tag == "polyline")
        {
            element = parsePolyElement(node, style, false);
        }
        else if (tag == "polygon")
        {
            element = parsePolyElement(node, style, true);
        }
        else if (tag == "text")
        {
            element = parseTextElement(node, style);
        }

        if (element)
        {
            element->setId(node.getAttribute("id"));
            element->setTransform(transform);
            element->setStyle(style);
        }

        return element;
    }

    std::shared_ptr<VectorPath> SvgReader::parsePathElement(const SvgXmlNode& node, const VectorStyle& style)
    {
        auto path = std::make_shared<VectorPath>();
        path->setStyle(style);
        if (node.hasAttribute("d"))
        {
            parsePathData(node.getAttribute("d"), *path);
        }
        if (path->isEmpty())
        {
            return nullptr;
        }
        return path;
    }

    std::shared_ptr<VectorPath> SvgReader::parseRectElement(const SvgXmlNode& node, const VectorStyle& style)
    {
        double x  = parseLength(node.getAttribute("x"), 0.0);
        double y  = parseLength(node.getAttribute("y"), 0.0);
        double w  = parseLength(node.getAttribute("width"), 0.0);
        double h  = parseLength(node.getAttribute("height"), 0.0);
        double rx = parseLength(node.getAttribute("rx"), 0.0);
        double ry = parseLength(node.getAttribute("ry"), rx);
        if (!node.hasAttribute("rx") && node.hasAttribute("ry"))
        {
            rx = ry;
        }
        if (w <= 0.0 || h <= 0.0)
        {
            return nullptr;
        }

        rx = std::clamp(rx, 0.0, w * 0.5);
        ry = std::clamp(ry, 0.0, h * 0.5);

        auto path = std::make_shared<VectorPath>();
        path->setStyle(style);

        if (rx <= 1e-6 || ry <= 1e-6)
        {
            VectorSubpath subpath(glm::dvec2(x, y));
            subpath.lineTo(glm::dvec2(x + w, y));
            subpath.lineTo(glm::dvec2(x + w, y + h));
            subpath.lineTo(glm::dvec2(x, y + h));
            subpath.close();
            path->addSubpath(subpath);
        }
        else
        {
            constexpr double kappa = 0.5522847498307936;
            double           kx    = rx * kappa;
            double           ky    = ry * kappa;

            VectorSubpath subpath(glm::dvec2(x + rx, y));
            subpath.lineTo(glm::dvec2(x + w - rx, y));
            subpath.cubicTo(glm::dvec2(x + w - rx + kx, y), glm::dvec2(x + w, y + ry - ky), glm::dvec2(x + w, y + ry));
            subpath.lineTo(glm::dvec2(x + w, y + h - ry));
            subpath.cubicTo(glm::dvec2(x + w, y + h - ry + ky), glm::dvec2(x + w - rx + kx, y + h), glm::dvec2(x + w - rx, y + h));
            subpath.lineTo(glm::dvec2(x + rx, y + h));
            subpath.cubicTo(glm::dvec2(x + rx - kx, y + h), glm::dvec2(x, y + h - ry + ky), glm::dvec2(x, y + h - ry));
            subpath.lineTo(glm::dvec2(x, y + ry));
            subpath.cubicTo(glm::dvec2(x, y + ry - ky), glm::dvec2(x + rx - kx, y), glm::dvec2(x + rx, y));
            subpath.close();
            path->addSubpath(subpath);
        }

        return path;
    }

    std::shared_ptr<VectorPath> SvgReader::parseCircleElement(const SvgXmlNode& node, const VectorStyle& style)
    {
        double cx = parseLength(node.getAttribute("cx"), 0.0);
        double cy = parseLength(node.getAttribute("cy"), 0.0);
        double r  = parseLength(node.getAttribute("r"), 0.0);
        if (r <= 0.0)
        {
            return nullptr;
        }

        auto path = std::make_shared<VectorPath>();
        path->setStyle(style);
        addEllipseSubpath(*path, cx, cy, r, r);
        return path;
    }

    std::shared_ptr<VectorPath> SvgReader::parseEllipseElement(const SvgXmlNode& node, const VectorStyle& style)
    {
        double cx = parseLength(node.getAttribute("cx"), 0.0);
        double cy = parseLength(node.getAttribute("cy"), 0.0);
        double rx = parseLength(node.getAttribute("rx"), 0.0);
        double ry = parseLength(node.getAttribute("ry"), 0.0);
        if (rx <= 0.0 || ry <= 0.0)
        {
            return nullptr;
        }

        auto path = std::make_shared<VectorPath>();
        path->setStyle(style);
        addEllipseSubpath(*path, cx, cy, rx, ry);
        return path;
    }

    std::shared_ptr<VectorPath> SvgReader::parseLineElement(const SvgXmlNode& node, const VectorStyle& style)
    {
        double x1 = parseLength(node.getAttribute("x1"), 0.0);
        double y1 = parseLength(node.getAttribute("y1"), 0.0);
        double x2 = parseLength(node.getAttribute("x2"), 0.0);
        double y2 = parseLength(node.getAttribute("y2"), 0.0);

        auto          path = std::make_shared<VectorPath>();
        VectorSubpath subpath(glm::dvec2(x1, y1));
        subpath.lineTo(glm::dvec2(x2, y2));
        path->setStyle(style);
        path->addSubpath(subpath);
        return path;
    }

    std::shared_ptr<VectorPath> SvgReader::parsePolyElement(const SvgXmlNode& node, const VectorStyle& style, bool closed)
    {
        std::vector<double> coords = parseNumberList(node.getAttribute("points"));
        if (coords.size() < 4)
        {
            return nullptr;
        }

        VectorSubpath subpath(glm::dvec2(coords[0], coords[1]));
        for (size_t i = 2; i + 1 < coords.size(); i += 2)
        {
            subpath.lineTo(glm::dvec2(coords[i], coords[i + 1]));
        }
        if (closed)
        {
            subpath.close();
        }

        auto path = std::make_shared<VectorPath>();
        path->setStyle(style);
        path->addSubpath(subpath);
        return path;
    }

    std::shared_ptr<VectorElement> SvgReader::parseTextElement(const SvgXmlNode& node, const VectorStyle& style)
    {
        auto textElem = std::make_shared<VectorText>();
        textElem->setStyle(style);
        textElem->setPosition(glm::dvec2(parseLength(node.getAttribute("x"), 0.0), parseLength(node.getAttribute("y"), 0.0)));
        textElem->setText(trim(collectTextContent(node)));

        std::map<std::string, std::string> inlineStyle = parseInlineStyle(node.getAttribute("style"));
        std::string                        fontFamily  = node.getAttribute("font-family");
        if (fontFamily.empty() && inlineStyle.find("font-family") != inlineStyle.end())
        {
            fontFamily = inlineStyle["font-family"];
        }
        textElem->setFontFamily(fontFamily);

        std::string fontSizeStr = node.getAttribute("font-size");
        if (fontSizeStr.empty() && inlineStyle.find("font-size") != inlineStyle.end())
        {
            fontSizeStr = inlineStyle["font-size"];
        }
        textElem->setFontSize(parseLength(fontSizeStr, 16.0));
        return textElem;
    }

    std::string SvgReader::collectTextContent(const SvgXmlNode& node)
    {
        std::string text = node.getTextContent();
        for (const std::shared_ptr<SvgXmlNode>& child : node.getChildren())
        {
            text += collectTextContent(*child);
        }
        return text;
    }

    void SvgReader::parsePathData(const std::string& pathData, VectorPath& outPath)
    {
        size_t        pos     = 0;
        char          command = 0;
        glm::dvec2    current(0.0, 0.0);
        glm::dvec2    subpathStart(0.0, 0.0);
        glm::dvec2    lastControl(0.0, 0.0);
        char          lastCommand = 0;
        bool          hasSubpath  = false;
        VectorSubpath activeSubpath;

        while (pos < pathData.size())
        {
            skipSeparators(pathData, pos);
            if (pos >= pathData.size())
            {
                break;
            }

            if (std::isalpha(static_cast<unsigned char>(pathData[pos])))
            {
                command = pathData[pos++];
            }
            else if (command == 0)
            {
                pos++;
                continue;
            }

            bool relative = std::islower(static_cast<unsigned char>(command)) != 0;
            char upper    = static_cast<char>(std::toupper(static_cast<unsigned char>(command)));

            if (upper == 'Z')
            {
                if (hasSubpath)
                {
                    activeSubpath.close();
                    outPath.addSubpath(activeSubpath);
                    hasSubpath = false;
                }
                current     = subpathStart;
                lastCommand = upper;
                command     = 0;
                continue;
            }

            if (upper == 'M')
            {
                double x = 0.0;
                double y = 0.0;
                if (!readNextNumber(pathData, pos, x) || !readNextNumber(pathData, pos, y))
                {
                    break;
                }
                if (hasSubpath && !activeSubpath.isEmpty())
                {
                    outPath.addSubpath(activeSubpath);
                }

                glm::dvec2 target = relative ? (current + glm::dvec2(x, y)) : glm::dvec2(x, y);
                activeSubpath     = VectorSubpath(target);
                hasSubpath        = true;
                current           = target;
                subpathStart      = target;
                lastCommand       = upper;
                command           = relative ? 'l' : 'L';
            }
            else if (upper == 'L')
            {
                double x = 0.0;
                double y = 0.0;
                if (!readNextNumber(pathData, pos, x) || !readNextNumber(pathData, pos, y))
                {
                    break;
                }
                if (!hasSubpath)
                {
                    activeSubpath = VectorSubpath(current);
                    hasSubpath    = true;
                    subpathStart  = current;
                }
                glm::dvec2 target = relative ? (current + glm::dvec2(x, y)) : glm::dvec2(x, y);
                activeSubpath.lineTo(target);
                current     = target;
                lastCommand = upper;
            }
            else if (upper == 'H')
            {
                double x = 0.0;
                if (!readNextNumber(pathData, pos, x))
                {
                    break;
                }
                if (!hasSubpath)
                {
                    activeSubpath = VectorSubpath(current);
                    hasSubpath    = true;
                    subpathStart  = current;
                }
                glm::dvec2 target(relative ? (current.x + x) : x, current.y);
                activeSubpath.lineTo(target);
                current     = target;
                lastCommand = upper;
            }
            else if (upper == 'V')
            {
                double y = 0.0;
                if (!readNextNumber(pathData, pos, y))
                {
                    break;
                }
                if (!hasSubpath)
                {
                    activeSubpath = VectorSubpath(current);
                    hasSubpath    = true;
                    subpathStart  = current;
                }
                glm::dvec2 target(current.x, relative ? (current.y + y) : y);
                activeSubpath.lineTo(target);
                current     = target;
                lastCommand = upper;
            }
            else if (upper == 'C')
            {
                double x1 = 0.0;
                double y1 = 0.0;
                double x2 = 0.0;
                double y2 = 0.0;
                double x  = 0.0;
                double y  = 0.0;
                if (!readNextNumber(pathData, pos, x1) || !readNextNumber(pathData, pos, y1) || !readNextNumber(pathData, pos, x2) || !readNextNumber(pathData, pos, y2) ||
                    !readNextNumber(pathData, pos, x) || !readNextNumber(pathData, pos, y))
                {
                    break;
                }
                if (!hasSubpath)
                {
                    activeSubpath = VectorSubpath(current);
                    hasSubpath    = true;
                    subpathStart  = current;
                }
                glm::dvec2 cp1 = relative ? (current + glm::dvec2(x1, y1)) : glm::dvec2(x1, y1);
                glm::dvec2 cp2 = relative ? (current + glm::dvec2(x2, y2)) : glm::dvec2(x2, y2);
                glm::dvec2 end = relative ? (current + glm::dvec2(x, y)) : glm::dvec2(x, y);
                activeSubpath.cubicTo(cp1, cp2, end);
                lastControl = cp2;
                current     = end;
                lastCommand = upper;
            }
            else if (upper == 'S')
            {
                double x2 = 0.0;
                double y2 = 0.0;
                double x  = 0.0;
                double y  = 0.0;
                if (!readNextNumber(pathData, pos, x2) || !readNextNumber(pathData, pos, y2) || !readNextNumber(pathData, pos, x) || !readNextNumber(pathData, pos, y))
                {
                    break;
                }
                if (!hasSubpath)
                {
                    activeSubpath = VectorSubpath(current);
                    hasSubpath    = true;
                    subpathStart  = current;
                }
                glm::dvec2 cp1 = (lastCommand == 'C' || lastCommand == 'S') ? (2.0 * current - lastControl) : current;
                glm::dvec2 cp2 = relative ? (current + glm::dvec2(x2, y2)) : glm::dvec2(x2, y2);
                glm::dvec2 end = relative ? (current + glm::dvec2(x, y)) : glm::dvec2(x, y);
                activeSubpath.cubicTo(cp1, cp2, end);
                lastControl = cp2;
                current     = end;
                lastCommand = upper;
            }
            else if (upper == 'Q')
            {
                double x1 = 0.0;
                double y1 = 0.0;
                double x  = 0.0;
                double y  = 0.0;
                if (!readNextNumber(pathData, pos, x1) || !readNextNumber(pathData, pos, y1) || !readNextNumber(pathData, pos, x) || !readNextNumber(pathData, pos, y))
                {
                    break;
                }
                if (!hasSubpath)
                {
                    activeSubpath = VectorSubpath(current);
                    hasSubpath    = true;
                    subpathStart  = current;
                }
                glm::dvec2 cp  = relative ? (current + glm::dvec2(x1, y1)) : glm::dvec2(x1, y1);
                glm::dvec2 end = relative ? (current + glm::dvec2(x, y)) : glm::dvec2(x, y);
                activeSubpath.quadraticTo(cp, end);
                lastControl = cp;
                current     = end;
                lastCommand = upper;
            }
            else if (upper == 'T')
            {
                double x = 0.0;
                double y = 0.0;
                if (!readNextNumber(pathData, pos, x) || !readNextNumber(pathData, pos, y))
                {
                    break;
                }
                if (!hasSubpath)
                {
                    activeSubpath = VectorSubpath(current);
                    hasSubpath    = true;
                    subpathStart  = current;
                }
                glm::dvec2 cp  = (lastCommand == 'Q' || lastCommand == 'T') ? (2.0 * current - lastControl) : current;
                glm::dvec2 end = relative ? (current + glm::dvec2(x, y)) : glm::dvec2(x, y);
                activeSubpath.quadraticTo(cp, end);
                lastControl = cp;
                current     = end;
                lastCommand = upper;
            }
            else if (upper == 'A')
            {
                double rx       = 0.0;
                double ry       = 0.0;
                double rotation = 0.0;
                bool   largeArc = false;
                bool   sweep    = false;
                double x        = 0.0;
                double y        = 0.0;
                if (!readNextNumber(pathData, pos, rx) || !readNextNumber(pathData, pos, ry) || !readNextNumber(pathData, pos, rotation) || !readNextFlag(pathData, pos, largeArc) ||
                    !readNextFlag(pathData, pos, sweep) || !readNextNumber(pathData, pos, x) || !readNextNumber(pathData, pos, y))
                {
                    break;
                }
                if (!hasSubpath)
                {
                    activeSubpath = VectorSubpath(current);
                    hasSubpath    = true;
                    subpathStart  = current;
                }
                glm::dvec2 end = relative ? (current + glm::dvec2(x, y)) : glm::dvec2(x, y);
                addArcAsCubics(activeSubpath, current, rx, ry, rotation, largeArc, sweep, end);
                current     = end;
                lastCommand = upper;
            }
            else
            {
                command = 0;
            }
        }

        if (hasSubpath && !activeSubpath.isEmpty())
        {
            outPath.addSubpath(activeSubpath);
        }
    }

    void SvgReader::addArcAsCubics(VectorSubpath&    subpath,
                                   const glm::dvec2& startPoint,
                                   double            rxInput,
                                   double            ryInput,
                                   double            xAxisRotationDeg,
                                   bool              largeArcFlag,
                                   bool              sweepFlag,
                                   const glm::dvec2& endPoint)
    {
        double rx = std::abs(rxInput);
        double ry = std::abs(ryInput);
        if (rx < 1e-9 || ry < 1e-9 || glm::length(endPoint - startPoint) < 1e-9)
        {
            subpath.lineTo(endPoint);
            return;
        }

        constexpr double pi     = 3.14159265358979323846;
        double           phi    = xAxisRotationDeg * (pi / 180.0);
        double           cosPhi = std::cos(phi);
        double           sinPhi = std::sin(phi);

        glm::dvec2 halfDelta = (startPoint - endPoint) * 0.5;
        double     x1p       = cosPhi * halfDelta.x + sinPhi * halfDelta.y;
        double     y1p       = -sinPhi * halfDelta.x + cosPhi * halfDelta.y;

        double lambda = (x1p * x1p) / (rx * rx) + (y1p * y1p) / (ry * ry);
        if (lambda > 1.0)
        {
            double scale = std::sqrt(lambda);
            rx *= scale;
            ry *= scale;
        }

        double rx2   = rx * rx;
        double ry2   = ry * ry;
        double x1p2  = x1p * x1p;
        double y1p2  = y1p * y1p;
        double denom = rx2 * y1p2 + ry2 * x1p2;
        double num   = std::max(0.0, rx2 * ry2 - denom);
        double coeff = (denom > 1e-12) ? std::sqrt(num / denom) : 0.0;
        if (largeArcFlag == sweepFlag)
        {
            coeff = -coeff;
        }

        double cxp = coeff * (rx * y1p / ry);
        double cyp = coeff * (-ry * x1p / rx);

        glm::dvec2 mid = (startPoint + endPoint) * 0.5;
        glm::dvec2 center(cosPhi * cxp - sinPhi * cyp + mid.x, sinPhi * cxp + cosPhi * cyp + mid.y);

        glm::dvec2 u((x1p - cxp) / rx, (y1p - cyp) / ry);
        glm::dvec2 v((-x1p - cxp) / rx, (-y1p - cyp) / ry);

        double startAngle = std::atan2(u.y, u.x);
        double deltaAngle = std::atan2(u.x * v.y - u.y * v.x, u.x * v.x + u.y * v.y);
        if (!sweepFlag && deltaAngle > 0.0)
        {
            deltaAngle -= 2.0 * pi;
        }
        else if (sweepFlag && deltaAngle < 0.0)
        {
            deltaAngle += 2.0 * pi;
        }

        int    segments = std::max(1, static_cast<int>(std::ceil(std::abs(deltaAngle) / (pi * 0.5))));
        double step     = deltaAngle / static_cast<double>(segments);
        double alpha    = (4.0 / 3.0) * std::tan(step * 0.25);

        for (int i = 0; i < segments; i++)
        {
            double t1 = startAngle + static_cast<double>(i) * step;
            double t2 = t1 + step;

            glm::dvec2 p1(std::cos(t1), std::sin(t1));
            glm::dvec2 p2(std::cos(t2), std::sin(t2));
            glm::dvec2 q1 = p1 + alpha * glm::dvec2(-p1.y, p1.x);
            glm::dvec2 q2 = p2 - alpha * glm::dvec2(-p2.y, p2.x);

            glm::dvec2 cp1(cosPhi * (rx * q1.x) - sinPhi * (ry * q1.y) + center.x, sinPhi * (rx * q1.x) + cosPhi * (ry * q1.y) + center.y);
            glm::dvec2 cp2(cosPhi * (rx * q2.x) - sinPhi * (ry * q2.y) + center.x, sinPhi * (rx * q2.x) + cosPhi * (ry * q2.y) + center.y);
            glm::dvec2 ep(cosPhi * (rx * p2.x) - sinPhi * (ry * p2.y) + center.x, sinPhi * (rx * p2.x) + cosPhi * (ry * p2.y) + center.y);
            if (i + 1 == segments)
            {
                ep = endPoint;
            }
            subpath.cubicTo(cp1, cp2, ep);
        }
    }

    void SvgReader::addEllipseSubpath(VectorPath& outPath, double cx, double cy, double rx, double ry)
    {
        constexpr double kappa = 0.5522847498307936;
        double           kx    = rx * kappa;
        double           ky    = ry * kappa;

        VectorSubpath subpath(glm::dvec2(cx + rx, cy));
        subpath.cubicTo(glm::dvec2(cx + rx, cy + ky), glm::dvec2(cx + kx, cy + ry), glm::dvec2(cx, cy + ry));
        subpath.cubicTo(glm::dvec2(cx - kx, cy + ry), glm::dvec2(cx - rx, cy + ky), glm::dvec2(cx - rx, cy));
        subpath.cubicTo(glm::dvec2(cx - rx, cy - ky), glm::dvec2(cx - kx, cy - ry), glm::dvec2(cx, cy - ry));
        subpath.cubicTo(glm::dvec2(cx + kx, cy - ry), glm::dvec2(cx + rx, cy - ky), glm::dvec2(cx + rx, cy));
        subpath.close();
        outPath.addSubpath(subpath);
    }

    VectorStyle SvgReader::parseStyle(const SvgXmlNode& node, const VectorStyle& inheritedStyle)
    {
        VectorStyle                        style       = inheritedStyle;
        std::map<std::string, std::string> inlineStyle = parseInlineStyle(node.getAttribute("style"));

        auto getProp = [&](const std::string& name, std::string& outValue) -> bool
        {
            auto it = inlineStyle.find(name);
            if (it != inlineStyle.end())
            {
                outValue = it->second;
                return true;
            }
            if (node.hasAttribute(name))
            {
                outValue = node.getAttribute(name);
                return true;
            }
            return false;
        };

        std::string value;
        if (getProp("fill", value))
        {
            glm::u8vec4 color(0, 0, 0, 255);
            bool        isNone = false;
            if (parseColor(value, color, isNone))
            {
                style.setHasFill(!isNone);
                if (!isNone)
                {
                    style.setFillColor(color);
                }
            }
        }

        if (getProp("fill-opacity", value))
        {
            double      alpha = std::clamp(parseLength(value, 1.0), 0.0, 1.0);
            glm::u8vec4 c     = style.getFillColor();
            c.a               = static_cast<uint8_t>(std::round(alpha * 255.0));
            style.setFillColor(c);
        }

        if (getProp("fill-rule", value))
        {
            std::string lower = toLower(trim(value));
            if (lower == "evenodd")
            {
                style.setFillRule(VectorFillRule::EvenOdd);
            }
            else if (lower == "nonzero")
            {
                style.setFillRule(VectorFillRule::NonZero);
            }
        }

        if (getProp("stroke", value))
        {
            glm::u8vec4 color(0, 0, 0, 255);
            bool        isNone = false;
            if (parseColor(value, color, isNone))
            {
                style.setHasStroke(!isNone);
                if (!isNone)
                {
                    style.setStrokeColor(color);
                }
            }
        }

        if (getProp("stroke-opacity", value))
        {
            double      alpha = std::clamp(parseLength(value, 1.0), 0.0, 1.0);
            glm::u8vec4 c     = style.getStrokeColor();
            c.a               = static_cast<uint8_t>(std::round(alpha * 255.0));
            style.setStrokeColor(c);
        }

        if (getProp("stroke-width", value))
        {
            style.setStrokeWidth(std::max(0.0, parseLength(value, 1.0)));
        }

        if (getProp("opacity", value))
        {
            style.setOpacity(std::clamp(parseLength(value, 1.0), 0.0, 1.0));
        }

        return style;
    }

    std::map<std::string, std::string> SvgReader::parseInlineStyle(const std::string& styleStr)
    {
        std::map<std::string, std::string> props;
        size_t                             pos = 0;
        while (pos < styleStr.size())
        {
            size_t semi = styleStr.find(';', pos);
            if (semi == std::string::npos)
            {
                semi = styleStr.size();
            }
            std::string entry = styleStr.substr(pos, semi - pos);
            size_t      colon = entry.find(':');
            if (colon != std::string::npos)
            {
                std::string key = toLower(trim(entry.substr(0, colon)));
                std::string val = trim(entry.substr(colon + 1));
                if (!key.empty())
                {
                    props[key] = val;
                }
            }
            pos = semi + 1;
        }
        return props;
    }

    bool SvgReader::parseColor(const std::string& colorStr, glm::u8vec4& outColor, bool& isNone)
    {
        std::string s = toLower(trim(colorStr));
        isNone        = false;
        if (s.empty() || s == "none")
        {
            isNone = true;
            return true;
        }
        if (s == "transparent")
        {
            outColor = glm::u8vec4(0, 0, 0, 0);
            return true;
        }

        if (s[0] == '#')
        {
            auto hexVal = [](char c) -> int
            {
                if (c >= '0' && c <= '9')
                {
                    return c - '0';
                }
                if (c >= 'a' && c <= 'f')
                {
                    return 10 + (c - 'a');
                }
                return -1;
            };

            if (s.size() == 4)
            {
                int r = hexVal(s[1]);
                int g = hexVal(s[2]);
                int b = hexVal(s[3]);
                if (r >= 0 && g >= 0 && b >= 0)
                {
                    outColor = glm::u8vec4(r * 17, g * 17, b * 17, 255);
                    return true;
                }
            }
            else if (s.size() == 7)
            {
                int r1 = hexVal(s[1]);
                int r2 = hexVal(s[2]);
                int g1 = hexVal(s[3]);
                int g2 = hexVal(s[4]);
                int b1 = hexVal(s[5]);
                int b2 = hexVal(s[6]);
                if (r1 >= 0 && r2 >= 0 && g1 >= 0 && g2 >= 0 && b1 >= 0 && b2 >= 0)
                {
                    outColor = glm::u8vec4(r1 * 16 + r2, g1 * 16 + g2, b1 * 16 + b2, 255);
                    return true;
                }
            }
            return false;
        }

        if (s.rfind("rgb(", 0) == 0 || s.rfind("rgba(", 0) == 0)
        {
            size_t openParen  = s.find('(');
            size_t closeParen = s.find(')');
            if (openParen != std::string::npos && closeParen != std::string::npos && closeParen > openParen)
            {
                std::vector<double> vals = parseNumberList(s.substr(openParen + 1, closeParen - openParen - 1));
                if (vals.size() >= 3)
                {
                    uint8_t r = static_cast<uint8_t>(std::clamp(std::round(vals[0]), 0.0, 255.0));
                    uint8_t g = static_cast<uint8_t>(std::clamp(std::round(vals[1]), 0.0, 255.0));
                    uint8_t b = static_cast<uint8_t>(std::clamp(std::round(vals[2]), 0.0, 255.0));
                    uint8_t a = 255;
                    if (vals.size() >= 4)
                    {
                        double alpha = vals[3] <= 1.0 ? vals[3] * 255.0 : vals[3];
                        a            = static_cast<uint8_t>(std::clamp(std::round(alpha), 0.0, 255.0));
                    }
                    outColor = glm::u8vec4(r, g, b, a);
                    return true;
                }
            }
            return false;
        }

        static const std::map<std::string, glm::u8vec4> namedColors = {
            {   "black",   glm::u8vec4(0,   0,   0, 255) },
            {   "white", glm::u8vec4(255, 255, 255, 255) },
            {     "red", glm::u8vec4(255,   0,   0, 255) },
            {   "green",   glm::u8vec4(0, 128,   0, 255) },
            {    "lime",   glm::u8vec4(0, 255,   0, 255) },
            {    "blue",   glm::u8vec4(0,   0, 255, 255) },
            {  "yellow", glm::u8vec4(255, 255,   0, 255) },
            {    "cyan",   glm::u8vec4(0, 255, 255, 255) },
            {    "aqua",   glm::u8vec4(0, 255, 255, 255) },
            { "magenta", glm::u8vec4(255,   0, 255, 255) },
            { "fuchsia", glm::u8vec4(255,   0, 255, 255) },
            {    "gray", glm::u8vec4(128, 128, 128, 255) },
            {    "grey", glm::u8vec4(128, 128, 128, 255) },
            {  "silver", glm::u8vec4(192, 192, 192, 255) },
            {  "orange", glm::u8vec4(255, 165,   0, 255) },
            {  "purple", glm::u8vec4(128,   0, 128, 255) },
            {    "navy",   glm::u8vec4(0,   0, 128, 255) },
            {    "teal",   glm::u8vec4(0, 128, 128, 255) },
            {  "maroon", glm::u8vec4(128,   0,   0, 255) },
            {   "olive", glm::u8vec4(128, 128,   0, 255) }
        };

        auto it = namedColors.find(s);
        if (it != namedColors.end())
        {
            outColor = it->second;
            return true;
        }

        return false;
    }

    glm::dmat3 SvgReader::parseTransform(const std::string& transformStr)
    {
        glm::dmat3       result(1.0);
        constexpr double pi  = 3.14159265358979323846;
        size_t           pos = 0;

        while (pos < transformStr.size())
        {
            while (pos < transformStr.size() && (std::isspace(static_cast<unsigned char>(transformStr[pos])) || transformStr[pos] == ','))
            {
                pos++;
            }
            if (pos >= transformStr.size())
            {
                break;
            }

            size_t nameStart = pos;
            while (pos < transformStr.size() && std::isalpha(static_cast<unsigned char>(transformStr[pos])))
            {
                pos++;
            }
            std::string funcName = toLower(transformStr.substr(nameStart, pos - nameStart));

            size_t openParen  = transformStr.find('(', pos);
            size_t closeParen = transformStr.find(')', openParen == std::string::npos ? pos : openParen);
            if (openParen == std::string::npos || closeParen == std::string::npos)
            {
                break;
            }

            std::vector<double> args = parseNumberList(transformStr.substr(openParen + 1, closeParen - openParen - 1));
            pos                      = closeParen + 1;

            glm::dmat3 m(1.0);
            if (funcName == "matrix" && args.size() >= 6)
            {
                m[0][0] = args[0];
                m[0][1] = args[1];
                m[1][0] = args[2];
                m[1][1] = args[3];
                m[2][0] = args[4];
                m[2][1] = args[5];
            }
            else if (funcName == "translate" && !args.empty())
            {
                m[2][0] = args[0];
                m[2][1] = args.size() >= 2 ? args[1] : 0.0;
            }
            else if (funcName == "scale" && !args.empty())
            {
                m[0][0] = args[0];
                m[1][1] = args.size() >= 2 ? args[1] : args[0];
            }
            else if (funcName == "rotate" && !args.empty())
            {
                double rad = args[0] * (pi / 180.0);
                double c   = std::cos(rad);
                double s   = std::sin(rad);
                m[0][0]    = c;
                m[0][1]    = s;
                m[1][0]    = -s;
                m[1][1]    = c;
                if (args.size() >= 3)
                {
                    glm::dmat3 t1(1.0);
                    t1[2][0] = args[1];
                    t1[2][1] = args[2];
                    glm::dmat3 t2(1.0);
                    t2[2][0] = -args[1];
                    t2[2][1] = -args[2];
                    m        = t1 * m * t2;
                }
            }
            else if (funcName == "skewx" && !args.empty())
            {
                m[1][0] = std::tan(args[0] * (pi / 180.0));
            }
            else if (funcName == "skewy" && !args.empty())
            {
                m[0][1] = std::tan(args[0] * (pi / 180.0));
            }

            result = result * m;
        }

        return result;
    }

    std::vector<double> SvgReader::parseNumberList(const std::string& text)
    {
        std::vector<double> values;
        size_t              pos   = 0;
        double              value = 0.0;
        while (readNextNumber(text, pos, value))
        {
            values.push_back(value);
        }
        return values;
    }

    double SvgReader::parseLength(const std::string& text, double defaultValue)
    {
        size_t pos   = 0;
        double value = defaultValue;
        if (!readNextNumber(text, pos, value))
        {
            return defaultValue;
        }
        return value;
    }

    bool SvgReader::readNextNumber(const std::string& text, size_t& pos, double& outValue)
    {
        skipSeparators(text, pos);
        if (pos >= text.size())
        {
            return false;
        }

        size_t start = pos;
        if (text[pos] == '+' || text[pos] == '-')
        {
            pos++;
        }

        bool hasDigits = false;
        while (pos < text.size() && std::isdigit(static_cast<unsigned char>(text[pos])))
        {
            hasDigits = true;
            pos++;
        }

        if (pos < text.size() && text[pos] == '.')
        {
            pos++;
            while (pos < text.size() && std::isdigit(static_cast<unsigned char>(text[pos])))
            {
                hasDigits = true;
                pos++;
            }
        }

        if (!hasDigits)
        {
            pos = start;
            return false;
        }

        if (pos < text.size() && (text[pos] == 'e' || text[pos] == 'E'))
        {
            size_t expPos = pos + 1;
            if (expPos < text.size() && (text[expPos] == '+' || text[expPos] == '-'))
            {
                expPos++;
            }
            if (expPos < text.size() && std::isdigit(static_cast<unsigned char>(text[expPos])))
            {
                pos = expPos;
                while (pos < text.size() && std::isdigit(static_cast<unsigned char>(text[pos])))
                {
                    pos++;
                }
            }
        }

        try
        {
            outValue = std::stod(text.substr(start, pos - start));
            return true;
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    bool SvgReader::readNextFlag(const std::string& text, size_t& pos, bool& outFlag)
    {
        skipSeparators(text, pos);
        if (pos >= text.size())
        {
            return false;
        }
        if (text[pos] == '0')
        {
            outFlag = false;
            pos++;
            return true;
        }
        if (text[pos] == '1')
        {
            outFlag = true;
            pos++;
            return true;
        }
        return false;
    }

    void SvgReader::skipSeparators(const std::string& text, size_t& pos)
    {
        while (pos < text.size() && (std::isspace(static_cast<unsigned char>(text[pos])) || text[pos] == ','))
        {
            pos++;
        }
    }

    std::string SvgReader::trim(const std::string& text)
    {
        size_t start = 0;
        while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start])))
        {
            start++;
        }
        size_t end = text.size();
        while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1])))
        {
            end--;
        }
        return text.substr(start, end - start);
    }

    std::string SvgReader::toLower(const std::string& text)
    {
        std::string result = text;
        for (char& c : result)
        {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        return result;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/VectorGraphics/Svg/SvgWriter.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("SvgReader/RoundTripShapesGroupAndText")
{
    std::string svg = R"svg(<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 80">
  <defs><linearGradient id="ignored"/></defs>
  <g id="layer1" transform="translate(10, 5)">
    <rect id="box" x="0" y="0" width="20" height="10" fill="#ff0000"/>
    <path id="curve" d="M 0 0 L 10 0 Q 15 5 10 10 Z" fill="#00ff00" fill-rule="evenodd"/>
    <text id="label" x="5" y="15" font-family="sans-serif" font-size="12">Hammer</text>
  </g>
</svg>)svg";

    CADacomb::VectorGraphics::VectorGraphics doc = CADacomb::VectorGraphics::SvgReader::loadFromString(svg);
    REQUIRE(doc.hasViewBox());
    CHECK(doc.getViewBoxSize() == glm::dvec2(100.0, 80.0));
    REQUIRE(doc.getElements().size() == 1);
    CHECK(doc.getElements()[0]->getType() == CADacomb::VectorGraphics::VectorElementType::Group);

    auto group = std::dynamic_pointer_cast<CADacomb::VectorGraphics::VectorGroup>(doc.getElements()[0]);
    REQUIRE(group != nullptr);
    REQUIRE(group->getChildren().size() == 3);
    CHECK(group->getChildren()[0]->getType() == CADacomb::VectorGraphics::VectorElementType::Path);
    CHECK(group->getChildren()[1]->getType() == CADacomb::VectorGraphics::VectorElementType::Path);
    CHECK(group->getChildren()[2]->getType() == CADacomb::VectorGraphics::VectorElementType::Text);

    auto textElem = std::dynamic_pointer_cast<CADacomb::VectorGraphics::VectorText>(group->getChildren()[2]);
    REQUIRE(textElem != nullptr);
    CHECK(textElem->getText() == "Hammer");
    CHECK(textElem->getFontFamily() == "sans-serif");
    CHECK(textElem->getFontSize() == Catch::Approx(12.0));

    std::string                              saved    = CADacomb::VectorGraphics::SvgWriter::saveToString(doc);
    CADacomb::VectorGraphics::VectorGraphics reloaded = CADacomb::VectorGraphics::SvgReader::loadFromString(saved);
    REQUIRE(reloaded.getElements().size() == 1);
    auto reloadedGroup = std::dynamic_pointer_cast<CADacomb::VectorGraphics::VectorGroup>(reloaded.getElements()[0]);
    REQUIRE(reloadedGroup != nullptr);
    CHECK(reloadedGroup->getChildren().size() == 3);
}
#endif
