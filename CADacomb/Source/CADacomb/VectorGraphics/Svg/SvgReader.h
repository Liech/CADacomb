#pragma once

#include "CADacomb/VectorGraphics/Svg/SvgXmlNode.h"
#include "CADacomb/VectorGraphics/Data/VectorGraphics.h"
#include "CADacomb/VectorGraphics/Data/VectorPath.h"
#include "CADacomb/VectorGraphics/Data/VectorStyle.h"

#include <glm/glm.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace CADacomb::VectorGraphics
{
    // Zero-dependency parser loading SVG files or strings into a VectorGraphics document.
    class SvgReader
    {
      public:
        SvgReader();
        virtual ~SvgReader();

        static VectorGraphics loadFromFile(const std::string& filePath);
        static VectorGraphics loadFromString(const std::string& svgContent);

      private:
        static std::shared_ptr<SvgXmlNode> parseXmlRoot(const std::string& content);
        static std::shared_ptr<SvgXmlNode> parseXmlElement(const std::string& content, size_t& pos);
        static void                        skipWhitespaceAndComments(const std::string& content, size_t& pos);
        static std::string                 unescapeXml(const std::string& text);
        static std::string                 stripNamespacePrefix(const std::string& tagName);

        static std::shared_ptr<VectorElement> convertNode(const SvgXmlNode& node, const VectorStyle& inheritedStyle);
        static std::shared_ptr<VectorPath>    parsePathElement(const SvgXmlNode& node, const VectorStyle& style);
        static std::shared_ptr<VectorPath>    parseRectElement(const SvgXmlNode& node, const VectorStyle& style);
        static std::shared_ptr<VectorPath>    parseCircleElement(const SvgXmlNode& node, const VectorStyle& style);
        static std::shared_ptr<VectorPath>    parseEllipseElement(const SvgXmlNode& node, const VectorStyle& style);
        static std::shared_ptr<VectorPath>    parseLineElement(const SvgXmlNode& node, const VectorStyle& style);
        static std::shared_ptr<VectorPath>    parsePolyElement(const SvgXmlNode& node, const VectorStyle& style, bool closed);
        static std::shared_ptr<VectorElement> parseTextElement(const SvgXmlNode& node, const VectorStyle& style);
        static std::string                    collectTextContent(const SvgXmlNode& node);

        static void parsePathData(const std::string& pathData, VectorPath& outPath);
        static void addArcAsCubics(VectorSubpath& subpath, const glm::dvec2& startPoint, double rx, double ry, double xAxisRotationDeg, bool largeArcFlag, bool sweepFlag, const glm::dvec2& endPoint);
        static void addEllipseSubpath(VectorPath& outPath, double cx, double cy, double rx, double ry);

        static VectorStyle                        parseStyle(const SvgXmlNode& node, const VectorStyle& inheritedStyle);
        static std::map<std::string, std::string> parseInlineStyle(const std::string& styleStr);
        static bool                               parseColor(const std::string& colorStr, glm::u8vec4& outColor, bool& isNone);
        static glm::dmat3                         parseTransform(const std::string& transformStr);

        static std::vector<double> parseNumberList(const std::string& text);
        static double              parseLength(const std::string& text, double defaultValue);
        static bool                readNextNumber(const std::string& text, size_t& pos, double& outValue);
        static bool                readNextFlag(const std::string& text, size_t& pos, bool& outFlag);
        static void                skipSeparators(const std::string& text, size_t& pos);
        static std::string         trim(const std::string& text);
        static std::string         toLower(const std::string& text);
    };
}
