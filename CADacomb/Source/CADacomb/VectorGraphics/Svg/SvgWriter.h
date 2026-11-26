#pragma once

#include "CADacomb/VectorGraphics/Data/VectorGraphics.h"

#include <glm/glm.hpp>
#include <ostream>
#include <string>

namespace CADacomb::VectorGraphics
{
    class VectorElement;
    class VectorGroup;
    class VectorPath;
    class VectorStyle;
    class VectorText;

    // Serializes a VectorGraphics document to standard SVG XML text or files.
    class SvgWriter
    {
      public:
        SvgWriter();
        virtual ~SvgWriter();

        static void        saveToFile(const VectorGraphics& document, const std::string& filePath);
        static std::string saveToString(const VectorGraphics& document);

      private:
        static void        writeElement(std::ostream& out, const VectorElement& element, int indent);
        static void        writeGroup(std::ostream& out, const VectorGroup& group, int indent);
        static void        writePath(std::ostream& out, const VectorPath& path, int indent);
        static void        writeText(std::ostream& out, const VectorText& textElement, int indent);
        static void        writeCommonAttributes(std::ostream& out, const VectorElement& element);
        static void        writeStyleAttributes(std::ostream& out, const VectorStyle& style);
        static std::string formatColor(const glm::u8vec4& color);
        static std::string formatTransform(const glm::dmat3& matrix);
        static bool        isIdentityTransform(const glm::dmat3& matrix);
        static std::string escapeXml(const std::string& text);
        static std::string makeIndent(int indent);
    };
}
