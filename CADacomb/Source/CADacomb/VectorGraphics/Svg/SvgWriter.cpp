#include "CADacomb/VectorGraphics/Svg/SvgWriter.h"

#include "CADacomb/VectorGraphics/Data/VectorGroup.h"
#include "CADacomb/VectorGraphics/Data/VectorPath.h"
#include "CADacomb/VectorGraphics/Data/VectorText.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace CADacomb::VectorGraphics
{
    SvgWriter::SvgWriter() {}

    SvgWriter::~SvgWriter() {}

    void SvgWriter::saveToFile(const VectorGraphics& document, const std::string& filePath)
    {
        std::ofstream file(filePath, std::ios::binary);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open SVG file for writing: " + filePath);
        }
        std::string content = saveToString(document);
        file.write(content.data(), static_cast<std::streamsize>(content.size()));
    }

    std::string SvgWriter::saveToString(const VectorGraphics& document)
    {
        std::ostringstream out;
        out << std::setprecision(6);

        glm::dvec2 origin(0.0, 0.0);
        glm::dvec2 size(100.0, 100.0);
        if (document.hasViewBox())
        {
            origin = document.getViewBoxOrigin();
            size   = document.getViewBoxSize();
        }
        else
        {
            VectorBounds bounds = document.computeBounds();
            if (bounds.isValid())
            {
                origin = bounds.getMin();
                size   = bounds.getSize();
            }
        }

        out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        out << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"" << origin.x << " " << origin.y << " " << size.x << " " << size.y << "\" width=\"" << size.x << "\" height=\"" << size.y
            << "\">\n";

        for (const std::shared_ptr<VectorElement>& element : document.getElements())
        {
            writeElement(out, *element, 1);
        }

        out << "</svg>\n";
        return out.str();
    }

    void SvgWriter::writeElement(std::ostream& out, const VectorElement& element, int indent)
    {
        switch (element.getType())
        {
            case VectorElementType::Group:
                writeGroup(out, static_cast<const VectorGroup&>(element), indent);
                break;
            case VectorElementType::Path:
                writePath(out, static_cast<const VectorPath&>(element), indent);
                break;
            case VectorElementType::Text:
                writeText(out, static_cast<const VectorText&>(element), indent);
                break;
        }
    }

    void SvgWriter::writeGroup(std::ostream& out, const VectorGroup& group, int indent)
    {
        out << makeIndent(indent) << "<g";
        writeCommonAttributes(out, group);
        out << ">\n";

        for (const std::shared_ptr<VectorElement>& child : group.getChildren())
        {
            writeElement(out, *child, indent + 1);
        }

        out << makeIndent(indent) << "</g>\n";
    }

    void SvgWriter::writePath(std::ostream& out, const VectorPath& path, int indent)
    {
        out << makeIndent(indent) << "<path";
        writeCommonAttributes(out, path);
        writeStyleAttributes(out, path.getStyle());

        out << " d=\"";
        bool firstSubpath = true;
        for (const VectorSubpath& subpath : path.getSubpaths())
        {
            if (!firstSubpath)
            {
                out << " ";
            }
            firstSubpath = false;

            const glm::dvec2& start = subpath.getStartPoint();
            out << "M " << start.x << " " << start.y;

            for (const VectorSegment& segment : subpath.getSegments())
            {
                if (segment.getType() == VectorSegmentType::Line)
                {
                    const glm::dvec2& end = segment.getEndPoint();
                    out << " L " << end.x << " " << end.y;
                }
                else
                {
                    const glm::dvec2& cp1 = segment.getControlPoint1();
                    const glm::dvec2& cp2 = segment.getControlPoint2();
                    const glm::dvec2& end = segment.getEndPoint();
                    out << " C " << cp1.x << " " << cp1.y << " " << cp2.x << " " << cp2.y << " " << end.x << " " << end.y;
                }
            }

            if (subpath.isClosed())
            {
                out << " Z";
            }
        }
        out << "\"/>\n";
    }

    void SvgWriter::writeText(std::ostream& out, const VectorText& textElement, int indent)
    {
        out << makeIndent(indent) << "<text";
        writeCommonAttributes(out, textElement);
        writeStyleAttributes(out, textElement.getStyle());

        const glm::dvec2& pos = textElement.getPosition();
        out << " x=\"" << pos.x << "\" y=\"" << pos.y << "\"";
        if (!textElement.getFontFamily().empty())
        {
            out << " font-family=\"" << escapeXml(textElement.getFontFamily()) << "\"";
        }
        out << " font-size=\"" << textElement.getFontSize() << "\"";
        out << ">" << escapeXml(textElement.getText()) << "</text>\n";
    }

    void SvgWriter::writeCommonAttributes(std::ostream& out, const VectorElement& element)
    {
        if (!element.getId().empty())
        {
            out << " id=\"" << escapeXml(element.getId()) << "\"";
        }
        if (!isIdentityTransform(element.getTransform()))
        {
            out << " transform=\"" << formatTransform(element.getTransform()) << "\"";
        }
    }

    void SvgWriter::writeStyleAttributes(std::ostream& out, const VectorStyle& style)
    {
        if (style.hasFill())
        {
            out << " fill=\"" << formatColor(style.getFillColor()) << "\"";
            if (style.getFillColor().a < 255)
            {
                out << " fill-opacity=\"" << (static_cast<double>(style.getFillColor().a) / 255.0) << "\"";
            }
        }
        else
        {
            out << " fill=\"none\"";
        }

        if (style.getFillRule() == VectorFillRule::EvenOdd)
        {
            out << " fill-rule=\"evenodd\"";
        }

        if (style.hasStroke())
        {
            out << " stroke=\"" << formatColor(style.getStrokeColor()) << "\"";
            out << " stroke-width=\"" << style.getStrokeWidth() << "\"";
            if (style.getStrokeColor().a < 255)
            {
                out << " stroke-opacity=\"" << (static_cast<double>(style.getStrokeColor().a) / 255.0) << "\"";
            }
        }

        if (style.getOpacity() < 1.0)
        {
            out << " opacity=\"" << style.getOpacity() << "\"";
        }
    }

    std::string SvgWriter::formatColor(const glm::u8vec4& color)
    {
        std::ostringstream hex;
        hex << "#" << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(color.r) << std::setw(2) << static_cast<int>(color.g) << std::setw(2) << static_cast<int>(color.b);
        return hex.str();
    }

    std::string SvgWriter::formatTransform(const glm::dmat3& matrix)
    {
        std::ostringstream out;
        out << std::setprecision(6);
        out << "matrix(" << matrix[0][0] << " " << matrix[0][1] << " " << matrix[1][0] << " " << matrix[1][1] << " " << matrix[2][0] << " " << matrix[2][1] << ")";
        return out.str();
    }

    bool SvgWriter::isIdentityTransform(const glm::dmat3& matrix)
    {
        constexpr double eps = 1e-9;
        return std::abs(matrix[0][0] - 1.0) < eps && std::abs(matrix[0][1]) < eps && std::abs(matrix[1][0]) < eps && std::abs(matrix[1][1] - 1.0) < eps && std::abs(matrix[2][0]) < eps &&
               std::abs(matrix[2][1]) < eps;
    }

    std::string SvgWriter::escapeXml(const std::string& text)
    {
        std::string result;
        result.reserve(text.size());
        for (char c : text)
        {
            if (c == '&')
            {
                result += "&amp;";
            }
            else if (c == '<')
            {
                result += "&lt;";
            }
            else if (c == '>')
            {
                result += "&gt;";
            }
            else if (c == '"')
            {
                result += "&quot;";
            }
            else if (c == '\'')
            {
                result += "&apos;";
            }
            else
            {
                result += c;
            }
        }
        return result;
    }

    std::string SvgWriter::makeIndent(int indent)
    {
        return std::string(static_cast<size_t>(indent) * 2, ' ');
    }
}
