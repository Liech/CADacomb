#include "CADacomb/VectorGraphics/Rasterizer/VectorGraphicsRasterizer.h"

#include "CADacomb/VectorGraphics/Data/VectorGroup.h"
#include "CADacomb/VectorGraphics/Data/VectorPath.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace CADacomb::VectorGraphics
{
    VectorGraphicsRasterizer::VectorGraphicsRasterizer() {}

    VectorGraphicsRasterizer::~VectorGraphicsRasterizer() {}

    CADacomb::Texture::Texture VectorGraphicsRasterizer::rasterize(const VectorGraphics& document, uint32_t width, uint32_t height, const glm::u8vec4& backgroundColor)
    {
        CADacomb::Texture::Texture target(width, height, backgroundColor);
        if (width == 0 || height == 0 || document.isEmpty())
        {
            return target;
        }

        glm::dvec2 docOrigin(0.0, 0.0);
        glm::dvec2 docSize(1.0, 1.0);
        if (document.hasViewBox() && document.getViewBoxSize().x > 0.0 && document.getViewBoxSize().y > 0.0)
        {
            docOrigin = document.getViewBoxOrigin();
            docSize   = document.getViewBoxSize();
        }
        else
        {
            VectorBounds bounds = document.computeBounds();
            if (!bounds.isValid())
            {
                return target;
            }
            docOrigin = bounds.getMin();
            docSize   = bounds.getSize();
            if (docSize.x <= 1e-9)
            {
                docSize.x = 1.0;
            }
            if (docSize.y <= 1e-9)
            {
                docSize.y = 1.0;
            }
        }

        double     scaleX = static_cast<double>(width) / docSize.x;
        double     scaleY = static_cast<double>(height) / docSize.y;
        glm::dmat3 viewTransform(1.0);
        viewTransform[0][0] = scaleX;
        viewTransform[1][1] = scaleY;
        viewTransform[2][0] = -docOrigin.x * scaleX;
        viewTransform[2][1] = -docOrigin.y * scaleY;

        for (const std::shared_ptr<VectorElement>& element : document.getElements())
        {
            renderElement(*element, viewTransform, target);
        }

        return target;
    }

    void VectorGraphicsRasterizer::renderElement(const VectorElement& element, const glm::dmat3& parentTransform, CADacomb::Texture::Texture& target)
    {
        glm::dmat3 combinedTransform = parentTransform * element.getTransform();

        if (element.getType() == VectorElementType::Group)
        {
            const auto& group = static_cast<const VectorGroup&>(element);
            for (const std::shared_ptr<VectorElement>& child : group.getChildren())
            {
                renderElement(*child, combinedTransform, target);
            }
        }
        else if (element.getType() == VectorElementType::Path)
        {
            renderPath(static_cast<const VectorPath&>(element), combinedTransform, target);
        }
    }

    void VectorGraphicsRasterizer::renderPath(const VectorPath& path, const glm::dmat3& transform, CADacomb::Texture::Texture& target)
    {
        const VectorStyle& style = path.getStyle();
        if (!style.hasFill() && !style.hasStroke())
        {
            return;
        }

        std::vector<std::vector<glm::dvec2>> polylines;
        std::vector<bool>                    closedFlags;
        polylines.reserve(path.getSubpaths().size());
        closedFlags.reserve(path.getSubpaths().size());

        for (const VectorSubpath& subpath : path.getSubpaths())
        {
            VectorSubpath           transformed = subpath.transformed(transform);
            std::vector<glm::dvec2> flat        = transformed.flatten(0.25);
            if (flat.size() >= 2)
            {
                polylines.push_back(std::move(flat));
                closedFlags.push_back(subpath.isClosed());
            }
        }

        if (polylines.empty())
        {
            return;
        }

        double opacity = std::clamp(style.getOpacity(), 0.0, 1.0);
        if (style.hasFill())
        {
            glm::u8vec4 fill = style.getFillColor();
            fill.a           = static_cast<uint8_t>(std::round(static_cast<double>(fill.a) * opacity));
            if (fill.a > 0)
            {
                rasterizeFill(polylines, style.getFillRule(), fill, target);
            }
        }

        if (style.hasStroke() && style.getStrokeWidth() > 0.0)
        {
            glm::u8vec4 stroke = style.getStrokeColor();
            stroke.a           = static_cast<uint8_t>(std::round(static_cast<double>(stroke.a) * opacity));
            if (stroke.a > 0)
            {
                double scaleEstimate = std::sqrt(std::abs(transform[0][0] * transform[1][1] - transform[0][1] * transform[1][0]));
                double pixelWidth    = std::max(1.0, style.getStrokeWidth() * scaleEstimate);
                rasterizeStroke(polylines, closedFlags, pixelWidth, stroke, target);
            }
        }
    }

    void VectorGraphicsRasterizer::rasterizeFill(const std::vector<std::vector<glm::dvec2>>& polylines, VectorFillRule fillRule, const glm::u8vec4& color, CADacomb::Texture::Texture& target)
    {
        uint32_t width  = target.getWidth();
        uint32_t height = target.getHeight();

        double minY = static_cast<double>(height);
        double maxY = 0.0;
        for (const std::vector<glm::dvec2>& poly : polylines)
        {
            for (const glm::dvec2& p : poly)
            {
                minY = std::min(minY, p.y);
                maxY = std::max(maxY, p.y);
            }
        }

        int startY = std::max(0, static_cast<int>(std::floor(minY)));
        int endY   = std::min(static_cast<int>(height) - 1, static_cast<int>(std::ceil(maxY)));
        if (startY > endY)
        {
            return;
        }

        constexpr int                       subSamples = 2;
        std::vector<double>                 coverageRow(width, 0.0);
        std::vector<std::pair<double, int>> crossings;

        for (int py = startY; py <= endY; py++)
        {
            std::fill(coverageRow.begin(), coverageRow.end(), 0.0);

            for (int sy = 0; sy < subSamples; sy++)
            {
                double scanY = static_cast<double>(py) + (static_cast<double>(sy) + 0.5) / static_cast<double>(subSamples);
                crossings.clear();

                for (const std::vector<glm::dvec2>& poly : polylines)
                {
                    size_t count = poly.size();
                    for (size_t i = 0; i < count; i++)
                    {
                        const glm::dvec2& a = poly[i];
                        const glm::dvec2& b = poly[(i + 1) % count];
                        if ((a.y <= scanY && b.y > scanY) || (b.y <= scanY && a.y > scanY))
                        {
                            double t       = (scanY - a.y) / (b.y - a.y);
                            double x       = a.x + t * (b.x - a.x);
                            int    winding = (b.y > a.y) ? 1 : -1;
                            crossings.emplace_back(x, winding);
                        }
                    }
                }

                if (crossings.empty())
                {
                    continue;
                }

                std::sort(crossings.begin(), crossings.end(), [](const std::pair<double, int>& lhs, const std::pair<double, int>& rhs) { return lhs.first < rhs.first; });

                int windingCount = 0;
                for (size_t i = 0; i + 1 < crossings.size(); i++)
                {
                    windingCount += crossings[i].second;
                    bool inside = (fillRule == VectorFillRule::EvenOdd) ? ((windingCount & 1) != 0) : (windingCount != 0);
                    if (!inside)
                    {
                        continue;
                    }

                    double x0 = std::clamp(crossings[i].first, 0.0, static_cast<double>(width));
                    double x1 = std::clamp(crossings[i + 1].first, 0.0, static_cast<double>(width));
                    if (x1 <= x0)
                    {
                        continue;
                    }

                    int pxStart = std::max(0, static_cast<int>(std::floor(x0)));
                    int pxEnd   = std::min(static_cast<int>(width) - 1, static_cast<int>(std::floor(x1)));
                    for (int px = pxStart; px <= pxEnd; px++)
                    {
                        double overlapStart = std::max(x0, static_cast<double>(px));
                        double overlapEnd   = std::min(x1, static_cast<double>(px + 1));
                        if (overlapEnd > overlapStart)
                        {
                            coverageRow[static_cast<size_t>(px)] += (overlapEnd - overlapStart) / static_cast<double>(subSamples);
                        }
                    }
                }
            }

            for (uint32_t px = 0; px < width; px++)
            {
                double cov = coverageRow[px];
                if (cov > 1e-4)
                {
                    blendPixel(target, px, static_cast<uint32_t>(py), color, std::min(1.0, cov));
                }
            }
        }
    }

    void VectorGraphicsRasterizer::rasterizeStroke(const std::vector<std::vector<glm::dvec2>>& polylines,
                                                   const std::vector<bool>&                    closedFlags,
                                                   double                                      strokeWidth,
                                                   const glm::u8vec4&                          color,
                                                   CADacomb::Texture::Texture&                 target)
    {
        uint32_t width  = target.getWidth();
        uint32_t height = target.getHeight();
        double   halfW  = strokeWidth * 0.5;
        double   outerR = halfW + 0.5;
        double   innerR = std::max(0.0, halfW - 0.5);

        for (size_t pIdx = 0; pIdx < polylines.size(); pIdx++)
        {
            const std::vector<glm::dvec2>& poly     = polylines[pIdx];
            size_t                         segCount = closedFlags[pIdx] ? poly.size() : (poly.size() - 1);

            for (size_t i = 0; i < segCount; i++)
            {
                const glm::dvec2& a = poly[i];
                const glm::dvec2& b = poly[(i + 1) % poly.size()];

                int minX = std::max(0, static_cast<int>(std::floor(std::min(a.x, b.x) - outerR)));
                int maxX = std::min(static_cast<int>(width) - 1, static_cast<int>(std::ceil(std::max(a.x, b.x) + outerR)));
                int minY = std::max(0, static_cast<int>(std::floor(std::min(a.y, b.y) - outerR)));
                int maxY = std::min(static_cast<int>(height) - 1, static_cast<int>(std::ceil(std::max(a.y, b.y) + outerR)));

                glm::dvec2 ab     = b - a;
                double     abLen2 = glm::dot(ab, ab);

                for (int py = minY; py <= maxY; py++)
                {
                    for (int px = minX; px <= maxX; px++)
                    {
                        glm::dvec2 p(static_cast<double>(px) + 0.5, static_cast<double>(py) + 0.5);
                        double     t       = (abLen2 > 1e-12) ? std::clamp(glm::dot(p - a, ab) / abLen2, 0.0, 1.0) : 0.0;
                        glm::dvec2 closest = a + t * ab;
                        double     dist    = glm::length(p - closest);
                        if (dist <= outerR)
                        {
                            double cov = (dist <= innerR) ? 1.0 : std::clamp((outerR - dist) / std::max(outerR - innerR, 1e-6), 0.0, 1.0);
                            blendPixel(target, static_cast<uint32_t>(px), static_cast<uint32_t>(py), color, cov);
                        }
                    }
                }
            }
        }
    }

    void VectorGraphicsRasterizer::blendPixel(CADacomb::Texture::Texture& target, uint32_t x, uint32_t y, const glm::u8vec4& color, double coverage)
    {
        double srcA = (static_cast<double>(color.a) / 255.0) * coverage;
        if (srcA <= 0.0)
        {
            return;
        }

        glm::u8vec4 dst  = target.getPixel(x, y);
        double      dstA = static_cast<double>(dst.a) / 255.0;
        double      outA = srcA + dstA * (1.0 - srcA);
        if (outA <= 1e-6)
        {
            return;
        }

        double outR = (static_cast<double>(color.r) * srcA + static_cast<double>(dst.r) * dstA * (1.0 - srcA)) / outA;
        double outG = (static_cast<double>(color.g) * srcA + static_cast<double>(dst.g) * dstA * (1.0 - srcA)) / outA;
        double outB = (static_cast<double>(color.b) * srcA + static_cast<double>(dst.b) * dstA * (1.0 - srcA)) / outA;

        target.setPixel(x,
                        y,
                        glm::u8vec4(static_cast<uint8_t>(std::clamp(std::round(outR), 0.0, 255.0)),
                                    static_cast<uint8_t>(std::clamp(std::round(outG), 0.0, 255.0)),
                                    static_cast<uint8_t>(std::clamp(std::round(outB), 0.0, 255.0)),
                                    static_cast<uint8_t>(std::clamp(std::round(outA * 255.0), 0.0, 255.0))));
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/VectorGraphics/Svg/SvgReader.h"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("VectorGraphicsRasterizer/EvenOddHole")
{
    std::string svg = R"(<svg viewBox="0 0 100 100">
  <path d="M 10 10 L 90 10 L 90 90 L 10 90 Z M 35 35 L 65 35 L 65 65 L 35 65 Z" fill="#ff0000" fill-rule="evenodd"/>
</svg>)";

    CADacomb::VectorGraphics::VectorGraphics doc = CADacomb::VectorGraphics::SvgReader::loadFromString(svg);
    CADacomb::Texture::Texture               tex = CADacomb::VectorGraphics::VectorGraphicsRasterizer::rasterize(doc, 100, 100, glm::u8vec4(255, 255, 255, 255));

    CHECK(tex.getPixel(20, 20) == glm::u8vec4(255, 0, 0, 255));
    CHECK(tex.getPixel(50, 50) == glm::u8vec4(255, 255, 255, 255));
    CHECK(tex.getPixel(2, 2) == glm::u8vec4(255, 255, 255, 255));
}
#endif
