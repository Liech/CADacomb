#include "CADacomb/Texture/Texture.h"

#include <lodepng.h>
#include <stdexcept>

namespace CADacomb::Texture
{
    Texture::Texture() {}

    Texture::Texture(uint32_t widthInput, uint32_t heightInput, const glm::u8vec4& fillColor)
      : width(widthInput)
      , height(heightInput)
      , pixels(static_cast<size_t>(widthInput) * static_cast<size_t>(heightInput) * 4, 0)
    {
        for (size_t i = 0; i < pixels.size(); i += 4)
        {
            pixels[i]     = fillColor.r;
            pixels[i + 1] = fillColor.g;
            pixels[i + 2] = fillColor.b;
            pixels[i + 3] = fillColor.a;
        }
    }

    Texture::Texture(uint32_t widthInput, uint32_t heightInput, const std::vector<uint8_t>& rgbaPixels)
      : width(widthInput)
      , height(heightInput)
      , pixels(rgbaPixels)
    {
        size_t expectedSize = static_cast<size_t>(width) * static_cast<size_t>(height) * 4;
        if (pixels.size() != expectedSize)
        {
            throw std::invalid_argument("RGBA pixel buffer size does not match width * height * 4");
        }
    }

    Texture::~Texture() {}

    uint32_t Texture::getWidth() const
    {
        return width;
    }

    uint32_t Texture::getHeight() const
    {
        return height;
    }

    bool Texture::isEmpty() const
    {
        return width == 0 || height == 0 || pixels.empty();
    }

    size_t Texture::getByteSize() const
    {
        return pixels.size();
    }

    const std::vector<uint8_t>& Texture::getPixels() const
    {
        return pixels;
    }

    std::vector<uint8_t>& Texture::getPixels()
    {
        return pixels;
    }

    glm::u8vec4 Texture::getPixel(uint32_t x, uint32_t y) const
    {
        size_t offset = (static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 4;
        return glm::u8vec4(pixels[offset], pixels[offset + 1], pixels[offset + 2], pixels[offset + 3]);
    }

    void Texture::setPixel(uint32_t x, uint32_t y, const glm::u8vec4& color)
    {
        size_t offset      = (static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 4;
        pixels[offset]     = color.r;
        pixels[offset + 1] = color.g;
        pixels[offset + 2] = color.b;
        pixels[offset + 3] = color.a;
    }

    std::vector<std::vector<glm::u8vec4>> Texture::toColorGrid() const
    {
        std::vector<std::vector<glm::u8vec4>> grid(height, std::vector<glm::u8vec4>(width, glm::u8vec4(0, 0, 0, 0)));
        for (uint32_t y = 0; y < height; y++)
        {
            for (uint32_t x = 0; x < width; x++)
            {
                grid[y][x] = getPixel(x, y);
            }
        }
        return grid;
    }

    Texture Texture::fromColorGrid(const std::vector<std::vector<glm::u8vec4>>& grid)
    {
        if (grid.empty() || grid[0].empty())
        {
            return Texture();
        }

        uint32_t             gridHeight = static_cast<uint32_t>(grid.size());
        uint32_t             gridWidth  = static_cast<uint32_t>(grid[0].size());
        std::vector<uint8_t> rgbaPixels(static_cast<size_t>(gridWidth) * static_cast<size_t>(gridHeight) * 4, 0);

        for (uint32_t y = 0; y < gridHeight; y++)
        {
            if (grid[y].size() != gridWidth)
            {
                throw std::invalid_argument("Color grid rows must have uniform width");
            }
            for (uint32_t x = 0; x < gridWidth; x++)
            {
                size_t             offset = (static_cast<size_t>(y) * static_cast<size_t>(gridWidth) + static_cast<size_t>(x)) * 4;
                const glm::u8vec4& color  = grid[y][x];
                rgbaPixels[offset]        = color.r;
                rgbaPixels[offset + 1]    = color.g;
                rgbaPixels[offset + 2]    = color.b;
                rgbaPixels[offset + 3]    = color.a;
            }
        }

        return Texture(gridWidth, gridHeight, rgbaPixels);
    }

    TextureColorSpace Texture::getColorSpace() const
    {
        return colorSpace;
    }

    void Texture::setColorSpace(TextureColorSpace colorSpaceInput)
    {
        colorSpace = colorSpaceInput;
    }

    TextureFilterMode Texture::getFilterMode() const
    {
        return filterMode;
    }

    void Texture::setFilterMode(TextureFilterMode filterModeInput)
    {
        filterMode = filterModeInput;
    }

    TextureWrapMode Texture::getWrapMode() const
    {
        return wrapMode;
    }

    void Texture::setWrapMode(TextureWrapMode wrapModeInput)
    {
        wrapMode = wrapModeInput;
    }

    void Texture::saveToPngFile(const std::string& filename) const
    {
        unsigned int error = lodepng::encode(filename, pixels, width, height);
        if (error != 0)
        {
            throw std::runtime_error(std::string("Failed to encode PNG file: ") + lodepng_error_text(error));
        }
    }

    std::vector<uint8_t> Texture::encodeToPngMemory() const
    {
        std::vector<uint8_t> encoded;
        unsigned int         error = lodepng::encode(encoded, pixels, width, height);
        if (error != 0)
        {
            throw std::runtime_error(std::string("Failed to encode PNG memory: ") + lodepng_error_text(error));
        }
        return encoded;
    }

    Texture Texture::loadFromPngFile(const std::string& filename)
    {
        std::vector<uint8_t> decoded;
        unsigned int         decodedWidth  = 0;
        unsigned int         decodedHeight = 0;
        unsigned int         error         = lodepng::decode(decoded, decodedWidth, decodedHeight, filename);
        if (error != 0)
        {
            throw std::runtime_error(std::string("Failed to decode PNG file: ") + lodepng_error_text(error));
        }
        return Texture(static_cast<uint32_t>(decodedWidth), static_cast<uint32_t>(decodedHeight), decoded);
    }

    Texture Texture::loadFromPngMemory(const std::vector<uint8_t>& encodedBytes)
    {
        std::vector<uint8_t> decoded;
        unsigned int         decodedWidth  = 0;
        unsigned int         decodedHeight = 0;
        unsigned int         error         = lodepng::decode(decoded, decodedWidth, decodedHeight, encodedBytes);
        if (error != 0)
        {
            throw std::runtime_error(std::string("Failed to decode PNG memory: ") + lodepng_error_text(error));
        }
        return Texture(static_cast<uint32_t>(decodedWidth), static_cast<uint32_t>(decodedHeight), decoded);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Texture/PixelAccessAndColorGridRoundTrip")
{
    CADacomb::Texture::Texture tex(2, 2, glm::u8vec4(10, 20, 30, 255));
    tex.setPixel(1, 0, glm::u8vec4(200, 100, 50, 128));

    CHECK(tex.getWidth() == 2);
    CHECK(tex.getHeight() == 2);
    CHECK(tex.getByteSize() == 16);
    CHECK(tex.getPixel(0, 0) == glm::u8vec4(10, 20, 30, 255));
    CHECK(tex.getPixel(1, 0) == glm::u8vec4(200, 100, 50, 128));

    std::vector<std::vector<glm::u8vec4>> grid          = tex.toColorGrid();
    CADacomb::Texture::Texture            reconstructed = CADacomb::Texture::Texture::fromColorGrid(grid);
    CHECK(reconstructed.getWidth() == 2);
    CHECK(reconstructed.getHeight() == 2);
    CHECK(reconstructed.getPixel(1, 0) == glm::u8vec4(200, 100, 50, 128));
}

TEST_CASE("Texture/PngMemoryRoundTrip")
{
    CADacomb::Texture::Texture tex(3, 2, glm::u8vec4(5, 15, 25, 255));
    tex.setPixel(2, 1, glm::u8vec4(255, 0, 128, 255));

    std::vector<uint8_t>       pngBytes = tex.encodeToPngMemory();
    CADacomb::Texture::Texture decoded  = CADacomb::Texture::Texture::loadFromPngMemory(pngBytes);

    CHECK(decoded.getWidth() == 3);
    CHECK(decoded.getHeight() == 2);
    CHECK(decoded.getPixel(0, 0) == glm::u8vec4(5, 15, 25, 255));
    CHECK(decoded.getPixel(2, 1) == glm::u8vec4(255, 0, 128, 255));
}
#endif
