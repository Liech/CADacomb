#pragma once

#include "CADacomb/Texture/TextureColorSpace.h"
#include "CADacomb/Texture/TextureFilterMode.h"
#include "CADacomb/Texture/TextureWrapMode.h"

#include <cstddef>
#include <cstdint>
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace CADacomb::Texture
{
    // Headless RGBA8 2D texture with stb image I/O (PNG, JPG, TGA, BMP) and optional 2D color grid conversion.
    class Texture
    {
      public:
        Texture();
        Texture(uint32_t width, uint32_t height, const glm::u8vec4& fillColor);
        Texture(uint32_t width, uint32_t height, const std::vector<uint8_t>& rgbaPixels);
        virtual ~Texture();

        uint32_t                    getWidth() const;
        uint32_t                    getHeight() const;
        bool                        isEmpty() const;
        size_t                      getByteSize() const;
        const std::vector<uint8_t>& getPixels() const;
        std::vector<uint8_t>&       getPixels();

        glm::u8vec4 getPixel(uint32_t x, uint32_t y) const;
        void        setPixel(uint32_t x, uint32_t y, const glm::u8vec4& color);

        std::vector<std::vector<glm::u8vec4>> toColorGrid() const;
        static Texture                        fromColorGrid(const std::vector<std::vector<glm::u8vec4>>& grid);

        TextureColorSpace getColorSpace() const;
        void              setColorSpace(TextureColorSpace colorSpace);

        TextureFilterMode getFilterMode() const;
        void              setFilterMode(TextureFilterMode filterMode);

        TextureWrapMode getWrapMode() const;
        void            setWrapMode(TextureWrapMode wrapMode);

        void                 saveToPngFile(const std::string& filename) const;
        std::vector<uint8_t> encodeToPngMemory() const;
        static Texture       loadFromFile(const std::string& filename);
        static Texture       loadFromMemory(const std::vector<uint8_t>& encodedBytes);
        static Texture       loadFromPngFile(const std::string& filename);
        static Texture       loadFromPngMemory(const std::vector<uint8_t>& encodedBytes);

      private:
        static void writePngCallback(void* context, void* data, int size);

        uint32_t             width  = 0;
        uint32_t             height = 0;
        std::vector<uint8_t> pixels;
        TextureColorSpace    colorSpace = TextureColorSpace::SRGB;
        TextureFilterMode    filterMode = TextureFilterMode::Linear;
        TextureWrapMode      wrapMode   = TextureWrapMode::Repeat;
    };
}
