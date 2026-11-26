#include "CADacombRender/Texture/TextureImageConverter.h"

#include "Vulcant/Interface/VulcantDevice.h"
#include "Vulcant/Interface/VulcantImage.h"
#include "Vulcant/Interface/VulcantImageFormat.h"

namespace CADacombRender
{
    TextureImageConverter::TextureImageConverter() {}

    TextureImageConverter::~TextureImageConverter() {}

    std::unique_ptr<Vulcant::VulcantImage> TextureImageConverter::toVulcantImage(Vulcant::VulcantDevice& device, const CADacomb::Texture::Texture& texture, Vulcant::VulcantImageUsage usage)
    {
        std::unique_ptr<Vulcant::VulcantImage> image = device.createImage(texture.getWidth(), texture.getHeight(), 1, Vulcant::VulcantImageFormat::R8G8B8A8_UNORM);
        image->setUsage(usage);
        image->uploadPixelsRgba8(texture.getPixels());
        return image;
    }

    CADacomb::Texture::Texture TextureImageConverter::fromVulcantImage(Vulcant::VulcantImage& image)
    {
        glm::ivec2           resolution = image.getResolution();
        std::vector<uint8_t> pixels     = image.readPixelsRgba8();
        return CADacomb::Texture::Texture(static_cast<uint32_t>(resolution.x), static_cast<uint32_t>(resolution.y), pixels);
    }
}

#ifdef ISTESTPROJECT
#include "Vulcant/VulcantV/VulcantVDevice.h"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("TextureImageConverter/RoundTripGpuImage")
{
    Vulcant::VulcantV::VulcantVDevice device({}, false);

    CADacomb::Texture::Texture source(4, 4, glm::u8vec4(10, 20, 30, 255));
    source.setPixel(2, 1, glm::u8vec4(240, 120, 60, 255));

    std::unique_ptr<Vulcant::VulcantImage> gpuImage = CADacombRender::TextureImageConverter::toVulcantImage(device, source, Vulcant::VulcantImageUsage::SampleOnly);
    REQUIRE(gpuImage != nullptr);

    CADacomb::Texture::Texture readback = CADacombRender::TextureImageConverter::fromVulcantImage(*gpuImage);
    CHECK(readback.getWidth() == 4);
    CHECK(readback.getHeight() == 4);
    CHECK(readback.getPixel(0, 0) == glm::u8vec4(10, 20, 30, 255));
    CHECK(readback.getPixel(2, 1) == glm::u8vec4(240, 120, 60, 255));
}
#endif
