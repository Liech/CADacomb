#pragma once

#include "CADacomb/Texture/Texture.h"
#include "Vulcant/Interface/VulcantImageUsage.h"

#include <memory>

namespace Vulcant
{
    class VulcantDevice;
    class VulcantImage;
}

namespace CADacombRender
{
    // Bidirectional converter between headless CADacomb::Texture and GPU Vulcant::VulcantImage.
    class TextureImageConverter
    {
      public:
        TextureImageConverter();
        virtual ~TextureImageConverter();

        static std::unique_ptr<Vulcant::VulcantImage> toVulcantImage(Vulcant::VulcantDevice& device, const CADacomb::Texture::Texture& texture, Vulcant::VulcantImageUsage usage);
        static CADacomb::Texture::Texture             fromVulcantImage(Vulcant::VulcantImage& image);
    };
}
