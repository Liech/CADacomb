#pragma once

#include <cstdint>
#include <functional>
#include <glm/glm.hpp>

namespace Library::Volume
{
    struct LandscapeConfig
    {
      public:
        LandscapeConfig();
        virtual ~LandscapeConfig();

        uint32_t                                          seed            = 1337;
        int                                               octaves         = 4;
        double                                            frequency       = 0.05;
        double                                            persistence     = 0.5;
        double                                            lacunarity      = 2.0;
        double                                            baseHeightRatio = 0.4;
        double                                            heightScale     = 1.2;
        bool                                              enableCaves     = false;
        double                                            caveFrequency   = 0.04;
        double                                            caveThreshold   = 0.12;
        double                                            solidThreshold  = 0.0;
        std::function<double(const glm::dvec3& worldPos)> customDensity   = nullptr;
    };
}
