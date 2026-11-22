#pragma once

#include "CADacomb/Physics/Jolt/SoftBodyBuilderJ.h"

namespace CADacomb::Physics
{
    // Builds a hollow surface-shell soft body inflated by internal gas pressure.
    class BalloonSoftBodyBuilderJ : public SoftBodyBuilderJ
    {
      public:
        BalloonSoftBodyBuilderJ();
        virtual ~BalloonSoftBodyBuilderJ();

        JPH::Ref<JPH::SoftBodySharedSettings> build(const Triangulation& triangulation,
                                                    const glm::dmat4&    localScale,
                                                    double               mass,
                                                    double               edgeCompliance,
                                                    double               bendCompliance,
                                                    double               volumeCompliance,
                                                    int                  internalResolution,
                                                    std::vector<size_t>& originalToWeldedOut) const override;

        float resolvePressure(double requestedPressure) const override;
    };
}
