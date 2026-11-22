#pragma once

#include "CADacomb/Physics/Jolt/SoftBodyBuilderJ.h"

namespace CADacomb::Physics
{
    // Builds a solid-feeling soft body using dihedral surface bending, an internal core particle, radial struts, and core-linked volume tetrahedra.
    class InternalStrutsSoftBodyBuilderJ : public SoftBodyBuilderJ
    {
      public:
        InternalStrutsSoftBodyBuilderJ();
        virtual ~InternalStrutsSoftBodyBuilderJ();

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
