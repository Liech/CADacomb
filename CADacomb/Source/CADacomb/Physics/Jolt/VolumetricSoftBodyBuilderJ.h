#pragma once

#include "CADacomb/Physics/Jolt/SoftBodyBuilderJ.h"

namespace CADacomb::Physics
{
    // Builds a volumetric soft body by voxelizing the mesh interior into tetrahedral volume cells and binding the surface mesh to the interior lattice.
    class VolumetricSoftBodyBuilderJ : public SoftBodyBuilderJ
    {
      public:
        VolumetricSoftBodyBuilderJ();
        virtual ~VolumetricSoftBodyBuilderJ();

        JPH::Ref<JPH::SoftBodySharedSettings> build(const Triangulation& triangulation,
                                                    const glm::dmat4&    localScale,
                                                    double               mass,
                                                    double               edgeCompliance,
                                                    double               bendCompliance,
                                                    double               volumeCompliance,
                                                    int                  internalResolution,
                                                    std::vector<size_t>& originalToWeldedOut) const override;

        float resolvePressure(double requestedPressure) const override;

      private:
        void addCellTetrahedra(JPH::Ref<JPH::SoftBodySharedSettings>& sharedSettings, const std::array<JPH::uint32, 8>& corners, bool alternateParity, float volumeCompliance) const;
    };
}
