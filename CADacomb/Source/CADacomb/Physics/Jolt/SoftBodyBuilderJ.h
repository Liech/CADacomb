#pragma once

#include "CADacomb/Triangle/Triangulation.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <glm/glm.hpp>
#include <vector>

namespace CADacomb::Physics
{
    // Abstract base class for Jolt soft body constraint builders.
    class SoftBodyBuilderJ
    {
      public:
        SoftBodyBuilderJ();
        virtual ~SoftBodyBuilderJ();

        virtual JPH::Ref<JPH::SoftBodySharedSettings> build(const Triangulation& triangulation,
                                                            const glm::dmat4&    localScale,
                                                            double               mass,
                                                            double               edgeCompliance,
                                                            double               bendCompliance,
                                                            double               volumeCompliance,
                                                            int                  internalResolution,
                                                            std::vector<size_t>& originalToWeldedOut) const = 0;

        virtual float resolvePressure(double requestedPressure) const = 0;

      protected:
        void weldVertices(const Triangulation& triangulation, const glm::dmat4& localScale, std::vector<glm::dvec3>& weldedPositionsOut, std::vector<size_t>& originalToWeldedOut) const;

        void addWeldedFaces(const Triangulation&                     triangulation,
                            const std::vector<glm::dvec3>&           weldedPositions,
                            const std::vector<size_t>&               originalToWelded,
                            JPH::Ref<JPH::SoftBodySharedSettings>&   sharedSettings,
                            std::vector<std::array<JPH::uint32, 3>>& orientedFacesOut) const;

        void assignUniformParticleMass(JPH::Ref<JPH::SoftBodySharedSettings>& sharedSettings, double totalMass) const;
    };
}
