#include "CADacomb/Physics/Jolt/BalloonSoftBodyBuilderJ.h"

#include <algorithm>
#include <array>

namespace CADacomb::Physics
{
    BalloonSoftBodyBuilderJ::BalloonSoftBodyBuilderJ() {}

    BalloonSoftBodyBuilderJ::~BalloonSoftBodyBuilderJ() {}

    JPH::Ref<JPH::SoftBodySharedSettings> BalloonSoftBodyBuilderJ::build(const Triangulation& triangulation,
                                                                         const glm::dmat4&    localScale,
                                                                         double               mass,
                                                                         double               edgeCompliance,
                                                                         double               bendCompliance,
                                                                         double,
                                                                         int,
                                                                         std::vector<size_t>& originalToWeldedOut) const
    {
        std::vector<glm::dvec3> weldedPositions;
        weldVertices(triangulation, localScale, weldedPositions, originalToWeldedOut);

        JPH::Ref<JPH::SoftBodySharedSettings> sharedSettings = new JPH::SoftBodySharedSettings();
        sharedSettings->mVertices.reserve(weldedPositions.size());
        for (const auto& wp : weldedPositions)
        {
            sharedSettings->mVertices.push_back(
              JPH::SoftBodySharedSettings::Vertex(JPH::Float3(static_cast<float>(wp.x), static_cast<float>(wp.y), static_cast<float>(wp.z)), JPH::Float3(0.0f, 0.0f, 0.0f), 1.0f));
        }

        std::vector<std::array<JPH::uint32, 3>> orientedFaces;
        addWeldedFaces(triangulation, weldedPositions, originalToWeldedOut, sharedSettings, orientedFaces);
        assignUniformParticleMass(sharedSettings, mass);

        JPH::SoftBodySharedSettings::VertexAttributes vertexAttributes(
          static_cast<float>(std::max(0.0, edgeCompliance)), static_cast<float>(std::max(0.0, edgeCompliance)), static_cast<float>(std::max(0.0, bendCompliance)));
        sharedSettings->CreateConstraints(&vertexAttributes, 1, JPH::SoftBodySharedSettings::EBendType::Distance);
        sharedSettings->Optimize();
        return sharedSettings;
    }

    float BalloonSoftBodyBuilderJ::resolvePressure(double requestedPressure) const
    {
        return static_cast<float>(std::max(0.0, requestedPressure));
    }
}
