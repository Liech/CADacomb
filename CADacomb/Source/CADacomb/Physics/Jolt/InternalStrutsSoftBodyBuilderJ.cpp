#include "CADacomb/Physics/Jolt/InternalStrutsSoftBodyBuilderJ.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <utility>

namespace CADacomb::Physics
{
    InternalStrutsSoftBodyBuilderJ::InternalStrutsSoftBodyBuilderJ() {}

    InternalStrutsSoftBodyBuilderJ::~InternalStrutsSoftBodyBuilderJ() {}

    JPH::Ref<JPH::SoftBodySharedSettings> InternalStrutsSoftBodyBuilderJ::build(const Triangulation& triangulation,
                                                                                const glm::dmat4&    localScale,
                                                                                double               mass,
                                                                                double               edgeCompliance,
                                                                                double               bendCompliance,
                                                                                double               volumeCompliance,
                                                                                int,
                                                                                std::vector<size_t>& originalToWeldedOut) const
    {
        std::vector<glm::dvec3> weldedPositions;
        weldVertices(triangulation, localScale, weldedPositions, originalToWeldedOut);

        JPH::Ref<JPH::SoftBodySharedSettings> sharedSettings = new JPH::SoftBodySharedSettings();
        sharedSettings->mVertices.reserve(weldedPositions.size() + 1);

        glm::dvec3 centroid(0.0);
        for (const auto& wp : weldedPositions)
        {
            centroid += wp;
            sharedSettings->mVertices.push_back(
              JPH::SoftBodySharedSettings::Vertex(JPH::Float3(static_cast<float>(wp.x), static_cast<float>(wp.y), static_cast<float>(wp.z)), JPH::Float3(0.0f, 0.0f, 0.0f), 1.0f));
        }

        std::vector<std::array<JPH::uint32, 3>> orientedFaces;
        addWeldedFaces(triangulation, weldedPositions, originalToWeldedOut, sharedSettings, orientedFaces);

        const float clampedEdgeCompliance = static_cast<float>(std::max(1e-5, edgeCompliance));
        const float clampedBendCompliance = static_cast<float>(std::max(1e-5, bendCompliance));
        const float clampedVolCompliance  = static_cast<float>(std::max(1e-4, volumeCompliance));

        JPH::SoftBodySharedSettings::VertexAttributes vertexAttributes(clampedEdgeCompliance, clampedEdgeCompliance, clampedBendCompliance);
        sharedSettings->CreateConstraints(&vertexAttributes, 1, JPH::SoftBodySharedSettings::EBendType::Distance);

        const size_t surfaceCount = weldedPositions.size();
        if (surfaceCount >= 4)
        {
            centroid /= static_cast<double>(surfaceCount);
            const JPH::uint32 coreIdx = static_cast<JPH::uint32>(sharedSettings->mVertices.size());
            sharedSettings->mVertices.push_back(
              JPH::SoftBodySharedSettings::Vertex(JPH::Float3(static_cast<float>(centroid.x), static_cast<float>(centroid.y), static_cast<float>(centroid.z)), JPH::Float3(0.0f, 0.0f, 0.0f), 1.0f));

            std::set<std::pair<JPH::uint32, JPH::uint32>> addedEdges;
            for (const auto& edge : sharedSettings->mEdgeConstraints)
            {
                JPH::uint32 a = std::min(edge.mVertex[0], edge.mVertex[1]);
                JPH::uint32 b = std::max(edge.mVertex[0], edge.mVertex[1]);
                addedEdges.insert(std::make_pair(a, b));
            }

            auto addUniqueStrut = [&](JPH::uint32 a, JPH::uint32 b, float compliance)
            {
                if (a == b)
                {
                    return;
                }
                std::pair<JPH::uint32, JPH::uint32> key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
                if (addedEdges.insert(key).second)
                {
                    sharedSettings->mEdgeConstraints.push_back(JPH::SoftBodySharedSettings::Edge(key.first, key.second, compliance));
                }
            };

            for (size_t i = 0; i < surfaceCount; i++)
            {
                if (glm::distance(weldedPositions[i], centroid) > 1e-4)
                {
                    addUniqueStrut(static_cast<JPH::uint32>(i), coreIdx, clampedVolCompliance);
                }

                glm::dvec3 radial = weldedPositions[i] - centroid;
                double     rLen   = glm::length(radial);
                if (rLen > 1e-4)
                {
                    glm::dvec3 targetDir = -(radial / rLen);
                    size_t     bestIdx   = i;
                    double     bestScore = -1e30;
                    for (size_t j = 0; j < surfaceCount; j++)
                    {
                        if (j == i)
                        {
                            continue;
                        }
                        double score = glm::dot(weldedPositions[j] - centroid, targetDir);
                        if (score > bestScore)
                        {
                            bestScore = score;
                            bestIdx   = j;
                        }
                    }
                    if (bestIdx != i && glm::distance(weldedPositions[i], weldedPositions[bestIdx]) > 1e-4)
                    {
                        addUniqueStrut(static_cast<JPH::uint32>(i), static_cast<JPH::uint32>(bestIdx), clampedVolCompliance);
                    }
                }
            }

            sharedSettings->CalculateEdgeLengths();

            const float totalMass   = std::max(1e-4f, static_cast<float>(mass));
            const float coreMass    = totalMass * 0.5f;
            const float surfaceMass = totalMass * 0.5f;
            const float surfInvMass = static_cast<float>(surfaceCount) / surfaceMass;
            for (size_t i = 0; i < surfaceCount; i++)
            {
                sharedSettings->mVertices[i].mInvMass = surfInvMass;
            }
            sharedSettings->mVertices[coreIdx].mInvMass = 1.0f / coreMass;
        }
        else
        {
            assignUniformParticleMass(sharedSettings, mass);
        }

        sharedSettings->Optimize();
        return sharedSettings;
    }

    float InternalStrutsSoftBodyBuilderJ::resolvePressure(double) const
    {
        return 0.0f;
    }
}
