#include "CADacomb/Physics/Jolt/SoftBodyBuilderJ.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <tuple>

namespace CADacomb::Physics
{
    SoftBodyBuilderJ::SoftBodyBuilderJ() {}

    SoftBodyBuilderJ::~SoftBodyBuilderJ() {}

    void SoftBodyBuilderJ::weldVertices(const Triangulation& triangulation, const glm::dmat4& localScale, std::vector<glm::dvec3>& weldedPositionsOut, std::vector<size_t>& originalToWeldedOut) const
    {
        weldedPositionsOut.clear();
        originalToWeldedOut.assign(triangulation.vertices.size(), 0);
        std::map<std::tuple<int64_t, int64_t, int64_t>, size_t> weldMap;
        const double                                            weldScale = 100000.0;

        for (size_t i = 0; i < triangulation.vertices.size(); i++)
        {
            glm::dvec3 sv = glm::dvec3(localScale * glm::dvec4(triangulation.vertices[i], 1.0));
            auto       key =
              std::make_tuple(static_cast<int64_t>(std::llround(sv.x * weldScale)), static_cast<int64_t>(std::llround(sv.y * weldScale)), static_cast<int64_t>(std::llround(sv.z * weldScale)));
            auto it = weldMap.find(key);
            if (it != weldMap.end())
            {
                originalToWeldedOut[i] = it->second;
            }
            else
            {
                size_t newIdx = weldedPositionsOut.size();
                weldedPositionsOut.push_back(sv);
                weldMap[key]           = newIdx;
                originalToWeldedOut[i] = newIdx;
            }
        }
    }

    void SoftBodyBuilderJ::addWeldedFaces(const Triangulation&                     triangulation,
                                          const std::vector<glm::dvec3>&           weldedPositions,
                                          const std::vector<size_t>&               originalToWelded,
                                          JPH::Ref<JPH::SoftBodySharedSettings>&   sharedSettings,
                                          std::vector<std::array<JPH::uint32, 3>>& orientedFacesOut) const
    {
        orientedFacesOut.clear();
        glm::dvec3 centroid(0.0);
        if (!weldedPositions.empty())
        {
            for (const auto& wp : weldedPositions)
            {
                centroid += wp;
            }
            centroid /= static_cast<double>(weldedPositions.size());
        }

        double signedVolumeSix = 0.0;
        for (size_t i = 0; i + 2 < triangulation.indices.size(); i += 3)
        {
            size_t orig0 = triangulation.indices[i];
            size_t orig1 = triangulation.indices[i + 1];
            size_t orig2 = triangulation.indices[i + 2];
            if (orig0 < originalToWelded.size() && orig1 < originalToWelded.size() && orig2 < originalToWelded.size())
            {
                size_t w0 = originalToWelded[orig0];
                size_t w1 = originalToWelded[orig1];
                size_t w2 = originalToWelded[orig2];
                if (w0 != w1 && w1 != w2 && w2 != w0)
                {
                    glm::dvec3 p0 = weldedPositions[w0] - centroid;
                    glm::dvec3 p1 = weldedPositions[w1] - centroid;
                    glm::dvec3 p2 = weldedPositions[w2] - centroid;
                    signedVolumeSix += glm::dot(glm::cross(p1 - p0, p2 - p0), p0);
                }
            }
        }

        const bool flipWinding = signedVolumeSix < 0.0;
        for (size_t i = 0; i + 2 < triangulation.indices.size(); i += 3)
        {
            size_t orig0 = triangulation.indices[i];
            size_t orig1 = triangulation.indices[i + 1];
            size_t orig2 = triangulation.indices[i + 2];
            if (orig0 < originalToWelded.size() && orig1 < originalToWelded.size() && orig2 < originalToWelded.size())
            {
                JPH::uint32 w0 = static_cast<JPH::uint32>(originalToWelded[orig0]);
                JPH::uint32 w1 = static_cast<JPH::uint32>(originalToWelded[orig1]);
                JPH::uint32 w2 = static_cast<JPH::uint32>(originalToWelded[orig2]);
                if (w0 != w1 && w1 != w2 && w2 != w0)
                {
                    if (flipWinding)
                    {
                        sharedSettings->AddFace(JPH::SoftBodySharedSettings::Face(w0, w2, w1));
                        orientedFacesOut.push_back({ w0, w2, w1 });
                    }
                    else
                    {
                        sharedSettings->AddFace(JPH::SoftBodySharedSettings::Face(w0, w1, w2));
                        orientedFacesOut.push_back({ w0, w1, w2 });
                    }
                }
            }
        }
    }

    void SoftBodyBuilderJ::assignUniformParticleMass(JPH::Ref<JPH::SoftBodySharedSettings>& sharedSettings, double totalMass) const
    {
        const float particleCount   = std::max(1.0f, static_cast<float>(sharedSettings->mVertices.size()));
        const float clampedMass     = std::max(1e-4f, static_cast<float>(totalMass));
        const float invParticleMass = particleCount / clampedMass;
        for (auto& vertex : sharedSettings->mVertices)
        {
            vertex.mInvMass = invParticleMass;
        }
    }
}
