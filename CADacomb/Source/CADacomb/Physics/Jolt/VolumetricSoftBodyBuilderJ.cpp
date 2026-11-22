#include "CADacomb/Physics/Jolt/VolumetricSoftBodyBuilderJ.h"

#include "CADacomb/Volume/Flat/Voxelizer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>

namespace CADacomb::Physics
{
    VolumetricSoftBodyBuilderJ::VolumetricSoftBodyBuilderJ() {}

    VolumetricSoftBodyBuilderJ::~VolumetricSoftBodyBuilderJ() {}

    JPH::Ref<JPH::SoftBodySharedSettings> VolumetricSoftBodyBuilderJ::build(const Triangulation& triangulation,
                                                                            const glm::dmat4&    localScale,
                                                                            double               mass,
                                                                            double               edgeCompliance,
                                                                            double               bendCompliance,
                                                                            double               volumeCompliance,
                                                                            int                  internalResolution,
                                                                            std::vector<size_t>& originalToWeldedOut) const
    {
        std::vector<glm::dvec3> weldedPositions;
        weldVertices(triangulation, localScale, weldedPositions, originalToWeldedOut);

        JPH::Ref<JPH::SoftBodySharedSettings> sharedSettings = new JPH::SoftBodySharedSettings();
        sharedSettings->mVertices.reserve(weldedPositions.size());

        glm::dvec3 minBounds(std::numeric_limits<double>::max());
        glm::dvec3 maxBounds(std::numeric_limits<double>::lowest());
        for (const auto& wp : weldedPositions)
        {
            minBounds = glm::min(minBounds, wp);
            maxBounds = glm::max(maxBounds, wp);
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
        if (surfaceCount >= 4 && !orientedFaces.empty())
        {
            glm::dvec3 span    = maxBounds - minBounds;
            double     maxSpan = std::max({ span.x, span.y, span.z, 1e-4 });
            int        maxRes  = std::clamp(internalResolution, 2, 16);

            glm::u64vec3 gridRes(std::max<uint64_t>(2, static_cast<uint64_t>(std::round((span.x / maxSpan) * static_cast<double>(maxRes)))),
                                 std::max<uint64_t>(2, static_cast<uint64_t>(std::round((span.y / maxSpan) * static_cast<double>(maxRes)))),
                                 std::max<uint64_t>(2, static_cast<uint64_t>(std::round((span.z / maxSpan) * static_cast<double>(maxRes)))));

            glm::u64vec3 voxRes(gridRes.x, gridRes.y, ((gridRes.z + 7ULL) / 8ULL) * 8ULL);

            glm::dvec3 padding   = glm::max(span * 0.02, glm::dvec3(1e-4));
            glm::dvec3 gridStart = minBounds - padding;
            glm::dvec3 gridEnd   = maxBounds + padding;
            glm::dvec3 cellSize  = (gridEnd - gridStart) / glm::dvec3(static_cast<double>(gridRes.x), static_cast<double>(gridRes.y), static_cast<double>(gridRes.z));

            std::vector<size_t> weldedIndices;
            weldedIndices.reserve(orientedFaces.size() * 3);
            for (const auto& face : orientedFaces)
            {
                weldedIndices.push_back(static_cast<size_t>(face[0]));
                weldedIndices.push_back(static_cast<size_t>(face[1]));
                weldedIndices.push_back(static_cast<size_t>(face[2]));
            }

            Voxelizer                     voxelizer;
            std::unique_ptr<BinaryVolume> volume = voxelizer.voxelize(weldedPositions, weldedIndices, gridStart, gridEnd, voxRes);

            std::vector<glm::u64vec3> occupiedCells;
            if (volume && volume->data.size() == static_cast<size_t>(voxRes.x * voxRes.y * voxRes.z))
            {
                for (uint64_t x = 0; x < gridRes.x; x++)
                {
                    for (uint64_t y = 0; y < gridRes.y; y++)
                    {
                        for (uint64_t z = 0; z < gridRes.z; z++)
                        {
                            uint64_t sampleZ =
                              std::min<uint64_t>(voxRes.z - 1ULL, static_cast<uint64_t>(((static_cast<double>(z) + 0.5) / static_cast<double>(gridRes.z)) * static_cast<double>(voxRes.z)));
                            size_t stdIdx = static_cast<size_t>(x + y * voxRes.x + sampleZ * voxRes.x * voxRes.y);
                            if (volume->data[stdIdx])
                            {
                                occupiedCells.push_back(glm::u64vec3(x, y, z));
                            }
                        }
                    }
                }
            }

            if (occupiedCells.empty())
            {
                occupiedCells.push_back(glm::u64vec3(gridRes.x / 2, gridRes.y / 2, gridRes.z / 2));
            }

            std::map<std::tuple<uint64_t, uint64_t, uint64_t>, JPH::uint32> cornerMap;
            std::vector<glm::dvec3>                                         interiorPositions;
            std::vector<JPH::uint32>                                        interiorIndices;
            std::set<std::pair<JPH::uint32, JPH::uint32>>                   addedEdges;
            for (const auto& edge : sharedSettings->mEdgeConstraints)
            {
                JPH::uint32 a = std::min(edge.mVertex[0], edge.mVertex[1]);
                JPH::uint32 b = std::max(edge.mVertex[0], edge.mVertex[1]);
                addedEdges.insert(std::make_pair(a, b));
            }

            auto getOrCreateCorner = [&](uint64_t cx, uint64_t cy, uint64_t cz) -> JPH::uint32
            {
                auto key = std::make_tuple(cx, cy, cz);
                auto it  = cornerMap.find(key);
                if (it != cornerMap.end())
                {
                    return it->second;
                }
                glm::dvec3  pos    = gridStart + glm::dvec3(static_cast<double>(cx), static_cast<double>(cy), static_cast<double>(cz)) * cellSize;
                JPH::uint32 newIdx = static_cast<JPH::uint32>(sharedSettings->mVertices.size());
                sharedSettings->mVertices.push_back(
                  JPH::SoftBodySharedSettings::Vertex(JPH::Float3(static_cast<float>(pos.x), static_cast<float>(pos.y), static_cast<float>(pos.z)), JPH::Float3(0.0f, 0.0f, 0.0f), 1.0f));
                cornerMap[key] = newIdx;
                interiorPositions.push_back(pos);
                interiorIndices.push_back(newIdx);
                return newIdx;
            };

            auto addUniqueEdge = [&](JPH::uint32 a, JPH::uint32 b, float compliance)
            {
                if (a == b)
                {
                    return;
                }
                std::pair<JPH::uint32, JPH::uint32> edgeKey = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
                if (addedEdges.insert(edgeKey).second)
                {
                    sharedSettings->mEdgeConstraints.push_back(JPH::SoftBodySharedSettings::Edge(edgeKey.first, edgeKey.second, compliance));
                }
            };

            const std::array<std::pair<int, int>, 18> cellStrutPairs = { std::make_pair(0, 1), std::make_pair(2, 3), std::make_pair(4, 5), std::make_pair(6, 7), std::make_pair(0, 2),
                                                                         std::make_pair(1, 3), std::make_pair(4, 6), std::make_pair(5, 7), std::make_pair(0, 4), std::make_pair(1, 5),
                                                                         std::make_pair(2, 6), std::make_pair(3, 7), std::make_pair(0, 3), std::make_pair(4, 7), std::make_pair(0, 5),
                                                                         std::make_pair(2, 7), std::make_pair(0, 6), std::make_pair(1, 7) };

            for (const auto& cell : occupiedCells)
            {
                std::array<JPH::uint32, 8> corners = { getOrCreateCorner(cell.x, cell.y, cell.z),         getOrCreateCorner(cell.x + 1, cell.y, cell.z),
                                                       getOrCreateCorner(cell.x, cell.y + 1, cell.z),     getOrCreateCorner(cell.x + 1, cell.y + 1, cell.z),
                                                       getOrCreateCorner(cell.x, cell.y, cell.z + 1),     getOrCreateCorner(cell.x + 1, cell.y, cell.z + 1),
                                                       getOrCreateCorner(cell.x, cell.y + 1, cell.z + 1), getOrCreateCorner(cell.x + 1, cell.y + 1, cell.z + 1) };

                for (const auto& pair : cellStrutPairs)
                {
                    addUniqueEdge(corners[static_cast<size_t>(pair.first)], corners[static_cast<size_t>(pair.second)], clampedVolCompliance);
                }

                bool alternateParity = ((cell.x + cell.y + cell.z) & 1ULL) != 0ULL;
                addCellTetrahedra(sharedSettings, corners, alternateParity, clampedVolCompliance);
            }

            const size_t                                bindCount = std::min<size_t>(4, interiorIndices.size());
            std::vector<std::pair<double, JPH::uint32>> distances;
            distances.resize(interiorIndices.size());

            for (size_t sIdx = 0; sIdx < surfaceCount; sIdx++)
            {
                const glm::dvec3& sp = weldedPositions[sIdx];
                for (size_t i = 0; i < interiorIndices.size(); i++)
                {
                    glm::dvec3 diff = interiorPositions[i] - sp;
                    distances[i]    = std::make_pair(glm::dot(diff, diff), interiorIndices[i]);
                }
                std::partial_sort(distances.begin(),
                                  distances.begin() + static_cast<std::ptrdiff_t>(bindCount),
                                  distances.end(),
                                  [](const std::pair<double, JPH::uint32>& lhs, const std::pair<double, JPH::uint32>& rhs) { return lhs.first < rhs.first; });

                for (size_t k = 0; k < bindCount; k++)
                {
                    if (distances[k].first > 1e-8)
                    {
                        addUniqueEdge(static_cast<JPH::uint32>(sIdx), distances[k].second, clampedEdgeCompliance);
                    }
                }
            }

            sharedSettings->CalculateEdgeLengths();
            sharedSettings->CalculateVolumeConstraintVolumes();
        }

        assignUniformParticleMass(sharedSettings, mass);
        sharedSettings->Optimize();
        return sharedSettings;
    }

    float VolumetricSoftBodyBuilderJ::resolvePressure(double) const
    {
        return 0.0f;
    }

    void VolumetricSoftBodyBuilderJ::addCellTetrahedra(JPH::Ref<JPH::SoftBodySharedSettings>& sharedSettings,
                                                       const std::array<JPH::uint32, 8>&      corners,
                                                       bool                                   alternateParity,
                                                       float                                  volumeCompliance) const
    {
        const std::array<std::array<int, 4>, 5> evenTets = {
            std::array<int, 4>{ 0, 1, 3, 5 },
             std::array<int, 4>{ 0, 3, 2, 6 },
             std::array<int, 4>{ 0, 5, 4, 6 },
             std::array<int, 4>{ 3, 5, 6, 7 },
             std::array<int, 4>{ 0, 3, 5, 6 }
        };

        const std::array<std::array<int, 4>, 5> oddTets = {
            std::array<int, 4>{ 1, 2, 0, 4 },
             std::array<int, 4>{ 1, 3, 2, 7 },
             std::array<int, 4>{ 1, 4, 5, 7 },
             std::array<int, 4>{ 2, 4, 6, 7 },
             std::array<int, 4>{ 1, 2, 4, 7 }
        };

        const auto& tets = alternateParity ? oddTets : evenTets;
        for (const auto& tet : tets)
        {
            JPH::uint32 v0 = corners[static_cast<size_t>(tet[0])];
            JPH::uint32 v1 = corners[static_cast<size_t>(tet[1])];
            JPH::uint32 v2 = corners[static_cast<size_t>(tet[2])];
            JPH::uint32 v3 = corners[static_cast<size_t>(tet[3])];

            JPH::Vec3 p0(sharedSettings->mVertices[v0].mPosition);
            JPH::Vec3 p1(sharedSettings->mVertices[v1].mPosition);
            JPH::Vec3 p2(sharedSettings->mVertices[v2].mPosition);
            JPH::Vec3 p3(sharedSettings->mVertices[v3].mPosition);
            float     vol = (p1 - p0).Cross(p2 - p0).Dot(p3 - p0);
            if (vol >= 0.0f)
            {
                sharedSettings->mVolumeConstraints.push_back(JPH::SoftBodySharedSettings::Volume(v0, v1, v2, v3, volumeCompliance));
            }
            else
            {
                sharedSettings->mVolumeConstraints.push_back(JPH::SoftBodySharedSettings::Volume(v0, v2, v1, v3, volumeCompliance));
            }
        }
    }
}
