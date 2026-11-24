#include "CADacombRender/SVORaycast/TetraProxy/SVOTetraCageBuilder.h"

#include <algorithm>
#include <array>
#include <unordered_map>

namespace CADacomb::Render
{
    SVOTetraCageBuilder::SVOTetraCageBuilder() {}

    SVOTetraCageBuilder::~SVOTetraCageBuilder() {}

    bool SVOTetraCageBuilder::isCellOccupied(const CADacomb::SVO<bool>& svo, uint32_t cx, uint32_t cy, uint32_t cz, uint32_t cellSize)
    {
        const auto& nodes   = svo.getRaw();
        uint32_t    nodeIdx = 0u;
        uint32_t    curSize = svo.getDimension();
        uint32_t    tx      = cx * cellSize;
        uint32_t    ty      = cy * cellSize;
        uint32_t    tz      = cz * cellSize;
        uint32_t    ox      = 0u;
        uint32_t    oy      = 0u;
        uint32_t    oz      = 0u;

        while (curSize > cellSize)
        {
            const auto& node = nodes[nodeIdx];
            if (node.childPointer == 0xFFFFFFFFu)
            {
                return node.value;
            }

            uint32_t half   = curSize >> 1u;
            uint32_t octant = 0u;
            if (tx >= ox + half)
            {
                octant |= 1u;
                ox += half;
            }
            if (ty >= oy + half)
            {
                octant |= 2u;
                oy += half;
            }
            if (tz >= oz + half)
            {
                octant |= 4u;
                oz += half;
            }

            nodeIdx = node.childPointer + octant;
            curSize = half;
        }

        const auto& cellNode = nodes[nodeIdx];
        return cellNode.value || (cellNode.childPointer != 0xFFFFFFFFu);
    }

    uint32_t SVOTetraCageBuilder::clampPowerOfTwo(uint32_t value, uint32_t maxValue)
    {
        uint32_t clamped = std::clamp(value, 1u, std::max(1u, maxValue));
        uint32_t pot     = 1u;
        while (pot * 2u <= clamped)
        {
            pot *= 2u;
        }
        return pot;
    }

    int SVOTetraCageBuilder::log2Uint(uint32_t value)
    {
        int exp = 0;
        while (value >>= 1u)
        {
            exp++;
        }
        return exp;
    }

    uint64_t SVOTetraCageBuilder::makeFaceKey(uint32_t a, uint32_t b, uint32_t c)
    {
        if (a > b)
        {
            std::swap(a, b);
        }
        if (b > c)
        {
            std::swap(b, c);
        }
        if (a > b)
        {
            std::swap(a, b);
        }
        return (static_cast<uint64_t>(a) << 40u) | (static_cast<uint64_t>(b) << 20u) | static_cast<uint64_t>(c);
    }

    void SVOTetraCageBuilder::orientTetrahedron(const std::vector<GPUTetraVertex>& vertices, glm::uvec4& indices)
    {
        glm::vec3 p0  = glm::vec3(vertices[indices.x].restVoxelPos);
        glm::vec3 p1  = glm::vec3(vertices[indices.y].restVoxelPos);
        glm::vec3 p2  = glm::vec3(vertices[indices.z].restVoxelPos);
        glm::vec3 p3  = glm::vec3(vertices[indices.w].restVoxelPos);
        float     vol = glm::dot(glm::cross(p1 - p0, p2 - p0), p3 - p0);
        if (vol < 0.0f)
        {
            std::swap(indices.z, indices.w);
        }
    }

    void SVOTetraCageBuilder::build(const CADacomb::SVO<bool>&       svo,
                                    const CADacomb::AABB3&           volumeBounds,
                                    uint32_t                         gridResolution,
                                    std::vector<GPUTetraVertex>&     outVertices,
                                    std::vector<GPUTetrahedron>&     outTetrahedra,
                                    std::vector<TetraSurfaceVertex>& outSurfaceVertices)
    {
        outVertices.clear();
        outTetrahedra.clear();
        outSurfaceVertices.clear();

        uint32_t dim = svo.getDimension();
        if (dim == 0u || svo.getRaw().empty())
        {
            return;
        }

        uint32_t grid     = clampPowerOfTwo(gridResolution, std::min(dim, 16u));
        uint32_t cellSize = dim / grid;
        uint32_t stride   = grid + 1u;

        glm::vec3 bMin  = glm::vec3(volumeBounds.getMin());
        glm::vec3 bSize = glm::vec3(volumeBounds.getSize());

        std::vector<int32_t> gridVertexMap(static_cast<size_t>(stride) * stride * stride, -1);

        auto getOrCreateVertex = [&](uint32_t gx, uint32_t gy, uint32_t gz) -> uint32_t
        {
            size_t flatIdx = (static_cast<size_t>(gz) * stride + gy) * stride + gx;
            if (gridVertexMap[flatIdx] >= 0)
            {
                return static_cast<uint32_t>(gridVertexMap[flatIdx]);
            }

            glm::vec3 uvw(static_cast<float>(gx) / static_cast<float>(grid),
                          static_cast<float>(gy) / static_cast<float>(grid),
                          static_cast<float>(gz) / static_cast<float>(grid));
            glm::vec3 restVoxel  = uvw * static_cast<float>(dim);
            glm::vec3 localPos   = bMin + uvw * bSize;
            uint32_t  newIdx     = static_cast<uint32_t>(outVertices.size());
            gridVertexMap[flatIdx] = static_cast<int32_t>(newIdx);
            outVertices.emplace_back(restVoxel, localPos);
            return newIdx;
        };

        const std::array<std::array<uint32_t, 4>, 5> evenTets = { {
          { 0u, 3u, 5u, 6u },
          { 1u, 0u, 3u, 5u },
          { 2u, 0u, 6u, 3u },
          { 4u, 0u, 5u, 6u },
          { 7u, 3u, 6u, 5u },
        } };

        const std::array<std::array<uint32_t, 4>, 5> oddTets = { {
          { 1u, 2u, 4u, 7u },
          { 0u, 1u, 2u, 4u },
          { 3u, 1u, 7u, 2u },
          { 5u, 1u, 4u, 7u },
          { 6u, 2u, 7u, 4u },
        } };

        for (uint32_t cz = 0u; cz < grid; cz++)
        {
            for (uint32_t cy = 0u; cy < grid; cy++)
            {
                for (uint32_t cx = 0u; cx < grid; cx++)
                {
                    if (!isCellOccupied(svo, cx, cy, cz, cellSize))
                    {
                        continue;
                    }

                    std::array<uint32_t, 8> corners = {
                        getOrCreateVertex(cx, cy, cz),
                        getOrCreateVertex(cx + 1u, cy, cz),
                        getOrCreateVertex(cx, cy + 1u, cz),
                        getOrCreateVertex(cx + 1u, cy + 1u, cz),
                        getOrCreateVertex(cx, cy, cz + 1u),
                        getOrCreateVertex(cx + 1u, cy, cz + 1u),
                        getOrCreateVertex(cx, cy + 1u, cz + 1u),
                        getOrCreateVertex(cx + 1u, cy + 1u, cz + 1u),
                    };

                    const auto& tetTable = (((cx + cy + cz) & 1u) == 0u) ? evenTets : oddTets;
                    for (const auto& t : tetTable)
                    {
                        glm::uvec4 vIdx(corners[t[0]], corners[t[1]], corners[t[2]], corners[t[3]]);
                        orientTetrahedron(outVertices, vIdx);
                        outTetrahedra.emplace_back(vIdx, glm::ivec4(-1));
                    }
                }
            }
        }

        const std::array<std::array<uint32_t, 3>, 4> oppositeFace = { {
          { 1u, 2u, 3u },
          { 0u, 3u, 2u },
          { 0u, 1u, 3u },
          { 0u, 2u, 1u },
        } };

        std::unordered_map<uint64_t, uint64_t> faceMap;
        faceMap.reserve(outTetrahedra.size() * 4u);

        for (size_t tIdx = 0; tIdx < outTetrahedra.size(); tIdx++)
        {
            const glm::uvec4& v = outTetrahedra[tIdx].vertexIndices;
            for (uint32_t f = 0u; f < 4u; f++)
            {
                uint32_t va  = v[oppositeFace[f][0]];
                uint32_t vb  = v[oppositeFace[f][1]];
                uint32_t vc  = v[oppositeFace[f][2]];
                uint64_t key = makeFaceKey(va, vb, vc);

                auto it = faceMap.find(key);
                if (it == faceMap.end())
                {
                    faceMap.emplace(key, (static_cast<uint64_t>(tIdx) << 2u) | static_cast<uint64_t>(f));
                }
                else
                {
                    uint32_t otherTet  = static_cast<uint32_t>(it->second >> 2u);
                    uint32_t otherFace = static_cast<uint32_t>(it->second & 3u);
                    outTetrahedra[tIdx].neighborTets[f]            = static_cast<int32_t>(otherTet);
                    outTetrahedra[otherTet].neighborTets[otherFace] = static_cast<int32_t>(tIdx);
                    faceMap.erase(it);
                }
            }
        }

        outSurfaceVertices.reserve(faceMap.size() * 3u);
        for (size_t tIdx = 0; tIdx < outTetrahedra.size(); tIdx++)
        {
            const glm::uvec4& v = outTetrahedra[tIdx].vertexIndices;
            const glm::ivec4& n = outTetrahedra[tIdx].neighborTets;
            for (uint32_t f = 0u; f < 4u; f++)
            {
                if (n[f] >= 0)
                {
                    continue;
                }

                uint32_t va            = v[oppositeFace[f][0]];
                uint32_t vb            = v[oppositeFace[f][1]];
                uint32_t vc            = v[oppositeFace[f][2]];
                uint32_t packedTetFace = (static_cast<uint32_t>(tIdx) << 2u) | f;
                outSurfaceVertices.emplace_back(va, packedTetFace);
                outSurfaceVertices.emplace_back(vb, packedTetFace);
                outSurfaceVertices.emplace_back(vc, packedTetFace);
            }
        }
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb::Render
{
    TEST_CASE("SVOTetraCageBuilder/SingleCubeConformingCage")
    {
        CADacomb::SVO<bool> svo(4u);
        for (int z = 0; z < 4; z++)
        {
            for (int y = 0; y < 4; y++)
            {
                for (int x = 0; x < 4; x++)
                {
                    svo.setVoxel(glm::ivec3(x, y, z), true);
                }
            }
        }

        std::vector<GPUTetraVertex>     vertices;
        std::vector<GPUTetrahedron>     tetrahedra;
        std::vector<TetraSurfaceVertex> surfaceVertices;
        CADacomb::AABB3                 bounds(glm::dvec3(0.0), glm::dvec3(4.0));

        SVOTetraCageBuilder::build(svo, bounds, 2u, vertices, tetrahedra, surfaceVertices);

        REQUIRE(vertices.size() == 27u);
        REQUIRE(tetrahedra.size() == 40u);
        REQUIRE(surfaceVertices.size() == 144u);
    }

    TEST_CASE("SVOTetraCageBuilder/SparseSingleVoxelCellIncluded")
    {
        CADacomb::SVO<bool> svo(4u);
        svo.setVoxel(glm::ivec3(0, 0, 0), true);

        std::vector<GPUTetraVertex>     vertices;
        std::vector<GPUTetrahedron>     tetrahedra;
        std::vector<TetraSurfaceVertex> surfaceVertices;
        CADacomb::AABB3                 bounds(glm::dvec3(0.0), glm::dvec3(4.0));

        SVOTetraCageBuilder::build(svo, bounds, 2u, vertices, tetrahedra, surfaceVertices);

        REQUIRE(tetrahedra.size() == 5u);
        REQUIRE(surfaceVertices.size() == 36u);
    }
}
#endif
