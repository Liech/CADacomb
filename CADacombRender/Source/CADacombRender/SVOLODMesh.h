#pragma once

#include "CADacomb/Volume/AABB3.h"
#include "CADacomb/Volume/LODStrategy/LODStrategy.h"
#include "CADacomb/Volume/SVO/SVO.h"
#include "CADacomb/Volume/SVOMesher/SVOCPUMesher.h"
#include "CADacombRender/SVOLODChunk.h"
#include "CADacombRender/SceneDataConverter.h"
#include "CADacombRender/VulcantFragmentationBuffer.h"
#include "Vulcant/Toolkit/MeshRenderer.h"
#include "Vulcant/Toolkit/SceneData.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace CADacomb::Render
{
    template<typename T = bool>
    class SVOLODMesh
    {
      public:
        SVOLODMesh(Vulcant::VulcantDevice& device, const CADacomb::SVO<T>& svo, const CADacomb::AABB3& volumeBounds, size_t vertexCapacity, uint32_t chunkSize = 32, int initialLOD = -1);
        virtual ~SVOLODMesh();

        void markDirty(const CADacomb::AABB3& dirtyBounds);
        void markAllDirty();

        void update(const Vulcant::Toolkit::SceneData& sceneData, CADacomb::LODStrategy& strategy);

        CADacomb::SVO<T>&               getSVO();
        const CADacomb::SVO<T>&         getSVO() const;
        Vulcant::VulcantBuffer&         getVertexBuffer();
        Vulcant::VulcantBuffer&         getIndirectBuffer();
        size_t                          getVertexCount() const;
        const CADacomb::AABB3&          getVolumeBounds() const;
        const std::vector<SVOLODChunk>& getChunks() const;
        const std::vector<SVOLODChunk>& getBorderChunks() const;

        const glm::vec3& getColor() const;
        void             setColor(const glm::vec3& color);

      private:
        struct BorderLink
        {
            int     chunkIndexA = -1;
            int     chunkIndexB = -1;
            uint8_t axis        = 0;
        };

        void   remeshChunk(SVOLODChunk& chunk, int previousLOD = -1);
        void   remeshBorder(size_t borderIndex);
        bool   uploadMeshChunk(SVOLODChunk& targetChunk, const CADacomb::MeshChunk& meshChunk);
        void   markChunkBordersDirty(size_t chunkIndex);
        size_t getChunkIndex(uint32_t x, uint32_t y, uint32_t z) const;

        CADacomb::Render::VulcantFragmentationBuffer<Vulcant::Toolkit::MeshVertex> buffer;
        const CADacomb::SVO<T>&                                                    svo;
        CADacomb::AABB3                                                            volumeBounds;
        std::vector<SVOLODChunk>                                                   chunks;
        std::vector<SVOLODChunk>                                                   borderChunks;
        std::vector<BorderLink>                                                    borderLinks;
        glm::vec3                                                                  meshColor          = glm::vec3(0.8f, 0.8f, 0.8f);
        double                                                                     leafVoxelLength    = 1.0;
        uint32_t                                                                   chunkSize          = 32;
        uint32_t                                                                   chunksPerDim       = 0;
        uint32_t                                                                   borderCountPerAxis = 0;
        int                                                                        maxLOD             = 0;
        std::unique_ptr<Vulcant::VulcantBuffer>                                    indirectBuffer;
    };

    template<typename T>
    SVOLODMesh<T>::SVOLODMesh(Vulcant::VulcantDevice& device, const CADacomb::SVO<T>& svo, const CADacomb::AABB3& volumeBounds, size_t vertexCapacity, uint32_t chunkSize, int initialLOD)
      : buffer(device, vertexCapacity, Vulcant::VulcantBufferType::Vertex)
      , svo(svo)
      , volumeBounds(volumeBounds)
      , chunkSize(chunkSize)
    {
        uint32_t dim    = svo.getDimension();
        leafVoxelLength = volumeBounds.getSize().x / static_cast<double>(dim);

        uint32_t temp = chunkSize;
        maxLOD        = 0;
        while (temp >>= 1)
        {
            maxLOD++;
        }

        int startingLOD = (initialLOD < 0) ? maxLOD : std::clamp(initialLOD, 0, maxLOD);

        chunksPerDim = (dim + chunkSize - 1) / chunkSize;
        for (uint32_t z = 0; z < chunksPerDim; z++)
        {
            for (uint32_t y = 0; y < chunksPerDim; y++)
            {
                for (uint32_t x = 0; x < chunksPerDim; x++)
                {
                    glm::ivec3      minBound(x * chunkSize, y * chunkSize, z * chunkSize);
                    glm::dvec3      chunkMin = volumeBounds.getMin() + glm::dvec3(minBound) * leafVoxelLength;
                    glm::dvec3      chunkMax = chunkMin + glm::dvec3(chunkSize) * leafVoxelLength;
                    CADacomb::AABB3 chunkBounds(chunkMin, chunkMax);
                    chunks.emplace_back(minBound, chunkSize, chunkBounds, startingLOD);
                }
            }
        }

        borderCountPerAxis = (chunksPerDim + 1) * chunksPerDim * chunksPerDim;
        borderChunks.reserve(3 * borderCountPerAxis);
        borderLinks.reserve(3 * borderCountPerAxis);

        double worldChunkLen = static_cast<double>(chunkSize) * leafVoxelLength;

        for (uint32_t z = 0; z < chunksPerDim; z++)
        {
            for (uint32_t y = 0; y < chunksPerDim; y++)
            {
                for (uint32_t x = 0; x <= chunksPerDim; x++)
                {
                    glm::ivec3      negMinBound((static_cast<int>(x) - 1) * static_cast<int>(chunkSize), static_cast<int>(y * chunkSize), static_cast<int>(z * chunkSize));
                    double          planeX = volumeBounds.getMin().x + static_cast<double>(x) * worldChunkLen;
                    double          minY   = volumeBounds.getMin().y + static_cast<double>(y) * worldChunkLen;
                    double          minZ   = volumeBounds.getMin().z + static_cast<double>(z) * worldChunkLen;
                    CADacomb::AABB3 bBounds(glm::dvec3(planeX, minY, minZ), glm::dvec3(planeX, minY + worldChunkLen, minZ + worldChunkLen));

                    borderChunks.emplace_back(negMinBound, chunkSize, bBounds, startingLOD);
                    borderLinks.push_back(BorderLink{
                      .chunkIndexA = (x > 0) ? static_cast<int>(getChunkIndex(x - 1, y, z)) : -1, .chunkIndexB = (x < chunksPerDim) ? static_cast<int>(getChunkIndex(x, y, z)) : -1, .axis = 0 });
                }
            }
        }

        for (uint32_t z = 0; z < chunksPerDim; z++)
        {
            for (uint32_t y = 0; y <= chunksPerDim; y++)
            {
                for (uint32_t x = 0; x < chunksPerDim; x++)
                {
                    glm::ivec3      negMinBound(static_cast<int>(x * chunkSize), (static_cast<int>(y) - 1) * static_cast<int>(chunkSize), static_cast<int>(z * chunkSize));
                    double          minX   = volumeBounds.getMin().x + static_cast<double>(x) * worldChunkLen;
                    double          planeY = volumeBounds.getMin().y + static_cast<double>(y) * worldChunkLen;
                    double          minZ   = volumeBounds.getMin().z + static_cast<double>(z) * worldChunkLen;
                    CADacomb::AABB3 bBounds(glm::dvec3(minX, planeY, minZ), glm::dvec3(minX + worldChunkLen, planeY, minZ + worldChunkLen));

                    borderChunks.emplace_back(negMinBound, chunkSize, bBounds, startingLOD);
                    borderLinks.push_back(BorderLink{
                      .chunkIndexA = (y > 0) ? static_cast<int>(getChunkIndex(x, y - 1, z)) : -1, .chunkIndexB = (y < chunksPerDim) ? static_cast<int>(getChunkIndex(x, y, z)) : -1, .axis = 1 });
                }
            }
        }

        for (uint32_t z = 0; z <= chunksPerDim; z++)
        {
            for (uint32_t y = 0; y < chunksPerDim; y++)
            {
                for (uint32_t x = 0; x < chunksPerDim; x++)
                {
                    glm::ivec3      negMinBound(static_cast<int>(x * chunkSize), static_cast<int>(y * chunkSize), (static_cast<int>(z) - 1) * static_cast<int>(chunkSize));
                    double          minX   = volumeBounds.getMin().x + static_cast<double>(x) * worldChunkLen;
                    double          minY   = volumeBounds.getMin().y + static_cast<double>(y) * worldChunkLen;
                    double          planeZ = volumeBounds.getMin().z + static_cast<double>(z) * worldChunkLen;
                    CADacomb::AABB3 bBounds(glm::dvec3(minX, minY, planeZ), glm::dvec3(minX + worldChunkLen, minY + worldChunkLen, planeZ));

                    borderChunks.emplace_back(negMinBound, chunkSize, bBounds, startingLOD);
                    borderLinks.push_back(BorderLink{
                      .chunkIndexA = (z > 0) ? static_cast<int>(getChunkIndex(x, y, z - 1)) : -1, .chunkIndexB = (z < chunksPerDim) ? static_cast<int>(getChunkIndex(x, y, z)) : -1, .axis = 2 });
                }
            }
        }

        indirectBuffer          = device.createIndirectBuffer(1, sizeof(uint32_t) * 4);
        uint32_t indirectCmd[4] = { 0u, 1u, 0u, 0u };
        indirectBuffer->uploadToGPU(indirectCmd, 1);
    }

    template<typename T>
    SVOLODMesh<T>::~SVOLODMesh()
    {
    }

    template<typename T>
    void SVOLODMesh<T>::markDirty(const CADacomb::AABB3& dirtyBounds)
    {
        for (size_t i = 0; i < chunks.size(); i++)
        {
            if (chunks[i].getWorldBounds().intersects(dirtyBounds))
            {
                chunks[i].setDirty(true);
                markChunkBordersDirty(i);
            }
        }
    }

    template<typename T>
    void SVOLODMesh<T>::markAllDirty()
    {
        for (auto& chunk : chunks)
        {
            chunk.setDirty(true);
        }
        for (auto& border : borderChunks)
        {
            border.setDirty(true);
        }
    }

    template<typename T>
    void SVOLODMesh<T>::update(const Vulcant::Toolkit::SceneData& sceneData, CADacomb::LODStrategy& strategy)
    {
        strategy.startFrame(SceneDataConverter::convert(sceneData), volumeBounds, buffer.getCapacity(), svo.getDimension());

        for (size_t i = 0; i < chunks.size(); i++)
        {
            SVOLODChunk&          chunk              = chunks[i];
            double                currentVoxelLength = chunk.getVoxelLength(leafVoxelLength);
            CADacomb::LODDecision decision           = strategy.evaluate(chunk.getWorldBounds(), currentVoxelLength, chunk.getLOD(), buffer.size());

            int  previousLOD = chunk.getLOD();
            bool lodChanged  = false;
            if (decision == CADacomb::LODDecision::Refine && chunk.getLOD() > 0)
            {
                chunk.setLOD(chunk.getLOD() - 1);
                lodChanged = true;
            }
            else if (decision == CADacomb::LODDecision::Coarsen && chunk.getLOD() < maxLOD)
            {
                chunk.setLOD(chunk.getLOD() + 1);
                lodChanged = true;
            }

            if (lodChanged || chunk.isDirty())
            {
                remeshChunk(chunk, previousLOD);
                chunk.setDirty(false);
                markChunkBordersDirty(i);
            }
        }

        for (size_t b = 0; b < borderChunks.size(); b++)
        {
            if (borderChunks[b].isDirty())
            {
                remeshBorder(b);
                borderChunks[b].setDirty(false);
            }
        }

        strategy.endFrame();
        buffer.syncGPU();
        uint32_t indirectCmd[4] = { static_cast<uint32_t>(buffer.size()), 1u, 0u, 0u };
        indirectBuffer->uploadToGPU(indirectCmd, 1);
    }

    template<typename T>
    CADacomb::SVO<T>& SVOLODMesh<T>::getSVO()
    {
        return svo;
    }

    template<typename T>
    const CADacomb::SVO<T>& SVOLODMesh<T>::getSVO() const
    {
        return svo;
    }

    template<typename T>
    Vulcant::VulcantBuffer& SVOLODMesh<T>::getVertexBuffer()
    {
        return buffer.getGPUBuffer();
    }

    template<typename T>
    Vulcant::VulcantBuffer& SVOLODMesh<T>::getIndirectBuffer()
    {
        return *indirectBuffer;
    }

    template<typename T>
    size_t SVOLODMesh<T>::getVertexCount() const
    {
        return buffer.size();
    }

    template<typename T>
    const CADacomb::AABB3& SVOLODMesh<T>::getVolumeBounds() const
    {
        return volumeBounds;
    }

    template<typename T>
    const std::vector<SVOLODChunk>& SVOLODMesh<T>::getChunks() const
    {
        return chunks;
    }

    template<typename T>
    const std::vector<SVOLODChunk>& SVOLODMesh<T>::getBorderChunks() const
    {
        return borderChunks;
    }

    template<typename T>
    const glm::vec3& SVOLODMesh<T>::getColor() const
    {
        return meshColor;
    }

    template<typename T>
    void SVOLODMesh<T>::setColor(const glm::vec3& color)
    {
        meshColor = color;
    }

    template<typename T>
    void SVOLODMesh<T>::remeshChunk(SVOLODChunk& chunk, int previousLOD)
    {
        CADacomb::MeshChunk meshChunk;
        meshChunk.minBound      = chunk.getMinBound();
        meshChunk.size          = chunk.getSize();
        meshChunk.maxResolution = chunk.getSize();
        meshChunk.lodLevel      = chunk.getLOD();

        CADacomb::SVOCPUMesher::meshChunk(svo, meshChunk);

        if (!uploadMeshChunk(chunk, meshChunk) && previousLOD != -1 && previousLOD != chunk.getLOD())
        {
            chunk.setLOD(previousLOD);
            remeshChunk(chunk, -1);
        }
    }

    template<typename T>
    void SVOLODMesh<T>::remeshBorder(size_t borderIndex)
    {
        SVOLODChunk&      border = borderChunks[borderIndex];
        const BorderLink& link   = borderLinks[borderIndex];

        bool hasNeg = (link.chunkIndexA >= 0);
        bool hasPos = (link.chunkIndexB >= 0);
        int  lodA   = hasNeg ? chunks[link.chunkIndexA].getLOD() : maxLOD;
        int  lodB   = hasPos ? chunks[link.chunkIndexB].getLOD() : maxLOD;
        if (!hasNeg)
        {
            lodA = lodB;
        }
        if (!hasPos)
        {
            lodB = lodA;
        }

        border.setLOD(std::min(lodA, lodB));

        CADacomb::MeshChunk meshChunk = CADacomb::MeshChunk::createBorder(border.getMinBound(), border.getSize(), link.axis, hasNeg, lodA, border.getSize(), hasPos, lodB, border.getSize());

        CADacomb::SVOCPUMesher::meshChunk(svo, meshChunk);
        uploadMeshChunk(border, meshChunk);
    }

    template<typename T>
    bool SVOLODMesh<T>::uploadMeshChunk(SVOLODChunk& targetChunk, const CADacomb::MeshChunk& meshChunk)
    {
        if (targetChunk.hasPackage())
        {
            buffer.remove(targetChunk.getPackageId());
            targetChunk.clearPackage();
        }

        if (meshChunk.quads.empty())
        {
            return true;
        }

        std::vector<Vulcant::Toolkit::MeshVertex> vertices;
        vertices.reserve(meshChunk.quads.size() * 6);

        glm::dvec3 chunkOrigin = volumeBounds.getMin() + glm::dvec3(targetChunk.getMinBound()) * leafVoxelLength;

        for (const auto& q : meshChunk.quads)
        {
            glm::vec3 normal  = glm::vec3(q.getNormal());
            auto      corners = q.getCorners(chunkOrigin, leafVoxelLength);

            vertices.push_back({ glm::vec3(corners[0]), normal, meshColor });
            vertices.push_back({ glm::vec3(corners[1]), normal, meshColor });
            vertices.push_back({ glm::vec3(corners[2]), normal, meshColor });

            vertices.push_back({ glm::vec3(corners[0]), normal, meshColor });
            vertices.push_back({ glm::vec3(corners[2]), normal, meshColor });
            vertices.push_back({ glm::vec3(corners[3]), normal, meshColor });
        }

        size_t id = buffer.set(vertices);
        if (id != 0)
        {
            targetChunk.setPackageId(id);
            return true;
        }
        return false;
    }

    template<typename T>
    void SVOLODMesh<T>::markChunkBordersDirty(size_t chunkIndex)
    {
        if (chunksPerDim == 0)
        {
            return;
        }

        uint32_t d  = chunksPerDim;
        uint32_t ix = static_cast<uint32_t>(chunkIndex) % d;
        uint32_t iy = (static_cast<uint32_t>(chunkIndex) / d) % d;
        uint32_t iz = static_cast<uint32_t>(chunkIndex) / (d * d);

        size_t negX = 0 * borderCountPerAxis + iz * (d * (d + 1)) + iy * (d + 1) + ix;
        size_t posX = negX + 1;

        size_t negY = 1 * borderCountPerAxis + iz * (d * (d + 1)) + iy * d + ix;
        size_t posY = negY + d;

        size_t negZ = 2 * borderCountPerAxis + iz * (d * d) + iy * d + ix;
        size_t posZ = negZ + d * d;

        borderChunks[negX].setDirty(true);
        borderChunks[posX].setDirty(true);
        borderChunks[negY].setDirty(true);
        borderChunks[posY].setDirty(true);
        borderChunks[negZ].setDirty(true);
        borderChunks[posZ].setDirty(true);
    }

    template<typename T>
    size_t SVOLODMesh<T>::getChunkIndex(uint32_t x, uint32_t y, uint32_t z) const
    {
        return static_cast<size_t>(z) * chunksPerDim * chunksPerDim + static_cast<size_t>(y) * chunksPerDim + static_cast<size_t>(x);
    }
}
