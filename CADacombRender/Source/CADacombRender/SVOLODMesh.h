#pragma once

#include "CADacomb/Util/AABB3.h"
#include "CADacomb/Volume/LODStrategy/LODStrategy.h"
#include "CADacomb/Volume/SVO/SVO.h"
#include "CADacomb/Volume/SVO/SVODualTraversal.h"
#include "CADacomb/Volume/SVOMesher/SVOCPUMesher.h"
#include "CADacombRender/SVOLOD/SVOCompletedMesh.h"
#include "CADacombRender/SVOLOD/SVOMeshJob.h"
#include "CADacombRender/SVOLODChunk.h"
#include "CADacombRender/SceneDataConverter.h"
#include "CADacombRender/VulcantFragmentationBuffer.h"
#include "Vulcant/Toolkit/MeshRenderer.h"
#include "Vulcant/Toolkit/SceneData.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

namespace CADacomb::Render
{
    // Hierarchical octree-based LOD mesh manager for SVOs with async background meshing, atomic swaps, and budgeted GPU uploads.
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

        size_t getMaxUploadsPerFrame() const;
        void   setMaxUploadsPerFrame(size_t maxUploads);

        double getMaxUploadTimeMs() const;
        void   setMaxUploadTimeMs(double maxMs);

        bool isAsyncEnabled() const;
        void setAsyncEnabled(bool enabled);

        bool isAmbientOcclusionEnabled() const;
        void setAmbientOcclusionEnabled(bool enabled);

      private:
        static int log2Uint(uint32_t value);
        int        getMinLocalLOD(uint32_t nodeSize) const;
        int        getMaxLocalLOD(uint32_t nodeSize) const;
        bool       isSVONodeLeaf(const glm::ivec3& minBound, uint32_t size) const;
        bool       isVoxelSolidAt(const glm::ivec3& voxelPos, int lodDepth) const;
        void       startWorkers();
        void       stopAllWorkers();
        void       workerLoop();
        void       markNodeDirty(size_t nodeIndex);
        void       ensureChildrenAllocated(size_t parentIndex);
        void       beginSubdivideNode(size_t nodeIndex, const glm::dvec3& camPos, std::vector<SVOMeshJob>& outJobs);
        void       tryCommitSplit(size_t parentIndex);
        void       beginCollapseChildren(size_t parentIndex, const glm::dvec3& camPos, std::vector<SVOMeshJob>& outJobs);
        void       tryCommitCollapse(size_t parentIndex, const std::vector<Vulcant::Toolkit::MeshVertex>& parentVerts, int parentLod);
        void       collectIntersectingLeaves(size_t nodeIndex, const glm::ivec3& boxMin, const glm::ivec3& boxMax, std::vector<size_t>& outLeaves) const;
        void       getNeighborLODs(size_t nodeIndex, std::array<int, 6>& outNeighborLODs) const;
        void       markFaceNeighborsDirty(size_t nodeIndex);
        void       buildQuadVertices(const glm::ivec3&                          chunkMinBound,
                                     uint32_t                                   coordShift,
                                     const glm::vec3&                           color,
                                     bool                                       computeAO,
                                     const std::vector<CADacomb::PackedQuad>&   quads,
                                     std::vector<Vulcant::Toolkit::MeshVertex>& outVertices) const;
        bool       canFitReplacement(size_t freedVertices, size_t newVertices) const;
        bool       uploadVertices(size_t chunkIndex, const std::vector<Vulcant::Toolkit::MeshVertex>& vertices);
        void       releaseChunkPackage(size_t chunkIndex);
        SVOMeshJob createJobForNode(size_t nodeIndex, int targetLod, const glm::dvec3& camPos) const;

        CADacomb::Render::VulcantFragmentationBuffer<Vulcant::Toolkit::MeshVertex> buffer;
        const CADacomb::SVO<T>&                                                    svo;
        CADacomb::AABB3                                                            volumeBounds;
        std::vector<SVOLODChunk>                                                   chunks;
        std::vector<SVOLODChunk>                                                   borderChunks;
        std::vector<int32_t>                                                       nodeParent;
        std::vector<int32_t>                                                       nodeFirstChild;
        std::vector<uint8_t>                                                       nodeActive;
        std::vector<uint8_t>                                                       nodeQueued;
        std::vector<uint8_t>                                                       nodeSplitting;
        std::vector<uint8_t>                                                       nodeCollapsing;
        std::vector<uint8_t>                                                       nodeStagedReady;
        std::vector<int>                                                           nodeTargetLOD;
        std::vector<uint64_t>                                                      nodeGeneration;
        std::vector<size_t>                                                        nodePackageVertexCount;
        std::vector<std::vector<Vulcant::Toolkit::MeshVertex>>                     nodeStagedVertices;
        glm::vec3                                                                  meshColor               = glm::vec3(0.8f, 0.8f, 0.8f);
        double                                                                     leafVoxelLength         = 1.0;
        uint32_t                                                                   minChunkSize            = 32;
        uint32_t                                                                   svoDimension            = 0;
        int                                                                        maxTreeDepth            = 0;
        size_t                                                                     maxUploadsPerFrame      = 8;
        double                                                                     maxUploadTimeMs         = 2.0;
        bool                                                                       asyncEnabled            = true;
        bool                                                                       ambientOcclusionEnabled = false;
        bool                                                                       gpuBufferDirty          = false;
        std::unique_ptr<Vulcant::VulcantBuffer>                                    indirectBuffer;

        std::mutex                   jobMutex;
        std::condition_variable      jobCv;
        std::vector<SVOMeshJob>      pendingJobs;
        std::mutex                   completedMutex;
        std::deque<SVOCompletedMesh> completedMeshes;
        std::vector<std::thread>     workers;
        bool                         stopWorkers = false;
    };

    template<typename T>
    int SVOLODMesh<T>::log2Uint(uint32_t value)
    {
        int exp = 0;
        while (value >>= 1)
        {
            exp++;
        }
        return exp;
    }

    template<typename T>
    int SVOLODMesh<T>::getMinLocalLOD(uint32_t nodeSize) const
    {
        if (nodeSize <= minChunkSize)
        {
            return 0;
        }
        return log2Uint(nodeSize / minChunkSize);
    }

    template<typename T>
    int SVOLODMesh<T>::getMaxLocalLOD(uint32_t nodeSize) const
    {
        return log2Uint(nodeSize);
    }

    template<typename T>
    bool SVOLODMesh<T>::isSVONodeLeaf(const glm::ivec3& minBound, uint32_t size) const
    {
        CADacomb::SVODualNode<T> node = CADacomb::SVODualTraversal::findSubtreeNode(svo, minBound, size);
        return node.isLeaf || node.isVirtual;
    }

    template<typename T>
    bool SVOLODMesh<T>::isVoxelSolidAt(const glm::ivec3& voxelPos, int lodDepth) const
    {
        int dim = static_cast<int>(svoDimension);
        if (voxelPos.x < 0 || voxelPos.y < 0 || voxelPos.z < 0 || voxelPos.x >= dim || voxelPos.y >= dim || voxelPos.z >= dim)
        {
            return false;
        }

        T val = svo.getVoxel(voxelPos, lodDepth);
        if constexpr (std::is_same_v<T, bool>)
        {
            return val;
        }
        else if constexpr (requires { val.fill; })
        {
            return val.fill > 0;
        }
        else if constexpr (requires { val > 0; })
        {
            return val > 0;
        }
        else
        {
            return static_cast<bool>(val);
        }
    }

    template<typename T>
    SVOLODMesh<T>::SVOLODMesh(Vulcant::VulcantDevice& device, const CADacomb::SVO<T>& svo, const CADacomb::AABB3& volumeBounds, size_t vertexCapacity, uint32_t chunkSize, int initialLOD)
      : buffer(device, vertexCapacity, Vulcant::VulcantBufferType::Vertex)
      , svo(svo)
      , volumeBounds(volumeBounds)
    {
        svoDimension    = svo.getDimension();
        leafVoxelLength = (svoDimension > 0) ? (volumeBounds.getSize().x / static_cast<double>(svoDimension)) : 1.0;
        maxTreeDepth    = log2Uint(svoDimension);

        uint32_t clampedChunk = std::clamp(chunkSize, 8u, std::max(8u, svoDimension));
        uint32_t potChunk     = 1;
        while (potChunk * 2 <= clampedChunk)
        {
            potChunk *= 2;
        }
        minChunkSize = std::min(potChunk, std::max(1u, svoDimension));

        int rootMinLOD  = getMinLocalLOD(svoDimension);
        int rootMaxLOD  = getMaxLocalLOD(svoDimension);
        int startingLOD = (initialLOD < 0) ? rootMaxLOD : std::clamp(initialLOD, rootMinLOD, rootMaxLOD);

        chunks.emplace_back(glm::ivec3(0), svoDimension, volumeBounds, startingLOD);
        nodeParent.push_back(-1);
        nodeFirstChild.push_back(-1);
        nodeActive.push_back(1);
        nodeQueued.push_back(0);
        nodeSplitting.push_back(0);
        nodeCollapsing.push_back(0);
        nodeStagedReady.push_back(0);
        nodeTargetLOD.push_back(startingLOD);
        nodeGeneration.push_back(1);
        nodePackageVertexCount.push_back(0);
        nodeStagedVertices.emplace_back();

        indirectBuffer          = device.createIndirectBuffer(1, sizeof(uint32_t) * 4);
        uint32_t indirectCmd[4] = { 0u, 1u, 0u, 0u };
        indirectBuffer->uploadToGPU(indirectCmd, 1);

        startWorkers();
    }

    template<typename T>
    SVOLODMesh<T>::~SVOLODMesh()
    {
        stopAllWorkers();
    }

    template<typename T>
    void SVOLODMesh<T>::startWorkers()
    {
        if (!workers.empty())
        {
            return;
        }

        stopWorkers        = false;
        unsigned int cores = std::thread::hardware_concurrency();
        unsigned int count = std::max(1u, std::min(4u, cores > 1 ? cores / 2 : 1u));
        workers.reserve(count);
        for (unsigned int i = 0; i < count; i++)
        {
            workers.emplace_back(&SVOLODMesh<T>::workerLoop, this);
        }
    }

    template<typename T>
    void SVOLODMesh<T>::stopAllWorkers()
    {
        {
            std::lock_guard<std::mutex> lock(jobMutex);
            stopWorkers = true;
            pendingJobs.clear();
        }
        jobCv.notify_all();

        for (auto& w : workers)
        {
            if (w.joinable())
            {
                w.join();
            }
        }
        workers.clear();

        {
            std::lock_guard<std::mutex> lock(completedMutex);
            completedMeshes.clear();
        }
    }

    template<typename T>
    void SVOLODMesh<T>::workerLoop()
    {
        std::vector<CADacomb::PackedQuad> localQuads;

        while (true)
        {
            SVOMeshJob job;
            {
                std::unique_lock<std::mutex> lock(jobMutex);
                jobCv.wait(lock, [&]() { return stopWorkers || !pendingJobs.empty(); });
                if (stopWorkers)
                {
                    return;
                }

                size_t bestIdx = 0;
                double bestD   = pendingJobs[0].distSq;
                for (size_t i = 1; i < pendingJobs.size(); i++)
                {
                    if (pendingJobs[i].distSq < bestD)
                    {
                        bestD   = pendingJobs[i].distSq;
                        bestIdx = i;
                    }
                }

                job                  = pendingJobs[bestIdx];
                pendingJobs[bestIdx] = pendingJobs.back();
                pendingJobs.pop_back();
            }

            uint32_t coordShift = 0;
            CADacomb::SVOCPUMesher::meshChunkWithBoundaries(svo, job.minBound, job.size, job.lodLevel, job.neighborLODs, localQuads, coordShift);

            SVOCompletedMesh completed;
            completed.chunkIndex = job.chunkIndex;
            completed.generation = job.generation;
            completed.lodLevel   = job.lodLevel;
            buildQuadVertices(job.minBound, coordShift, job.color, job.computeAO, localQuads, completed.vertices);

            {
                std::lock_guard<std::mutex> lock(completedMutex);
                completedMeshes.push_back(std::move(completed));
            }
        }
    }

    template<typename T>
    void SVOLODMesh<T>::markNodeDirty(size_t nodeIndex)
    {
        chunks[nodeIndex].setDirty(true);
        nodeQueued[nodeIndex] = 0;
        nodeGeneration[nodeIndex]++;
    }

    template<typename T>
    void SVOLODMesh<T>::markDirty(const CADacomb::AABB3& dirtyBounds)
    {
        for (size_t i = 0; i < chunks.size(); i++)
        {
            if (nodeActive[i] && chunks[i].getWorldBounds().intersects(dirtyBounds))
            {
                markNodeDirty(i);
                markFaceNeighborsDirty(i);
            }
        }
    }

    template<typename T>
    void SVOLODMesh<T>::markAllDirty()
    {
        for (size_t i = 0; i < chunks.size(); i++)
        {
            if (nodeActive[i])
            {
                markNodeDirty(i);
            }
        }
    }

    template<typename T>
    bool SVOLODMesh<T>::canFitReplacement(size_t freedVertices, size_t newVertices) const
    {
        size_t baseSize = (buffer.size() >= freedVertices) ? (buffer.size() - freedVertices) : 0;
        return (baseSize + newVertices) <= buffer.getCapacity();
    }

    template<typename T>
    void SVOLODMesh<T>::releaseChunkPackage(size_t chunkIndex)
    {
        SVOLODChunk& targetChunk = chunks[chunkIndex];
        if (targetChunk.hasPackage())
        {
            buffer.remove(targetChunk.getPackageId());
            targetChunk.clearPackage();
            nodePackageVertexCount[chunkIndex] = 0;
            gpuBufferDirty                     = true;
        }
    }

    template<typename T>
    bool SVOLODMesh<T>::uploadVertices(size_t chunkIndex, const std::vector<Vulcant::Toolkit::MeshVertex>& vertices)
    {
        size_t oldVerts = nodePackageVertexCount[chunkIndex];
        if (!canFitReplacement(oldVerts, vertices.size()))
        {
            return false;
        }

        releaseChunkPackage(chunkIndex);

        if (vertices.empty())
        {
            return true;
        }

        size_t id = buffer.set(vertices);
        if (id != 0)
        {
            chunks[chunkIndex].setPackageId(id);
            nodePackageVertexCount[chunkIndex] = vertices.size();
            gpuBufferDirty                     = true;
            return true;
        }
        return false;
    }

    template<typename T>
    SVOMeshJob SVOLODMesh<T>::createJobForNode(size_t nodeIndex, int targetLod, const glm::dvec3& camPos) const
    {
        SVOMeshJob job;
        job.chunkIndex = nodeIndex;
        job.generation = nodeGeneration[nodeIndex];
        job.minBound   = chunks[nodeIndex].getMinBound();
        job.size       = chunks[nodeIndex].getSize();
        job.lodLevel   = targetLod;
        job.color      = meshColor;
        job.computeAO  = ambientOcclusionEnabled && (targetLod <= 1);
        glm::dvec3 d   = chunks[nodeIndex].getWorldBounds().getCenter() - camPos;
        job.distSq     = glm::dot(d, d);
        getNeighborLODs(nodeIndex, job.neighborLODs);
        return job;
    }

    template<typename T>
    void SVOLODMesh<T>::ensureChildrenAllocated(size_t parentIndex)
    {
        if (nodeFirstChild[parentIndex] >= 0)
        {
            return;
        }

        uint32_t   parentSize     = chunks[parentIndex].getSize();
        uint32_t   childSize      = parentSize >> 1;
        glm::ivec3 parentMinBound = chunks[parentIndex].getMinBound();
        int        childStartLOD  = std::clamp(chunks[parentIndex].getLOD(), getMinLocalLOD(childSize), getMaxLocalLOD(childSize));

        int32_t firstChild          = static_cast<int32_t>(chunks.size());
        nodeFirstChild[parentIndex] = firstChild;

        for (int cz = 0; cz < 2; cz++)
        {
            for (int cy = 0; cy < 2; cy++)
            {
                for (int cx = 0; cx < 2; cx++)
                {
                    glm::ivec3      childMinBound = parentMinBound + glm::ivec3(cx, cy, cz) * static_cast<int>(childSize);
                    glm::dvec3      worldMin      = volumeBounds.getMin() + glm::dvec3(childMinBound) * leafVoxelLength;
                    glm::dvec3      worldMax      = worldMin + glm::dvec3(childSize) * leafVoxelLength;
                    CADacomb::AABB3 childBounds(worldMin, worldMax);

                    chunks.emplace_back(childMinBound, childSize, childBounds, childStartLOD);
                    nodeParent.push_back(static_cast<int32_t>(parentIndex));
                    nodeFirstChild.push_back(-1);
                    nodeActive.push_back(0);
                    nodeQueued.push_back(0);
                    nodeSplitting.push_back(0);
                    nodeCollapsing.push_back(0);
                    nodeStagedReady.push_back(0);
                    nodeTargetLOD.push_back(childStartLOD);
                    nodeGeneration.push_back(1);
                    nodePackageVertexCount.push_back(0);
                    nodeStagedVertices.emplace_back();
                }
            }
        }
    }

    template<typename T>
    void SVOLODMesh<T>::beginSubdivideNode(size_t nodeIndex, const glm::dvec3& camPos, std::vector<SVOMeshJob>& outJobs)
    {
        ensureChildrenAllocated(nodeIndex);

        int32_t  firstChild    = nodeFirstChild[nodeIndex];
        uint32_t childSize     = chunks[nodeIndex].getSize() >> 1;
        int      childTargetLD = std::max(getMinLocalLOD(childSize), chunks[nodeIndex].getLOD() - 1);

        nodeSplitting[nodeIndex] = 1;

        for (int i = 0; i < 8; i++)
        {
            size_t childIdx = static_cast<size_t>(firstChild + i);
            chunks[childIdx].setLOD(childTargetLD);
            nodeTargetLOD[childIdx]   = childTargetLD;
            nodeStagedReady[childIdx] = 0;
            nodeStagedVertices[childIdx].clear();
            nodeGeneration[childIdx]++;
            nodeQueued[childIdx] = 1;
            outJobs.push_back(createJobForNode(childIdx, childTargetLD, camPos));
        }
    }

    template<typename T>
    void SVOLODMesh<T>::tryCommitSplit(size_t parentIndex)
    {
        if (!nodeSplitting[parentIndex])
        {
            return;
        }

        int32_t firstChild = nodeFirstChild[parentIndex];
        if (firstChild < 0)
        {
            nodeSplitting[parentIndex] = 0;
            return;
        }

        size_t totalChildVerts = 0;
        for (int i = 0; i < 8; i++)
        {
            size_t childIdx = static_cast<size_t>(firstChild + i);
            if (!nodeStagedReady[childIdx])
            {
                return;
            }
            totalChildVerts += nodeStagedVertices[childIdx].size();
        }

        size_t parentVerts = nodePackageVertexCount[parentIndex];
        if (!canFitReplacement(parentVerts, totalChildVerts))
        {
            nodeSplitting[parentIndex] = 0;
            for (int i = 0; i < 8; i++)
            {
                size_t childIdx           = static_cast<size_t>(firstChild + i);
                nodeStagedReady[childIdx] = 0;
                nodeQueued[childIdx]      = 0;
                nodeStagedVertices[childIdx].clear();
            }
            return;
        }

        releaseChunkPackage(parentIndex);
        nodeActive[parentIndex]    = 0;
        nodeSplitting[parentIndex] = 0;
        nodeQueued[parentIndex]    = 0;

        for (int i = 0; i < 8; i++)
        {
            size_t childIdx           = static_cast<size_t>(firstChild + i);
            nodeActive[childIdx]      = 1;
            nodeQueued[childIdx]      = 0;
            nodeStagedReady[childIdx] = 0;
            chunks[childIdx].setDirty(false);
            uploadVertices(childIdx, nodeStagedVertices[childIdx]);
            nodeStagedVertices[childIdx].clear();
            nodeStagedVertices[childIdx].shrink_to_fit();
        }

        markFaceNeighborsDirty(parentIndex);
    }

    template<typename T>
    void SVOLODMesh<T>::beginCollapseChildren(size_t parentIndex, const glm::dvec3& camPos, std::vector<SVOMeshJob>& outJobs)
    {
        if (nodeCollapsing[parentIndex])
        {
            return;
        }

        int parentLod               = getMinLocalLOD(chunks[parentIndex].getSize());
        nodeCollapsing[parentIndex] = 1;
        nodeTargetLOD[parentIndex]  = parentLod;
        nodeGeneration[parentIndex]++;
        nodeQueued[parentIndex] = 1;
        outJobs.push_back(createJobForNode(parentIndex, parentLod, camPos));
    }

    template<typename T>
    void SVOLODMesh<T>::tryCommitCollapse(size_t parentIndex, const std::vector<Vulcant::Toolkit::MeshVertex>& parentVerts, int parentLod)
    {
        if (!nodeCollapsing[parentIndex])
        {
            return;
        }

        int32_t firstChild = nodeFirstChild[parentIndex];
        if (firstChild < 0)
        {
            nodeCollapsing[parentIndex] = 0;
            return;
        }

        size_t childrenVerts = 0;
        for (int i = 0; i < 8; i++)
        {
            size_t childIdx = static_cast<size_t>(firstChild + i);
            if (!nodeActive[childIdx] || nodeSplitting[childIdx])
            {
                nodeCollapsing[parentIndex] = 0;
                nodeQueued[parentIndex]     = 0;
                return;
            }
            childrenVerts += nodePackageVertexCount[childIdx];
        }

        if (!canFitReplacement(childrenVerts, parentVerts.size()))
        {
            nodeCollapsing[parentIndex] = 0;
            nodeQueued[parentIndex]     = 0;
            return;
        }

        for (int i = 0; i < 8; i++)
        {
            size_t childIdx = static_cast<size_t>(firstChild + i);
            releaseChunkPackage(childIdx);
            nodeActive[childIdx] = 0;
            nodeQueued[childIdx] = 0;
            nodeGeneration[childIdx]++;
        }

        nodeActive[parentIndex]     = 1;
        nodeCollapsing[parentIndex] = 0;
        nodeQueued[parentIndex]     = 0;
        chunks[parentIndex].setLOD(parentLod);
        chunks[parentIndex].setDirty(false);
        uploadVertices(parentIndex, parentVerts);
        markFaceNeighborsDirty(parentIndex);
    }

    template<typename T>
    void SVOLODMesh<T>::collectIntersectingLeaves(size_t nodeIndex, const glm::ivec3& boxMin, const glm::ivec3& boxMax, std::vector<size_t>& outLeaves) const
    {
        const glm::ivec3& nMin = chunks[nodeIndex].getMinBound();
        int               sz   = static_cast<int>(chunks[nodeIndex].getSize());
        glm::ivec3        nMax = nMin + glm::ivec3(sz);

        if (nMax.x <= boxMin.x || nMin.x >= boxMax.x || nMax.y <= boxMin.y || nMin.y >= boxMax.y || nMax.z <= boxMin.z || nMin.z >= boxMax.z)
        {
            return;
        }

        if (nodeActive[nodeIndex])
        {
            outLeaves.push_back(nodeIndex);
            return;
        }

        int32_t firstChild = nodeFirstChild[nodeIndex];
        if (firstChild < 0)
        {
            return;
        }

        for (int i = 0; i < 8; i++)
        {
            collectIntersectingLeaves(static_cast<size_t>(firstChild + i), boxMin, boxMax, outLeaves);
        }
    }

    template<typename T>
    void SVOLODMesh<T>::getNeighborLODs(size_t nodeIndex, std::array<int, 6>& outNeighborLODs) const
    {
        const glm::ivec3& nMin = chunks[nodeIndex].getMinBound();
        int               sz   = static_cast<int>(chunks[nodeIndex].getSize());
        int               dim  = static_cast<int>(svoDimension);
        int               own  = chunks[nodeIndex].getLOD();

        std::vector<size_t> neighbors;
        neighbors.reserve(8);

        for (int dir = 0; dir < 6; dir++)
        {
            int  axisIdx    = dir / 2;
            bool isPositive = (dir & 1) != 0;

            glm::ivec3 boxMin = nMin;
            glm::ivec3 boxMax = nMin + glm::ivec3(sz);
            if (isPositive)
            {
                boxMin[axisIdx] = nMin[axisIdx] + sz;
                boxMax[axisIdx] = boxMin[axisIdx] + 1;
            }
            else
            {
                boxMax[axisIdx] = nMin[axisIdx];
                boxMin[axisIdx] = boxMax[axisIdx] - 1;
            }

            if (boxMin[axisIdx] < 0 || boxMin[axisIdx] >= dim)
            {
                outNeighborLODs[static_cast<size_t>(dir)] = own;
                continue;
            }

            neighbors.clear();
            collectIntersectingLeaves(0, boxMin, boxMax, neighbors);

            if (neighbors.empty())
            {
                outNeighborLODs[static_cast<size_t>(dir)] = own;
            }
            else
            {
                int minLod = chunks[neighbors[0]].getLOD();
                for (size_t k = 1; k < neighbors.size(); k++)
                {
                    minLod = std::min(minLod, chunks[neighbors[k]].getLOD());
                }
                outNeighborLODs[static_cast<size_t>(dir)] = minLod;
            }
        }
    }

    template<typename T>
    void SVOLODMesh<T>::markFaceNeighborsDirty(size_t nodeIndex)
    {
        const glm::ivec3& nMin = chunks[nodeIndex].getMinBound();
        int               sz   = static_cast<int>(chunks[nodeIndex].getSize());
        int               dim  = static_cast<int>(svoDimension);

        std::vector<size_t> neighbors;
        neighbors.reserve(16);

        for (int dir = 0; dir < 6; dir++)
        {
            int  axisIdx    = dir / 2;
            bool isPositive = (dir & 1) != 0;

            glm::ivec3 boxMin = nMin;
            glm::ivec3 boxMax = nMin + glm::ivec3(sz);
            if (isPositive)
            {
                boxMin[axisIdx] = nMin[axisIdx] + sz;
                boxMax[axisIdx] = boxMin[axisIdx] + 1;
            }
            else
            {
                boxMax[axisIdx] = nMin[axisIdx];
                boxMin[axisIdx] = boxMax[axisIdx] - 1;
            }

            if (boxMin[axisIdx] < 0 || boxMin[axisIdx] >= dim)
            {
                continue;
            }

            neighbors.clear();
            collectIntersectingLeaves(0, boxMin, boxMax, neighbors);
            for (size_t nbrIdx : neighbors)
            {
                if (!nodeSplitting[nbrIdx])
                {
                    markNodeDirty(nbrIdx);
                }
            }
        }
    }

    template<typename T>
    void SVOLODMesh<T>::buildQuadVertices(const glm::ivec3&                          chunkMinBound,
                                          uint32_t                                   coordShift,
                                          const glm::vec3&                           color,
                                          bool                                       computeAO,
                                          const std::vector<CADacomb::PackedQuad>&   quads,
                                          std::vector<Vulcant::Toolkit::MeshVertex>& outVertices) const
    {
        outVertices.clear();
        if (quads.empty())
        {
            return;
        }

        outVertices.reserve(quads.size() * 6);

        glm::dvec3 chunkOrigin     = volumeBounds.getMin() + glm::dvec3(chunkMinBound) * leafVoxelLength;
        double     shiftedVoxelLen = leafVoxelLength * static_cast<double>(1u << coordShift);

        for (const auto& q : quads)
        {
            uint32_t   dir     = q.getDirection();
            glm::ivec3 iNormal = q.getNormal();
            glm::vec3  normal  = glm::vec3(iNormal);
            auto       corners = q.getCorners(chunkOrigin, shiftedVoxelLen);

            if (!computeAO)
            {
                outVertices.push_back({ glm::vec3(corners[0]), normal, color });
                outVertices.push_back({ glm::vec3(corners[1]), normal, color });
                outVertices.push_back({ glm::vec3(corners[2]), normal, color });

                outVertices.push_back({ glm::vec3(corners[0]), normal, color });
                outVertices.push_back({ glm::vec3(corners[2]), normal, color });
                outVertices.push_back({ glm::vec3(corners[3]), normal, color });
                continue;
            }

            int        scaleShift = static_cast<int>(q.getScaleExp() + coordShift);
            int        scale      = 1 << scaleShift;
            int        lodDepth   = std::max(0, maxTreeDepth - scaleShift);
            glm::ivec3 quadVoxel  = chunkMinBound + glm::ivec3(q.getX() << coordShift, q.getY() << coordShift, q.getZ() << coordShift);
            glm::ivec3 frontBase  = quadVoxel + iNormal * scale;

            int        axisIdx = static_cast<int>(dir / 2);
            glm::ivec3 t1(0);
            glm::ivec3 t2(0);
            if (axisIdx == 0)
            {
                t1 = glm::ivec3(0, 1, 0);
                t2 = glm::ivec3(0, 0, 1);
            }
            else if (axisIdx == 1)
            {
                t1 = glm::ivec3(1, 0, 0);
                t2 = glm::ivec3(0, 0, 1);
            }
            else
            {
                t1 = glm::ivec3(1, 0, 0);
                t2 = glm::ivec3(0, 1, 0);
            }

            glm::dvec3 quadCenter = (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25;
            glm::dvec3 dt1(t1);
            glm::dvec3 dt2(t2);

            float ao[4];
            for (int ci = 0; ci < 4; ci++)
            {
                glm::dvec3 rel = corners[static_cast<size_t>(ci)] - quadCenter;
                int        s1  = (glm::dot(rel, dt1) >= 0.0) ? 1 : -1;
                int        s2  = (glm::dot(rel, dt2) >= 0.0) ? 1 : -1;

                bool side1  = isVoxelSolidAt(frontBase + t1 * (s1 * scale), lodDepth);
                bool side2  = isVoxelSolidAt(frontBase + t2 * (s2 * scale), lodDepth);
                bool corner = isVoxelSolidAt(frontBase + t1 * (s1 * scale) + t2 * (s2 * scale), lodDepth);

                int occ = (side1 && side2) ? 3 : ((side1 ? 1 : 0) + (side2 ? 1 : 0) + (corner ? 1 : 0));
                ao[ci]  = 1.0f - static_cast<float>(occ) * 0.17f;
            }

            glm::vec3 c0           = color * ao[0];
            glm::vec3 c1           = color * ao[1];
            glm::vec3 c2           = color * ao[2];
            glm::vec3 c3           = color * ao[3];
            bool      flipDiagonal = (ao[0] + ao[2] < ao[1] + ao[3]);

            if (!flipDiagonal)
            {
                outVertices.push_back({ glm::vec3(corners[0]), normal, c0 });
                outVertices.push_back({ glm::vec3(corners[1]), normal, c1 });
                outVertices.push_back({ glm::vec3(corners[2]), normal, c2 });

                outVertices.push_back({ glm::vec3(corners[0]), normal, c0 });
                outVertices.push_back({ glm::vec3(corners[2]), normal, c2 });
                outVertices.push_back({ glm::vec3(corners[3]), normal, c3 });
            }
            else
            {
                outVertices.push_back({ glm::vec3(corners[1]), normal, c1 });
                outVertices.push_back({ glm::vec3(corners[2]), normal, c2 });
                outVertices.push_back({ glm::vec3(corners[3]), normal, c3 });

                outVertices.push_back({ glm::vec3(corners[1]), normal, c1 });
                outVertices.push_back({ glm::vec3(corners[3]), normal, c3 });
                outVertices.push_back({ glm::vec3(corners[0]), normal, c0 });
            }
        }
    }

    template<typename T>
    void SVOLODMesh<T>::update(const Vulcant::Toolkit::SceneData& sceneData, CADacomb::LODStrategy& strategy)
    {
        auto       frameStart = std::chrono::steady_clock::now();
        glm::dvec3 camPos(sceneData.cameraPos.x, sceneData.cameraPos.y, sceneData.cameraPos.z);

        if (asyncEnabled)
        {
            size_t processedThisFrame = 0;
            while (maxUploadsPerFrame == 0 || processedThisFrame < maxUploadsPerFrame)
            {
                if (maxUploadTimeMs > 0.0 && processedThisFrame > 0)
                {
                    double elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - frameStart).count();
                    if (elapsedMs >= maxUploadTimeMs)
                    {
                        break;
                    }
                }

                SVOCompletedMesh completed;
                {
                    std::lock_guard<std::mutex> lock(completedMutex);
                    if (completedMeshes.empty())
                    {
                        break;
                    }
                    completed = std::move(completedMeshes.front());
                    completedMeshes.pop_front();
                }

                size_t idx = completed.chunkIndex;
                if (idx >= chunks.size() || completed.generation != nodeGeneration[idx])
                {
                    continue;
                }

                if (nodeCollapsing[idx])
                {
                    tryCommitCollapse(idx, completed.vertices, completed.lodLevel);
                    processedThisFrame++;
                    continue;
                }

                int32_t parentIdx = nodeParent[idx];
                if (!nodeActive[idx] && parentIdx >= 0 && nodeSplitting[static_cast<size_t>(parentIdx)])
                {
                    nodeStagedVertices[idx] = std::move(completed.vertices);
                    nodeStagedReady[idx]    = 1;
                    tryCommitSplit(static_cast<size_t>(parentIdx));
                    processedThisFrame++;
                    continue;
                }

                if (!nodeActive[idx])
                {
                    continue;
                }

                bool lodChanged = (completed.lodLevel != chunks[idx].getLOD());
                if (uploadVertices(idx, completed.vertices))
                {
                    chunks[idx].setLOD(completed.lodLevel);
                    if (lodChanged)
                    {
                        markFaceNeighborsDirty(idx);
                    }
                }
                nodeTargetLOD[idx] = chunks[idx].getLOD();
                nodeQueued[idx]    = 0;
                chunks[idx].setDirty(false);
                processedThisFrame++;
            }
        }

        strategy.startFrame(SceneDataConverter::convert(sceneData), volumeBounds, buffer.getCapacity(), svo.getDimension());

        std::vector<size_t> activeLeaves;
        activeLeaves.reserve(chunks.size());
        for (size_t i = 0; i < chunks.size(); i++)
        {
            if (nodeActive[i])
            {
                activeLeaves.push_back(i);
            }
        }

        std::sort(activeLeaves.begin(),
                  activeLeaves.end(),
                  [&](size_t a, size_t b)
                  {
                      glm::dvec3 da = chunks[a].getWorldBounds().getCenter() - camPos;
                      glm::dvec3 db = chunks[b].getWorldBounds().getCenter() - camPos;
                      return glm::dot(da, da) < glm::dot(db, db);
                  });

        std::vector<SVOMeshJob> newJobs;
        std::vector<size_t>  coarsenCandidates;
        std::vector<size_t>  refineCandidates;

        for (size_t i : activeLeaves)
        {
            if (nodeQueued[i] || nodeSplitting[i])
            {
                continue;
            }

            int32_t p = nodeParent[i];
            if (p >= 0 && nodeCollapsing[static_cast<size_t>(p)])
            {
                continue;
            }

            SVOLODChunk&          chunk              = chunks[i];
            double                currentVoxelLength = chunk.getVoxelLength(leafVoxelLength);
            CADacomb::LODDecision decision           = strategy.evaluate(chunk.getWorldBounds(), currentVoxelLength, chunk.getLOD(), buffer.size());

            if (decision == CADacomb::LODDecision::Coarsen)
            {
                coarsenCandidates.push_back(i);
            }
            else if (decision == CADacomb::LODDecision::Refine)
            {
                refineCandidates.push_back(i);
            }
        }

        std::vector<int32_t> collapseParents;
        for (size_t i : coarsenCandidates)
        {
            int maxLod = getMaxLocalLOD(chunks[i].getSize());
            if (chunks[i].getLOD() < maxLod)
            {
                int nextLod      = chunks[i].getLOD() + 1;
                nodeTargetLOD[i] = nextLod;
                nodeGeneration[i]++;
                if (asyncEnabled)
                {
                    nodeQueued[i] = 1;
                    newJobs.push_back(createJobForNode(i, nextLod, camPos));
                }
                else
                {
                    chunks[i].setLOD(nextLod);
                    chunks[i].setDirty(true);
                    markFaceNeighborsDirty(i);
                }
            }
            else if (nodeParent[i] >= 0)
            {
                int32_t p = nodeParent[i];
                if (std::find(collapseParents.begin(), collapseParents.end(), p) == collapseParents.end())
                {
                    collapseParents.push_back(p);
                }
            }
        }

        for (int32_t p : collapseParents)
        {
            size_t  parentIdx  = static_cast<size_t>(p);
            int32_t firstChild = nodeFirstChild[parentIdx];
            if (firstChild < 0 || nodeCollapsing[parentIdx])
            {
                continue;
            }

            bool allCanCollapse = true;
            for (int c = 0; c < 8; c++)
            {
                size_t childIdx = static_cast<size_t>(firstChild + c);
                if (!nodeActive[childIdx] || nodeSplitting[childIdx] || chunks[childIdx].getLOD() < getMaxLocalLOD(chunks[childIdx].getSize()))
                {
                    allCanCollapse = false;
                    break;
                }
            }

            if (allCanCollapse)
            {
                if (asyncEnabled)
                {
                    beginCollapseChildren(parentIdx, camPos, newJobs);
                }
                else
                {
                    for (int c = 0; c < 8; c++)
                    {
                        size_t childIdx = static_cast<size_t>(firstChild + c);
                        releaseChunkPackage(childIdx);
                        nodeActive[childIdx] = 0;
                    }
                    nodeActive[parentIdx] = 1;
                    chunks[parentIdx].setLOD(getMinLocalLOD(chunks[parentIdx].getSize()));
                    chunks[parentIdx].setDirty(true);
                    markFaceNeighborsDirty(parentIdx);
                }
            }
        }

        size_t maxRefinePerFrame = asyncEnabled ? 8u : refineCandidates.size();
        size_t refinedCount      = 0;
        for (size_t i : refineCandidates)
        {
            if (refinedCount >= maxRefinePerFrame)
            {
                break;
            }
            if (!nodeActive[i] || nodeQueued[i] || nodeSplitting[i])
            {
                continue;
            }
            if (buffer.size() >= (buffer.getCapacity() * 92) / 100)
            {
                break;
            }

            int minLod = getMinLocalLOD(chunks[i].getSize());
            if (chunks[i].getLOD() > minLod)
            {
                int nextLod      = chunks[i].getLOD() - 1;
                nodeTargetLOD[i] = nextLod;
                nodeGeneration[i]++;
                if (asyncEnabled)
                {
                    nodeQueued[i] = 1;
                    newJobs.push_back(createJobForNode(i, nextLod, camPos));
                }
                else
                {
                    chunks[i].setLOD(nextLod);
                    chunks[i].setDirty(true);
                    markFaceNeighborsDirty(i);
                }
                refinedCount++;
            }
            else if (chunks[i].getSize() > minChunkSize && !isSVONodeLeaf(chunks[i].getMinBound(), chunks[i].getSize()))
            {
                if (asyncEnabled)
                {
                    beginSubdivideNode(i, camPos, newJobs);
                }
                else
                {
                    ensureChildrenAllocated(i);
                    int32_t  firstChild    = nodeFirstChild[i];
                    uint32_t childSize     = chunks[i].getSize() >> 1;
                    int      childTargetLD = std::max(getMinLocalLOD(childSize), chunks[i].getLOD() - 1);
                    releaseChunkPackage(i);
                    nodeActive[i] = 0;
                    for (int c = 0; c < 8; c++)
                    {
                        size_t childIdx      = static_cast<size_t>(firstChild + c);
                        nodeActive[childIdx] = 1;
                        chunks[childIdx].setLOD(childTargetLD);
                        chunks[childIdx].setDirty(true);
                    }
                    markFaceNeighborsDirty(i);
                }
                refinedCount++;
            }
        }

        if (asyncEnabled)
        {
            for (size_t i : activeLeaves)
            {
                if (!nodeActive[i] || !chunks[i].isDirty() || nodeQueued[i] || nodeSplitting[i])
                {
                    continue;
                }

                nodeQueued[i] = 1;
                newJobs.push_back(createJobForNode(i, chunks[i].getLOD(), camPos));
            }

            if (!newJobs.empty())
            {
                {
                    std::lock_guard<std::mutex> lock(jobMutex);
                    for (auto& job : newJobs)
                    {
                        pendingJobs.push_back(std::move(job));
                    }
                }
                jobCv.notify_all();
            }
        }
        else
        {
            for (size_t i = 0; i < chunks.size(); i++)
            {
                if (!nodeActive[i] || !chunks[i].isDirty())
                {
                    continue;
                }

                std::array<int, 6>                        neighborLODs;
                std::vector<CADacomb::PackedQuad>         quads;
                std::vector<Vulcant::Toolkit::MeshVertex> verts;
                uint32_t                                  coordShift = 0;

                getNeighborLODs(i, neighborLODs);
                CADacomb::SVOCPUMesher::meshChunkWithBoundaries(svo, chunks[i].getMinBound(), chunks[i].getSize(), chunks[i].getLOD(), neighborLODs, quads, coordShift);
                buildQuadVertices(chunks[i].getMinBound(), coordShift, meshColor, ambientOcclusionEnabled && (chunks[i].getLOD() <= 1), quads, verts);
                uploadVertices(i, verts);
                chunks[i].setDirty(false);
            }
        }

        strategy.endFrame();

        if (gpuBufferDirty)
        {
            buffer.syncGPU();
            uint32_t indirectCmd[4] = { static_cast<uint32_t>(buffer.size()), 1u, 0u, 0u };
            indirectBuffer->uploadToGPU(indirectCmd, 1);
            gpuBufferDirty = false;
        }
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
        if (meshColor != color)
        {
            meshColor = color;
            for (size_t i = 0; i < chunks.size(); i++)
            {
                if (nodeActive[i] && chunks[i].hasPackage())
                {
                    markNodeDirty(i);
                }
            }
        }
    }

    template<typename T>
    size_t SVOLODMesh<T>::getMaxUploadsPerFrame() const
    {
        return maxUploadsPerFrame;
    }

    template<typename T>
    void SVOLODMesh<T>::setMaxUploadsPerFrame(size_t maxUploads)
    {
        maxUploadsPerFrame = maxUploads;
    }

    template<typename T>
    double SVOLODMesh<T>::getMaxUploadTimeMs() const
    {
        return maxUploadTimeMs;
    }

    template<typename T>
    void SVOLODMesh<T>::setMaxUploadTimeMs(double maxMs)
    {
        maxUploadTimeMs = maxMs;
    }

    template<typename T>
    bool SVOLODMesh<T>::isAsyncEnabled() const
    {
        return asyncEnabled;
    }

    template<typename T>
    void SVOLODMesh<T>::setAsyncEnabled(bool enabled)
    {
        asyncEnabled = enabled;
    }

    template<typename T>
    bool SVOLODMesh<T>::isAmbientOcclusionEnabled() const
    {
        return ambientOcclusionEnabled;
    }

    template<typename T>
    void SVOLODMesh<T>::setAmbientOcclusionEnabled(bool enabled)
    {
        if (ambientOcclusionEnabled != enabled)
        {
            ambientOcclusionEnabled = enabled;
            markAllDirty();
        }
    }
}
