#include "CADacombRender/SVORaycast/TetraProxy/SVOTetraRaycastRenderer.h"

#include "CADacombRender/SVORaycast/Storage/GPUSVONode.h"
#include "CADacombRender/SVORaycast/TetraProxy/SVOTetraCageBuilder.h"
#include "CADacombRender/SVORaycast/TetraProxy/TetraSineDeformer.h"
#include "SVORaycast/TetraProxy/SVORaycast_Tetra_frag.h"
#include "SVORaycast/TetraProxy/SVORaycast_Tetra_vert.h"
#include "Vulcant/Interface/VulcantBuffer.h"
#include "Vulcant/Interface/VulcantCullMode.h"
#include "Vulcant/Interface/VulcantDevice.h"
#include "Vulcant/Interface/VulcantGraphicCommand.h"
#include "Vulcant/Interface/VulcantGraphicPipeline.h"
#include "Vulcant/Interface/VulcantImage.h"
#include "Vulcant/Interface/VulcantRasterizationConfig.h"
#include "Vulcant/Interface/VulcantResource.h"
#include "Vulcant/Interface/VulcantSet.h"
#include "Vulcant/Interface/VulcantShader.h"

#include <algorithm>
#include <cmath>

namespace CADacomb::Render
{
    SVOTetraRaycastRenderer::SVOTetraRaycastRenderer(Vulcant::VulcantDevice& deviceInput)
      : device(deviceInput)
    {
        initializeResources();
    }

    SVOTetraRaycastRenderer::~SVOTetraRaycastRenderer()
    {
        if (drawCmd)
        {
            drawCmd->wait();
            drawCmd.reset();
        }
        descriptorSet.reset();
        pipeline.reset();
        tetraBuffer.reset();
        tetraVertexBuffer.reset();
        svoNodeBuffer.reset();
        raycastParamsUbo.reset();
        sceneDataUbo.reset();
        indirectBuffer.reset();
        surfaceVertexBuffer.reset();
        fragShader.reset();
        vertShader.reset();
    }

    void SVOTetraRaycastRenderer::initializeResources()
    {
        sceneDataUbo     = device.createUniform(1, sizeof(Vulcant::Toolkit::SceneData));
        raycastParamsUbo = device.createUniform(1, sizeof(SVORaycastParams));
        indirectBuffer   = device.createIndirectBuffer(1, sizeof(uint32_t) * 4);

        auto vertSpirv = std::vector<uint32_t>(SVORaycast_Tetra_vert_spirv, SVORaycast_Tetra_vert_spirv + SVORaycast_Tetra_vert_spirv_sizeInBytes / sizeof(uint32_t));
        auto fragSpirv = std::vector<uint32_t>(SVORaycast_Tetra_frag_spirv, SVORaycast_Tetra_frag_spirv + SVORaycast_Tetra_frag_spirv_sizeInBytes / sizeof(uint32_t));
        vertShader     = device.createShader(vertSpirv);
        fragShader     = device.createShader(fragSpirv);
        drawCmd        = device.createGraphicCommand();
    }

    void SVOTetraRaycastRenderer::uploadSVO(const CADacomb::SVO<bool>& svo, const CADacomb::AABB3& volumeBounds, uint32_t cageGridResolution)
    {
        const auto& rawNodes  = svo.getRaw();
        size_t      nodeCount = rawNodes.size();
        uploadedDimension     = svo.getDimension();

        drawCmd->wait();

        SVOTetraCageBuilder::build(svo, volumeBounds, cageGridResolution, cageVertices, cageTetrahedra, surfaceVertices);

        if (nodeCount == 0 || cageVertices.empty() || cageTetrahedra.empty() || surfaceVertices.empty())
        {
            svoNodeBuffer.reset();
            tetraVertexBuffer.reset();
            tetraBuffer.reset();
            surfaceVertexBuffer.reset();
            descriptorSet.reset();
            return;
        }

        svoNodeBuffer = device.createBuffer(nodeCount, sizeof(GPUSVONode), false);

        constexpr size_t        uploadBatchSize = 65536;
        std::vector<GPUSVONode> stagingBatch;
        stagingBatch.reserve(std::min(nodeCount, uploadBatchSize));

        size_t offset = 0;
        while (offset < nodeCount)
        {
            size_t count = std::min(uploadBatchSize, nodeCount - offset);
            stagingBatch.resize(count);
            for (size_t i = 0; i < count; i++)
            {
                const auto& src        = rawNodes[offset + i];
                uint32_t    materialId = src.value ? 1u : 0u;
                uint32_t    childPtr   = src.childPointer;
                uint32_t    validMask  = 0u;

                if (childPtr != 0xFFFFFFFFu)
                {
                    for (uint32_t c = 0; c < 8u; c++)
                    {
                        const auto& child = rawNodes[static_cast<size_t>(childPtr) + c];
                        if (child.value || child.childPointer != 0xFFFFFFFFu)
                        {
                            validMask |= (1u << c);
                        }
                    }

                    if (validMask == 0u)
                    {
                        childPtr = 0xFFFFFFFFu;
                    }
                }

                stagingBatch[i] = GPUSVONode(childPtr, materialId, validMask);
            }
            svoNodeBuffer->uploadToGPU(stagingBatch.data(), count, offset);
            offset += count;
        }

        tetraVertexBuffer = device.createBuffer(cageVertices.size(), sizeof(GPUTetraVertex), false);
        tetraVertexBuffer->uploadToGPU(cageVertices.data(), cageVertices.size(), 0);

        tetraBuffer = device.createBuffer(cageTetrahedra.size(), sizeof(GPUTetrahedron), false);
        tetraBuffer->uploadToGPU(cageTetrahedra.data(), cageTetrahedra.size(), 0);

        surfaceVertexBuffer = device.createVertexBuffer(surfaceVertices.size(), sizeof(TetraSurfaceVertex), false);
        surfaceVertexBuffer->uploadToGPU(surfaceVertices.data(), surfaceVertices.size(), 0);

        uint32_t indirectCmd[4] = { static_cast<uint32_t>(surfaceVertices.size()), 1u, 0u, 0u };
        indirectBuffer->uploadToGPU(indirectCmd, 1);

        descriptorSet.reset();
    }

    void SVOTetraRaycastRenderer::deformSine(const CADacomb::AABB3& volumeBounds, float timeSeconds, float amplitudeFactor)
    {
        if (!tetraVertexBuffer || cageVertices.empty())
        {
            return;
        }

        TetraSineDeformer::deform(cageVertices, volumeBounds, uploadedDimension, timeSeconds, amplitudeFactor);
        tetraVertexBuffer->uploadToGPU(cageVertices.data(), cageVertices.size(), 0);
    }

    void SVOTetraRaycastRenderer::uploadDeformedVertices(const std::vector<GPUTetraVertex>& verticesInput)
    {
        if (!tetraVertexBuffer || verticesInput.size() != cageVertices.size())
        {
            return;
        }

        cageVertices = verticesInput;
        tetraVertexBuffer->uploadToGPU(cageVertices.data(), cageVertices.size(), 0);
    }

    void SVOTetraRaycastRenderer::invalidatePipeline()
    {
        descriptorSet.reset();
        pipeline.reset();
    }

    int32_t SVOTetraRaycastRenderer::findCameraTetrahedron(const glm::vec3& localCameraPos) const
    {
        for (size_t i = 0; i < cageTetrahedra.size(); i++)
        {
            const glm::uvec4& vIdx = cageTetrahedra[i].vertexIndices;
            glm::vec3         p0   = glm::vec3(cageVertices[vIdx.x].deformedLocalPos);
            glm::vec3         e1   = glm::vec3(cageVertices[vIdx.y].deformedLocalPos) - p0;
            glm::vec3         e2   = glm::vec3(cageVertices[vIdx.z].deformedLocalPos) - p0;
            glm::vec3         e3   = glm::vec3(cageVertices[vIdx.w].deformedLocalPos) - p0;

            glm::vec3 c0  = glm::cross(e2, e3);
            glm::vec3 c1  = glm::cross(e3, e1);
            glm::vec3 c2  = glm::cross(e1, e2);
            float     det = glm::dot(e1, c0);
            if (std::abs(det) < 1e-12f)
            {
                continue;
            }

            float     invDet = 1.0f / det;
            glm::vec3 rel    = localCameraPos - p0;
            float     b1     = glm::dot(rel, c0) * invDet;
            float     b2     = glm::dot(rel, c1) * invDet;
            float     b3     = glm::dot(rel, c2) * invDet;
            float     b0     = 1.0f - b1 - b2 - b3;

            if (b0 >= -1e-4f && b1 >= -1e-4f && b2 >= -1e-4f && b3 >= -1e-4f)
            {
                return static_cast<int32_t>(i);
            }
        }
        return -1;
    }

    void SVOTetraRaycastRenderer::ensurePipeline(Vulcant::VulcantImage& colorTarget, Vulcant::VulcantImage& normalTarget, Vulcant::VulcantImage& depthTarget)
    {
        if (!pipeline)
        {
            std::vector<Vulcant::VulcantShader*> shaders          = { fragShader.get(), vertShader.get() };
            std::vector<Vulcant::VulcantImage*>  colorAttachments = { &colorTarget, &normalTarget };

            Vulcant::VulcantRasterizationConfig rasterConfig;
            rasterConfig.setCullMode(Vulcant::VulcantCullMode::None);
            rasterConfig.setClear(false);

            pipeline = device.createVulcanGraphicPipeline(shaders, colorAttachments, &depthTarget, nullptr, rasterConfig);
        }

        if (!descriptorSet)
        {
            std::vector<std::vector<std::shared_ptr<Vulcant::VulcantResource>>> sets;
            sets.push_back({ sceneDataUbo->asResource(),
                             raycastParamsUbo->asResource(),
                             svoNodeBuffer->asResource(),
                             tetraVertexBuffer->asResource(),
                             tetraBuffer->asResource() });
            descriptorSet = device.createSet(sets, *fragShader);
        }
    }

    void SVOTetraRaycastRenderer::render(Vulcant::VulcantImage&             colorTarget,
                                         Vulcant::VulcantImage&             normalTarget,
                                         Vulcant::VulcantImage&             depthTarget,
                                         const Vulcant::Toolkit::SceneData& worldSceneData,
                                         const CADacomb::AABB3&             volumeBounds,
                                         const glm::dmat4&                  transformation,
                                         const glm::vec3&                   baseColor,
                                         float                              pixelLodScale,
                                         uint32_t                           maxDepth)
    {
        if (!svoNodeBuffer || !tetraVertexBuffer || !tetraBuffer || !surfaceVertexBuffer)
        {
            return;
        }

        glm::ivec2 targetRes = colorTarget.getResolution();
        if (targetRes.x <= 0 || targetRes.y <= 0)
        {
            return;
        }

        if (targetRes != lastResolution || &colorTarget != lastColorTarget)
        {
            lastResolution  = targetRes;
            lastColorTarget = &colorTarget;
            invalidatePipeline();
            drawCmd = device.createGraphicCommand();
        }

        ensurePipeline(colorTarget, normalTarget, depthTarget);

        sceneDataUbo->uploadToGPU(&worldSceneData, 1);

        glm::vec3 worldCamPos = glm::vec3(worldSceneData.invViewMatrix[3]);
        glm::vec3 localCamPos = glm::vec3(glm::inverse(transformation) * glm::dvec4(worldCamPos, 1.0));

        raycastParams.setTransformation(transformation);
        raycastParams.aabbMin        = glm::vec4(glm::vec3(volumeBounds.getMin()), 0.0f);
        raycastParams.aabbSize       = glm::vec4(glm::vec3(volumeBounds.getSize()), 0.0f);
        raycastParams.baseColor      = glm::vec4(baseColor, 1.0f);
        raycastParams.svoDimension   = uploadedDimension;
        raycastParams.maxSteps       = 512u;
        raycastParams.maxDepth       = maxDepth;
        raycastParams.pixelLodScale  = std::max(0.25f, pixelLodScale);
        raycastParams.beamEnabled    = 0u;
        raycastParams.cameraTetIndex = findCameraTetrahedron(localCamPos);
        raycastParamsUbo->uploadToGPU(&raycastParams, 1);

        drawCmd->wait();
        drawCmd->startRecord();
        drawCmd->beginRendering(*pipeline);
        drawCmd->setViewportAndScissor(glm::uvec2(targetRes.x, targetRes.y));
        drawCmd->drawIndirect(*indirectBuffer, descriptorSet.get(), surfaceVertexBuffer.get());
        drawCmd->endRendering();
        drawCmd->endRecord();
        drawCmd->runAsync();
        drawCmd->wait();
    }

    size_t SVOTetraRaycastRenderer::getTetrahedronCount() const
    {
        return cageTetrahedra.size();
    }

    uint32_t SVOTetraRaycastRenderer::getUploadedDimension() const
    {
        return uploadedDimension;
    }
}
