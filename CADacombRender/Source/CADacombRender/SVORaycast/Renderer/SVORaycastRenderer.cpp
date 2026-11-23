#include "CADacombRender/SVORaycast/Renderer/SVORaycastRenderer.h"

#include "CADacombRender/SVORaycast/Storage/GPUSVONode.h"
#include "SVORaycast/Renderer/SVORaycast_Default_frag.h"
#include "SVORaycast/Renderer/SVORaycast_vert.h"
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
#include <array>
#include <vector>

namespace CADacomb::Render
{
    SVORaycastRenderer::SVORaycastRenderer(Vulcant::VulcantDevice& deviceInput)
      : device(deviceInput)
    {
        initializeResources();
    }

    SVORaycastRenderer::~SVORaycastRenderer()
    {
        if (drawCmd)
        {
            drawCmd->wait();
            drawCmd.reset();
        }
        descriptorSet.reset();
        pipeline.reset();
        svoNodeBuffer.reset();
        raycastParamsUbo.reset();
        sceneDataUbo.reset();
        indirectBuffer.reset();
        proxyVertexBuffer.reset();
        fragShader.reset();
        vertShader.reset();
    }

    void SVORaycastRenderer::initializeResources()
    {
        sceneDataUbo     = device.createUniform(1, sizeof(Vulcant::Toolkit::SceneData));
        raycastParamsUbo = device.createUniform(1, sizeof(SVORaycastParams));
        indirectBuffer   = device.createIndirectBuffer(1, sizeof(uint32_t) * 4);

        const std::array<glm::vec3, 36> cubeVertices = {
            glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f),

            glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(0.0f, 1.0f, 1.0f),
            glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(1.0f, 0.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f),

            glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 1.0f),
            glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f),

            glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 0.0f, 1.0f),
            glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f),

            glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f),
            glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 1.0f),

            glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f),
            glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 0.0f)
        };

        proxyVertexBuffer = device.createVertexBuffer(cubeVertices.size(), sizeof(glm::vec3), false);
        proxyVertexBuffer->uploadToGPU(cubeVertices.data(), cubeVertices.size(), 0);

        uint32_t indirectCmd[4] = { 36u, 1u, 0u, 0u };
        indirectBuffer->uploadToGPU(indirectCmd, 1);

        auto vertSpirv = std::vector<uint32_t>(SVORaycast_vert_spirv, SVORaycast_vert_spirv + SVORaycast_vert_spirv_sizeInBytes / sizeof(uint32_t));
        auto fragSpirv = std::vector<uint32_t>(SVORaycast_Default_frag_spirv, SVORaycast_Default_frag_spirv + SVORaycast_Default_frag_spirv_sizeInBytes / sizeof(uint32_t));
        vertShader     = device.createShader(vertSpirv);
        fragShader     = device.createShader(fragSpirv);
        drawCmd        = device.createGraphicCommand();
    }

    void SVORaycastRenderer::uploadSVO(const CADacomb::SVO<bool>& svo)
    {
        const auto& rawNodes  = svo.getRaw();
        size_t      nodeCount = rawNodes.size();
        uploadedNodeCount     = nodeCount;
        uploadedDimension     = svo.getDimension();

        if (nodeCount == 0)
        {
            svoNodeBuffer.reset();
            descriptorSet.reset();
            return;
        }

        drawCmd->wait();
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
                const auto& src = rawNodes[offset + i];
                stagingBatch[i] = GPUSVONode(src.childPointer, src.value ? 1u : 0u);
            }
            svoNodeBuffer->uploadToGPU(stagingBatch.data(), count, offset);
            offset += count;
        }

        descriptorSet.reset();
    }

    void SVORaycastRenderer::invalidatePipeline()
    {
        descriptorSet.reset();
        pipeline.reset();
    }

    void SVORaycastRenderer::ensurePipeline(Vulcant::VulcantImage& colorTarget, Vulcant::VulcantImage& normalTarget, Vulcant::VulcantImage& depthTarget)
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
            sets.push_back({ sceneDataUbo->asResource(), raycastParamsUbo->asResource(), svoNodeBuffer->asResource() });
            descriptorSet = device.createSet(sets, *fragShader);
        }
    }

    void SVORaycastRenderer::render(Vulcant::VulcantImage&             colorTarget,
                                    Vulcant::VulcantImage&             normalTarget,
                                    Vulcant::VulcantImage&             depthTarget,
                                    const Vulcant::Toolkit::SceneData& worldSceneData,
                                    const CADacomb::AABB3&             volumeBounds,
                                    const glm::dmat4&                  transformation,
                                    const glm::vec3&                   baseColor,
                                    float                              pixelLodScale,
                                    uint32_t                           maxDepth)
    {
        if (!svoNodeBuffer)
        {
            return;
        }

        glm::ivec2 targetRes = colorTarget.getResolution();
        if (targetRes.x <= 0 || targetRes.y <= 0)
        {
            return;
        }

        if (targetRes != lastResolution)
        {
            lastResolution = targetRes;
            invalidatePipeline();
            drawCmd = device.createGraphicCommand();
        }

        ensurePipeline(colorTarget, normalTarget, depthTarget);

        sceneDataUbo->uploadToGPU(&worldSceneData, 1);

        raycastParams.setTransformation(transformation);
        raycastParams.aabbMin       = glm::vec4(glm::vec3(volumeBounds.getMin()), 0.0f);
        raycastParams.aabbSize      = glm::vec4(glm::vec3(volumeBounds.getSize()), 0.0f);
        raycastParams.baseColor     = glm::vec4(baseColor, 1.0f);
        raycastParams.svoDimension  = uploadedDimension;
        raycastParams.maxSteps      = 512u;
        raycastParams.maxDepth      = maxDepth;
        raycastParams.pixelLodScale = std::max(0.25f, pixelLodScale);
        raycastParamsUbo->uploadToGPU(&raycastParams, 1);

        drawCmd->wait();
        drawCmd->startRecord();
        drawCmd->beginRendering(*pipeline);
        drawCmd->setViewportAndScissor(glm::uvec2(targetRes.x, targetRes.y));
        drawCmd->drawIndirect(*indirectBuffer, descriptorSet.get(), proxyVertexBuffer.get());
        drawCmd->endRendering();
        drawCmd->endRecord();
        drawCmd->runAsync();
        drawCmd->wait();
    }

    size_t SVORaycastRenderer::getUploadedNodeCount() const
    {
        return uploadedNodeCount;
    }

    uint32_t SVORaycastRenderer::getUploadedDimension() const
    {
        return uploadedDimension;
    }
}

#ifdef ISTESTPROJECT
#include "Vulcant/VulcantV/VulcantVDevice.h"
#include <catch2/catch_test_macros.hpp>

namespace CADacomb::Render
{
    TEST_CASE("SVORaycastRenderer/UploadSVO")
    {
        Vulcant::VulcantV::VulcantVDevice device({}, false);
        SVORaycastRenderer                renderer(device);

        CADacomb::SVO<bool> svo(16, false);
        svo.setVoxel({ 4, 4, 4 }, true);
        svo.setVoxel({ 12, 8, 2 }, true);

        renderer.uploadSVO(svo);

        CHECK(renderer.getUploadedDimension() == 16u);
        CHECK(renderer.getUploadedNodeCount() == svo.getRaw().size());
    }
}
#endif
