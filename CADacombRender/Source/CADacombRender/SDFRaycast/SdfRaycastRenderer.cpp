#include "CADacombRender/SDFRaycast/SdfRaycastRenderer.h"

#include "CADacombRender/Formula/FormulaSlangEmitter.h"
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

#include <array>
#include <exception>
#include <sstream>
#include <vector>

namespace CADacomb::Render
{
    SdfRaycastRenderer::SdfRaycastRenderer(Vulcant::VulcantDevice& deviceInput)
      : device(deviceInput)
    {
        initializeResources();
    }

    SdfRaycastRenderer::~SdfRaycastRenderer()
    {
        if (drawCmd)
        {
            drawCmd->wait();
            drawCmd.reset();
        }
        descriptorSet.reset();
        pipeline.reset();
        raycastParamsUbo.reset();
        sceneDataUbo.reset();
        indirectBuffer.reset();
        proxyVertexBuffer.reset();
        fragShader.reset();
        vertShader.reset();
    }

    std::string SdfRaycastRenderer::buildVertexShaderSource()
    {
        return R"SLANG(
struct SceneUniform
{
    float4x4 viewMatrix;
    float4x4 projectionMatrix;
    float4x4 invProjectionMatrix;
    float4x4 invViewMatrix;
    float4   cameraPos;
    float2   resolution;
    float    nearPlane;
    float    farPlane;
};

struct SdfRaycastParams
{
    float4x4 modelMatrix;
    float4x4 invModelMatrix;
    float4x4 normalMatrix;
    float4   aabbMin;
    float4   aabbSize;
    float4   baseColor;
    uint     maxSteps;
    float    hitEpsilon;
    float    stepScale;
    float    normalEpsilon;
    float    ambientStrength;
    float    pad0;
    float    pad1;
    float    pad2;
};

struct VertexInput
{
    float3 unitPosition : POSITION;
};

struct VertexOutput
{
    float4 position : SV_Position;
};

[[vk::binding(0, 0)]] ConstantBuffer<SceneUniform>     scene;
[[vk::binding(1, 0)]] ConstantBuffer<SdfRaycastParams> raycastParams;

[shader("vertex")]
VertexOutput main(VertexInput input)
{
    VertexOutput output;
    float3 localPos = raycastParams.aabbMin.xyz + input.unitPosition * raycastParams.aabbSize.xyz;
    float4 worldPos = mul(raycastParams.modelMatrix, float4(localPos, 1.0));
    float4 viewPos  = mul(scene.viewMatrix, worldPos);
    output.position = mul(scene.projectionMatrix, viewPos);
    return output;
}
)SLANG";
    }

    std::string SdfRaycastRenderer::buildFragmentShaderSource(const CADacomb::Sdf& sdf)
    {
        std::ostringstream out;
        out << FormulaSlangEmitter::emit(sdf.getFormula()) << "\n";
        out << R"SLANG(
struct SceneUniform
{
    float4x4 viewMatrix;
    float4x4 projectionMatrix;
    float4x4 invProjectionMatrix;
    float4x4 invViewMatrix;
    float4   cameraPos;
    float2   resolution;
    float    nearPlane;
    float    farPlane;
};

struct SdfRaycastParams
{
    float4x4 modelMatrix;
    float4x4 invModelMatrix;
    float4x4 normalMatrix;
    float4   aabbMin;
    float4   aabbSize;
    float4   baseColor;
    uint     maxSteps;
    float    hitEpsilon;
    float    stepScale;
    float    normalEpsilon;
    float    ambientStrength;
    float    pad0;
    float    pad1;
    float    pad2;
};

struct FragmentInput
{
    float4 position : SV_Position;
};

struct FragmentOutput
{
    float4 color  : SV_Target0;
    float4 normal : SV_Target1;
    float  depth  : SV_Depth;
};

[[vk::binding(0, 0)]] ConstantBuffer<SceneUniform>     scene;
[[vk::binding(1, 0)]] ConstantBuffer<SdfRaycastParams> raycastParams;

float sampleSdf(float3 p)
{
    float inp[3] = { p.x, p.y, p.z };
    float outp[1];
)SLANG";
        out << "    " << sdf.getFormula().getName() << "(inp, outp);\n";
        out << R"SLANG(    return outp[0];
}

float3 estimateNormal(float3 p, float eps)
{
    const float2 k = float2(1.0, -1.0);
    return normalize(
        k.xyy * sampleSdf(p + k.xyy * eps) +
        k.yyx * sampleSdf(p + k.yyx * eps) +
        k.yxy * sampleSdf(p + k.yxy * eps) +
        k.xxx * sampleSdf(p + k.xxx * eps));
}

bool intersectAABB(float3 ro, float3 rd, float3 bmin, float3 bmax, out float tNear, out float tFar)
{
    float3 invDir = 1.0 / rd;
    float3 t0 = (bmin - ro) * invDir;
    float3 t1 = (bmax - ro) * invDir;
    float3 tsmaller = min(t0, t1);
    float3 tbigger  = max(t0, t1);
    tNear = max(max(tsmaller.x, tsmaller.y), tsmaller.z);
    tFar  = min(min(tbigger.x, tbigger.y), tbigger.z);
    return tFar >= max(tNear, 0.0);
}

[shader("fragment")]
FragmentOutput main(FragmentInput input)
{
    float2 ndc = (input.position.xy / max(scene.resolution, float2(1.0))) * 2.0 - 1.0;

    float4 nearView = mul(scene.invProjectionMatrix, float4(ndc, 0.0, 1.0));
    nearView /= nearView.w;
    float4 farView = mul(scene.invProjectionMatrix, float4(ndc, 1.0, 1.0));
    farView /= farView.w;

    float3 worldOrigin = mul(scene.invViewMatrix, float4(nearView.xyz, 1.0)).xyz;
    float3 worldFar    = mul(scene.invViewMatrix, float4(farView.xyz, 1.0)).xyz;
    float3 worldDir    = normalize(worldFar - worldOrigin);

    float3 localOrigin = mul(raycastParams.invModelMatrix, float4(worldOrigin, 1.0)).xyz;
    float3 localFar    = mul(raycastParams.invModelMatrix, float4(worldOrigin + worldDir, 1.0)).xyz;
    float3 localDir    = normalize(localFar - localOrigin);

    float3 bmin = raycastParams.aabbMin.xyz;
    float3 bmax = bmin + raycastParams.aabbSize.xyz;

    float tNear;
    float tFar;
    if (!intersectAABB(localOrigin, localDir, bmin, bmax, tNear, tFar))
    {
        discard;
    }

    float  t        = max(tNear, 0.0);
    bool   hit      = false;
    float3 hitLocal = float3(0.0);

    for (uint step = 0; step < raycastParams.maxSteps && t <= tFar; step++)
    {
        float3 p = localOrigin + localDir * t;
        float  d = sampleSdf(p);
        if (d <= raycastParams.hitEpsilon)
        {
            hit      = true;
            hitLocal = p;
            break;
        }
        t += max(d * raycastParams.stepScale, raycastParams.hitEpsilon);
    }

    if (!hit)
    {
        discard;
    }

    float3 normalLocal = estimateNormal(hitLocal, raycastParams.normalEpsilon);
    float3 hitWorld    = mul(raycastParams.modelMatrix, float4(hitLocal, 1.0)).xyz;
    float3 normalWorld = normalize(mul((float3x3)raycastParams.normalMatrix, normalLocal));
    float3 normalView  = normalize(mul((float3x3)scene.viewMatrix, normalWorld));

    float3 lightDir = normalize(float3(0.4, 0.7, 0.6));
    float  ndotl    = max(dot(normalView, lightDir), 0.0);
    float  ambient  = clamp(raycastParams.ambientStrength, 0.0, 1.0);
    float3 color    = raycastParams.baseColor.rgb * (ambient + (1.0 - ambient) * ndotl);

    float4 clipPos = mul(scene.projectionMatrix, mul(scene.viewMatrix, float4(hitWorld, 1.0)));

    FragmentOutput output;
    output.color  = float4(color, 1.0);
    output.normal = float4(normalView * 0.5 + 0.5, 1.0);
    output.depth  = clipPos.z / clipPos.w;
    return output;
}
)SLANG";
        return out.str();
    }

    void SdfRaycastRenderer::initializeResources()
    {
        sceneDataUbo     = device.createUniform(1, sizeof(Vulcant::Toolkit::SceneData));
        raycastParamsUbo = device.createUniform(1, sizeof(SdfRaycastParams));
        indirectBuffer   = device.createIndirectBuffer(1, sizeof(uint32_t) * 4);

        const std::array<glm::vec3, 36> cubeVertices = {
            glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f),

            glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(0.0f, 1.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(1.0f, 0.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f),

            glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 1.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f),

            glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 0.0f, 1.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f),

            glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 1.0f),

            glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(1.0f, 1.0f, 0.0f)
        };

        proxyVertexBuffer = device.createVertexBuffer(cubeVertices.size(), sizeof(glm::vec3), false);
        proxyVertexBuffer->uploadToGPU(cubeVertices.data(), cubeVertices.size(), 0);

        uint32_t indirectCmd[4] = { 36u, 1u, 0u, 0u };
        indirectBuffer->uploadToGPU(indirectCmd, 1);

        vertShader = device.createSlangShader(buildVertexShaderSource());
        drawCmd    = device.createGraphicCommand();
    }

    bool SdfRaycastRenderer::setSdf(const CADacomb::Sdf& sdf, std::string& outError)
    {
        outError.clear();
        try
        {
            std::string fragSource = buildFragmentShaderSource(sdf);
            auto        compiled   = device.createSlangShader(fragSource);
            if (!compiled)
            {
                outError = "Failed to compile SDF Slang fragment shader.";
                return false;
            }

            if (drawCmd)
            {
                drawCmd->wait();
            }
            invalidatePipeline();
            fragShader = std::move(compiled);
            return true;
        }
        catch (const std::exception& e)
        {
            outError = e.what();
            return false;
        }
    }

    bool SdfRaycastRenderer::isReady() const
    {
        return vertShader != nullptr && fragShader != nullptr;
    }

    void SdfRaycastRenderer::invalidatePipeline()
    {
        descriptorSet.reset();
        pipeline.reset();
    }

    void SdfRaycastRenderer::ensurePipeline(Vulcant::VulcantImage& colorTarget, Vulcant::VulcantImage& normalTarget, Vulcant::VulcantImage& depthTarget)
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
            sets.push_back({ sceneDataUbo->asResource(), raycastParamsUbo->asResource() });
            descriptorSet = device.createSet(sets, *fragShader);
        }
    }

    void SdfRaycastRenderer::render(Vulcant::VulcantImage&             colorTarget,
                                    Vulcant::VulcantImage&             normalTarget,
                                    Vulcant::VulcantImage&             depthTarget,
                                    const Vulcant::Toolkit::SceneData& worldSceneData,
                                    const CADacomb::AABB3&             volumeBounds,
                                    const glm::dmat4&                  transformation,
                                    const glm::vec3&                   baseColor,
                                    uint32_t                           maxSteps,
                                    float                              hitEpsilon,
                                    float                              stepScale,
                                    float                              normalEpsilon,
                                    float                              ambientStrength)
    {
        if (!isReady())
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

        raycastParams.setTransformation(transformation);
        raycastParams.aabbMin         = glm::vec4(glm::vec3(volumeBounds.getMin()), 0.0f);
        raycastParams.aabbSize        = glm::vec4(glm::vec3(volumeBounds.getSize()), 0.0f);
        raycastParams.baseColor       = glm::vec4(baseColor, 1.0f);
        raycastParams.maxSteps        = maxSteps;
        raycastParams.hitEpsilon      = hitEpsilon;
        raycastParams.stepScale       = stepScale;
        raycastParams.normalEpsilon   = normalEpsilon;
        raycastParams.ambientStrength = ambientStrength;
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
}
