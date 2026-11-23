module SceneUniform;

// GPU uniform layout matching Vulcant::Toolkit::SceneData.
public struct SceneUniform
{
    public float4x4 viewMatrix;
    public float4x4 projectionMatrix;
    public float4x4 invProjectionMatrix;
    public float4x4 invViewMatrix;
    public float4   cameraPos;
    public float2   resolution;
    public float    nearPlane;
    public float    farPlane;
};
