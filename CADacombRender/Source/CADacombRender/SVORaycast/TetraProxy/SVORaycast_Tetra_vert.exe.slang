import SVORaycast.Pipeline.SVORaycastParams;
import SVORaycast.Pipeline.SceneUniform;
import SVORaycast.TetraProxy.GPUTetraVertex;
import SVORaycast.TetraProxy.TetraProxyVertexInput;
import SVORaycast.TetraProxy.TetraProxyVertexOutput;

[[vk::binding(0, 0)]] ConstantBuffer<SceneUniform>     scene;
[[vk::binding(1, 0)]] ConstantBuffer<SVORaycastParams> raycastParams;
[[vk::binding(3, 0)]] StructuredBuffer<GPUTetraVertex> tetraVertices;

[shader("vertex")]
TetraProxyVertexOutput main(TetraProxyVertexInput input)
{
    TetraProxyVertexOutput output;
    float3 localPos = tetraVertices[input.vertexAndTetIndex.x].deformedLocalPos.xyz;
    float4 worldPos = mul(raycastParams.modelMatrix, float4(localPos, 1.0));
    float4 viewPos  = mul(scene.viewMatrix, worldPos);
    output.position   = mul(scene.projectionMatrix, viewPos);
    output.tetAndFace = input.vertexAndTetIndex.y;
    return output;
}
