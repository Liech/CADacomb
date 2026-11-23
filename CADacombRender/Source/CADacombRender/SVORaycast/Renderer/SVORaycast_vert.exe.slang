import SVORaycast.Pipeline.ProxyVertexInput;
import SVORaycast.Pipeline.ProxyVertexOutput;
import SVORaycast.Pipeline.SVORaycastPipeline;

[shader("vertex")]
ProxyVertexOutput main(ProxyVertexInput input)
{
    return buildProxyVertex(input);
}
