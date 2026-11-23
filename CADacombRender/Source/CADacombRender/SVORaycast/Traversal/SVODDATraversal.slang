module SVODDATraversal;

import SVORaycast.Core.Ray;
import SVORaycast.Core.RayHit;
import SVORaycast.LOD.ILODPolicy;
import SVORaycast.Storage.GPUSVONode;
import SVORaycast.Storage.ISVOStorage;

// Stack-based front-to-back octree slab traversal parameterized on storage and LOD policy.
public RayHit traverseSVO<TStorage : ISVOStorage, TLOD : ILODPolicy>(Ray ray, TStorage storage, TLOD lodPolicy, uint maxSteps)
{
    uint dim = storage.getDimension();
    if (dim == 0u || ray.tMax <= ray.tMin)
    {
        return RayHit::miss(0u);
    }

    uint2  stackNodeDepth[48];
    uint4  stackMinSize[48];
    float2 stackRange[48];

    int sp            = 1;
    stackNodeDepth[0] = uint2(0u, 0u);
    stackMinSize[0]   = uint4(0u, 0u, 0u, dim);
    stackRange[0]     = float2(ray.tMin, ray.tMax);

    uint steps = 0u;

    while (sp > 0 && steps < maxSteps)
    {
        sp--;
        steps++;

        uint  nodeIdx  = stackNodeDepth[sp].x;
        uint  depth    = stackNodeDepth[sp].y;
        uint3 nodeMin  = stackMinSize[sp].xyz;
        uint  nodeSize = stackMinSize[sp].w;
        float tEnter   = stackRange[sp].x;
        float tExit    = stackRange[sp].y;

        GPUSVONode node   = storage.getNode(nodeIdx);
        bool       isLeaf = (node.childPointer == 0xFFFFFFFFu) || (nodeSize <= 1u) || lodPolicy.shouldStopAtNode(tEnter, float(nodeSize), depth);

        if (isLeaf)
        {
            if (storage.isSolid(node.value))
            {
                float3 bMin = float3(nodeMin);
                float3 bMax = float3(nodeMin + uint3(nodeSize));
                float3 t0   = (bMin - ray.origin) * ray.invDirection;
                float3 t1   = (bMax - ray.origin) * ray.invDirection;
                float3 tLow = min(t0, t1);

                float3 normal = float3(0.0, 1.0, 0.0);
                if (tLow.x >= tLow.y && tLow.x >= tLow.z)
                {
                    normal = float3(ray.direction.x < 0.0 ? 1.0 : -1.0, 0.0, 0.0);
                }
                else if (tLow.y >= tLow.z)
                {
                    normal = float3(0.0, ray.direction.y < 0.0 ? 1.0 : -1.0, 0.0);
                }
                else
                {
                    normal = float3(0.0, 0.0, ray.direction.z < 0.0 ? 1.0 : -1.0);
                }

                RayHit hit;
                hit.hit           = true;
                hit.t             = tEnter;
                hit.positionLocal = ray.at(tEnter);
                hit.normalLocal   = normal;
                hit.depth         = depth;
                hit.steps         = steps;
                hit.value         = node.value;
                return hit;
            }
            continue;
        }

        uint halfSize = nodeSize >> 1u;

        uint   candChild[4];
        uint3  candMin[4];
        float2 candRange[4];
        int    candCount = 0;

        [unroll]
        for (uint c = 0u; c < 8u; c++)
        {
            uint3 cMin = nodeMin + uint3((c & 1u) != 0u ? halfSize : 0u, (c & 2u) != 0u ? halfSize : 0u, (c & 4u) != 0u ? halfSize : 0u);
            uint3 cMax = cMin + uint3(halfSize);

            float3 t0    = (float3(cMin) - ray.origin) * ray.invDirection;
            float3 t1    = (float3(cMax) - ray.origin) * ray.invDirection;
            float3 tLow  = min(t0, t1);
            float3 tHigh = max(t0, t1);

            float cEnter = max(max(tLow.x, tLow.y), max(tLow.z, tEnter));
            float cExit  = min(min(tHigh.x, tHigh.y), min(tHigh.z, tExit));

            if (cExit > cEnter && candCount < 4)
            {
                candChild[candCount] = node.childPointer + c;
                candMin[candCount]   = cMin;
                candRange[candCount] = float2(cEnter, cExit);
                candCount++;
            }
        }

        for (int i = 0; i < candCount - 1; i++)
        {
            for (int j = i + 1; j < candCount; j++)
            {
                if (candRange[i].x < candRange[j].x)
                {
                    uint   tmpChild = candChild[i];
                    uint3  tmpMin   = candMin[i];
                    float2 tmpRange = candRange[i];

                    candChild[i] = candChild[j];
                    candMin[i]   = candMin[j];
                    candRange[i] = candRange[j];

                    candChild[j] = tmpChild;
                    candMin[j]   = tmpMin;
                    candRange[j] = tmpRange;
                }
            }
        }

        for (int i = 0; i < candCount; i++)
        {
            if (sp < 48)
            {
                stackNodeDepth[sp] = uint2(candChild[i], depth + 1u);
                stackMinSize[sp]   = uint4(candMin[i], halfSize);
                stackRange[sp]     = candRange[i];
                sp++;
            }
        }
    }

    return RayHit::miss(steps);
}
