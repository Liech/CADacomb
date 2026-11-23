module SVODDATraversal;

import SVORaycast.Core.Ray;
import SVORaycast.Core.RayHit;
import SVORaycast.LOD.ILODPolicy;
import SVORaycast.Storage.GPUSVONode;
import SVORaycast.Storage.ISVOStorage;

// Laine-Karras parametric octree DDA traversal parameterized on storage and LOD policy.
public RayHit traverseSVO<TStorage : ISVOStorage, TLOD : ILODPolicy>(Ray ray, TStorage storage, TLOD lodPolicy, uint maxSteps)
{
    uint dim = storage.getDimension();
    if (dim == 0u || ray.tMax <= ray.tMin)
    {
        return RayHit::miss(0u);
    }

    const int   sMax   = 23;
    const float eps    = 1e-12;
    float       dimF   = float(dim);
    float       invDim = 1.0 / dimF;

    float3 p = ray.origin * invDim + 1.0;
    float3 d = ray.direction;
    if (abs(d.x) < eps)
    {
        d.x = d.x < 0.0 ? -eps : eps;
    }
    if (abs(d.y) < eps)
    {
        d.y = d.y < 0.0 ? -eps : eps;
    }
    if (abs(d.z) < eps)
    {
        d.z = d.z < 0.0 ? -eps : eps;
    }

    float3 tCoef      = 1.0 / -abs(d);
    float3 tBias      = tCoef * p;
    uint   octantMask = 0u;

    if (d.x > 0.0)
    {
        octantMask |= 1u;
        tBias.x = 3.0 * tCoef.x - tBias.x;
    }
    if (d.y > 0.0)
    {
        octantMask |= 2u;
        tBias.y = 3.0 * tCoef.y - tBias.y;
    }
    if (d.z > 0.0)
    {
        octantMask |= 4u;
        tBias.z = 3.0 * tCoef.z - tBias.z;
    }

    float tMin = max(max(2.0 * tCoef.x - tBias.x, 2.0 * tCoef.y - tBias.y), 2.0 * tCoef.z - tBias.z);
    float tMax = min(min(tCoef.x - tBias.x, tCoef.y - tBias.y), tCoef.z - tBias.z);
    tMin       = max(tMin, ray.tMin * invDim);
    tMax       = min(tMax, ray.tMax * invDim);

    if (tMin >= tMax)
    {
        return RayHit::miss(0u);
    }

    uint       parent     = 0u;
    GPUSVONode parentNode = storage.getNode(0u);

    if (parentNode.isLeaf() || dim <= 1u || lodPolicy.shouldStopAtNode(tMin * dimF, dimF, 0u))
    {
        uint matId = parentNode.getMaterialId();
        if (storage.isSolid(matId))
        {
            float3 tEnterPlane = 2.0 * tCoef - tBias;
            float3 normal      = float3(0.0, 1.0, 0.0);
            if (tEnterPlane.x >= tEnterPlane.y && tEnterPlane.x >= tEnterPlane.z)
            {
                normal = float3(ray.direction.x < 0.0 ? 1.0 : -1.0, 0.0, 0.0);
            }
            else if (tEnterPlane.y >= tEnterPlane.z)
            {
                normal = float3(0.0, ray.direction.y < 0.0 ? 1.0 : -1.0, 0.0);
            }
            else
            {
                normal = float3(0.0, 0.0, ray.direction.z < 0.0 ? 1.0 : -1.0);
            }

            RayHit hit;
            hit.hit           = true;
            hit.t             = tMin * dimF;
            hit.positionLocal = ray.at(hit.t);
            hit.normalLocal   = normal;
            hit.depth         = 0u;
            hit.steps         = 1u;
            hit.value         = matId;
            return hit;
        }
        return RayHit::miss(1u);
    }

    uint  stackParent[23];
    float stackTMax[23];

    int    scale     = sMax - 1;
    float  scaleExp2 = 0.5;
    float3 pos       = float3(1.0, 1.0, 1.0);
    uint   idx       = 0u;

    if (1.5 * tCoef.x - tBias.x > tMin)
    {
        idx ^= 1u;
        pos.x = 1.5;
    }
    if (1.5 * tCoef.y - tBias.y > tMin)
    {
        idx ^= 2u;
        pos.y = 1.5;
    }
    if (1.5 * tCoef.z - tBias.z > tMin)
    {
        idx ^= 4u;
        pos.z = 1.5;
    }

    uint steps = 0u;

    while (scale < sMax && steps < maxSteps)
    {
        steps++;

        float3 tCorner = pos * tCoef - tBias;
        float  tcMax   = min(min(tCorner.x, tCorner.y), tCorner.z);

        uint realOctant = idx ^ octantMask;
        uint validMask  = parentNode.getValidMask();

        if ((validMask & (1u << realOctant)) != 0u && tMin <= tMax)
        {
            float tvMax = min(tMax, tcMax);
            if (tMin <= tvMax)
            {
                uint       childIdx  = parentNode.childPointer + realOctant;
                GPUSVONode childNode = storage.getNode(childIdx);
                uint       depth     = uint(sMax - scale);
                float      nodeSize  = dimF * scaleExp2;

                bool stopHere = childNode.isLeaf() || (nodeSize <= 1.0) || lodPolicy.shouldStopAtNode(tMin * dimF, nodeSize, depth);
                if (stopHere)
                {
                    uint matId = childNode.getMaterialId();
                    if (storage.isSolid(matId))
                    {
                        float3 tEnterPlane = (pos + scaleExp2) * tCoef - tBias;
                        float3 normal      = float3(0.0, 1.0, 0.0);
                        if (tEnterPlane.x >= tEnterPlane.y && tEnterPlane.x >= tEnterPlane.z)
                        {
                            normal = float3(ray.direction.x < 0.0 ? 1.0 : -1.0, 0.0, 0.0);
                        }
                        else if (tEnterPlane.y >= tEnterPlane.z)
                        {
                            normal = float3(0.0, ray.direction.y < 0.0 ? 1.0 : -1.0, 0.0);
                        }
                        else
                        {
                            normal = float3(0.0, 0.0, ray.direction.z < 0.0 ? 1.0 : -1.0);
                        }

                        RayHit hit;
                        hit.hit           = true;
                        hit.t             = tMin * dimF;
                        hit.positionLocal = ray.at(hit.t);
                        hit.normalLocal   = normal;
                        hit.depth         = depth;
                        hit.steps         = steps;
                        hit.value         = matId;
                        return hit;
                    }
                }
                else
                {
                    stackParent[scale] = parent;
                    stackTMax[scale]   = tMax;

                    parent     = childIdx;
                    parentNode = childNode;

                    float  halfScale = scaleExp2 * 0.5;
                    float3 tCenter   = halfScale * tCoef + tCorner;

                    idx = 0u;
                    scale--;
                    scaleExp2 = halfScale;

                    if (tCenter.x > tMin)
                    {
                        idx ^= 1u;
                        pos.x += scaleExp2;
                    }
                    if (tCenter.y > tMin)
                    {
                        idx ^= 2u;
                        pos.y += scaleExp2;
                    }
                    if (tCenter.z > tMin)
                    {
                        idx ^= 4u;
                        pos.z += scaleExp2;
                    }

                    tMax = tvMax;
                    continue;
                }
            }
        }

        uint stepMask = 0u;
        if (tCorner.x <= tcMax)
        {
            stepMask ^= 1u;
            pos.x -= scaleExp2;
        }
        if (tCorner.y <= tcMax)
        {
            stepMask ^= 2u;
            pos.y -= scaleExp2;
        }
        if (tCorner.z <= tcMax)
        {
            stepMask ^= 4u;
            pos.z -= scaleExp2;
        }

        tMin = tcMax;
        idx ^= stepMask;

        if ((idx & stepMask) != 0u)
        {
            uint differingBits = 0u;
            if ((stepMask & 1u) != 0u)
            {
                differingBits |= asuint(pos.x) ^ asuint(pos.x + scaleExp2);
            }
            if ((stepMask & 2u) != 0u)
            {
                differingBits |= asuint(pos.y) ^ asuint(pos.y + scaleExp2);
            }
            if ((stepMask & 4u) != 0u)
            {
                differingBits |= asuint(pos.z) ^ asuint(pos.z + scaleExp2);
            }

            scale     = int(firstbithigh(differingBits));
            scaleExp2 = asfloat((scale - sMax + 127) << 23);

            if (scale >= sMax)
            {
                break;
            }

            parent     = stackParent[scale];
            tMax       = stackTMax[scale];
            parentNode = storage.getNode(parent);

            uint shx = asuint(pos.x) >> uint(scale);
            uint shy = asuint(pos.y) >> uint(scale);
            uint shz = asuint(pos.z) >> uint(scale);
            pos.x    = asfloat(shx << uint(scale));
            pos.y    = asfloat(shy << uint(scale));
            pos.z    = asfloat(shz << uint(scale));
            idx      = (shx & 1u) | ((shy & 1u) << 1u) | ((shz & 1u) << 2u);
        }
    }

    return RayHit::miss(steps);
}
