module TetrahedronProxy;

import SVORaycast.Core.Ray;
import SVORaycast.Core.RayHit;
import SVORaycast.LOD.ILODPolicy;
import SVORaycast.Storage.ISVOStorage;
import SVORaycast.TetraProxy.GPUTetraVertex;
import SVORaycast.TetraProxy.GPUTetrahedron;
import SVORaycast.TetraProxy.TetraRaycastResult;
import SVORaycast.Traversal.SVODDATraversal;

// Piecewise-linear affine tetrahedron cage ray proxy supporting arbitrary vertex deformation and skinning.
public struct TetrahedronProxy
{
    public TetraRaycastResult trace<TStorage : ISVOStorage, TLOD : ILODPolicy>(
        float3   worldOrigin,
        float3   worldDirection,
        uint     packedTetAndFace,
        int      cameraTetIndex,
        TStorage storage,
        TLOD     lodPolicy,
        uint     maxStepsPerTet,
        uint     maxTetHops)
    {
        TetraRaycastResult result;
        result.hit            = false;
        result.svoHit         = RayHit::miss(0u);
        result.hitWorldPos    = float3(0.0);
        result.hitNormalWorld = float3(0.0, 1.0, 0.0);

        float3 localOrigin = mul(invModelMatrix, float4(worldOrigin, 1.0)).xyz;
        float3 localDirRaw = mul((float3x3)invModelMatrix, worldDirection);
        float  localDirLen = length(localDirRaw);
        if (localDirLen < 1e-8)
        {
            return result;
        }
        float3 localDir = localDirRaw / localDirLen;

        uint entryTetIndex = packedTetAndFace >> 2u;
        uint entryFaceIdx  = packedTetAndFace & 3u;

        bool isBackFace = false;
        {
            GPUTetrahedron entryTet = tetrahedra[entryTetIndex];
            float3         ep0      = tetraVertices[entryTet.vertexIndices.x].deformedLocalPos.xyz;
            float3         ee1      = tetraVertices[entryTet.vertexIndices.y].deformedLocalPos.xyz - ep0;
            float3         ee2      = tetraVertices[entryTet.vertexIndices.z].deformedLocalPos.xyz - ep0;
            float3         ee3      = tetraVertices[entryTet.vertexIndices.w].deformedLocalPos.xyz - ep0;
            float3         ec0      = cross(ee2, ee3);
            float3         ec1      = cross(ee3, ee1);
            float3         ec2      = cross(ee1, ee2);
            float          edet     = dot(ee1, ec0);
            if (abs(edet) < 1e-12)
            {
                return result;
            }

            float  eInvDet = 1.0 / edet;
            float3 ebD123  = float3(dot(localDir, ec0), dot(localDir, ec1), dot(localDir, ec2)) * eInvDet;

            float ebD[4];
            ebD[0] = -(ebD123.x + ebD123.y + ebD123.z);
            ebD[1] = ebD123.x;
            ebD[2] = ebD123.y;
            ebD[3] = ebD123.z;

            isBackFace = (ebD[entryFaceIdx] <= 0.0);
        }

        if (cameraTetIndex < 0 && isBackFace)
        {
            return result;
        }

        int   currentTet = (cameraTetIndex >= 0 && isBackFace) ? cameraTetIndex : int(entryTetIndex);
        int   prevTet    = -1;
        float tProgress  = 0.0;

        for (uint hop = 0u; hop < maxTetHops && currentTet >= 0; hop++)
        {
            GPUTetrahedron tet = tetrahedra[uint(currentTet)];
            GPUTetraVertex v0  = tetraVertices[tet.vertexIndices.x];
            GPUTetraVertex v1  = tetraVertices[tet.vertexIndices.y];
            GPUTetraVertex v2  = tetraVertices[tet.vertexIndices.z];
            GPUTetraVertex v3  = tetraVertices[tet.vertexIndices.w];

            float3 p0 = v0.deformedLocalPos.xyz;
            float3 e1 = v1.deformedLocalPos.xyz - p0;
            float3 e2 = v2.deformedLocalPos.xyz - p0;
            float3 e3 = v3.deformedLocalPos.xyz - p0;

            float3 c0  = cross(e2, e3);
            float3 c1  = cross(e3, e1);
            float3 c2  = cross(e1, e2);
            float  det = dot(e1, c0);

            if (abs(det) < 1e-12)
            {
                break;
            }

            float  invDet = 1.0 / det;
            float3 relO   = localOrigin - p0;

            float3 bO123 = float3(dot(relO, c0), dot(relO, c1), dot(relO, c2)) * invDet;
            float3 bD123 = float3(dot(localDir, c0), dot(localDir, c1), dot(localDir, c2)) * invDet;

            float bO[4];
            float bD[4];
            bO[0] = 1.0 - bO123.x - bO123.y - bO123.z;
            bO[1] = bO123.x;
            bO[2] = bO123.y;
            bO[3] = bO123.z;

            bD[0] = -(bD123.x + bD123.y + bD123.z);
            bD[1] = bD123.x;
            bD[2] = bD123.y;
            bD[3] = bD123.z;

            float tEnter   = tProgress;
            float tExit    = 1e30;
            int   exitFace = -1;
            bool  valid    = true;

            [unroll]
            for (int k = 0; k < 4; k++)
            {
                if (bD[k] > 1e-8)
                {
                    float tIn = -bO[k] / bD[k];
                    tEnter    = max(tEnter, tIn);
                }
                else if (bD[k] < -1e-8)
                {
                    float tOut = -bO[k] / bD[k];
                    if (tOut < tExit)
                    {
                        tExit    = tOut;
                        exitFace = k;
                    }
                }
                else if (bO[k] < -1e-5)
                {
                    valid = false;
                }
            }

            if (!valid || tExit <= tEnter)
            {
                break;
            }

            float3 r0 = v0.restVoxelPos.xyz;
            float3 r1 = v1.restVoxelPos.xyz - r0;
            float3 r2 = v2.restVoxelPos.xyz - r0;
            float3 r3 = v3.restVoxelPos.xyz - r0;

            float3 voxelOrigin = r0 + r1 * bO123.x + r2 * bO123.y + r3 * bO123.z;
            float3 voxelDirRaw = r1 * bD123.x + r2 * bD123.y + r3 * bD123.z;
            float  voxelScale  = length(voxelDirRaw);

            if (voxelScale > 1e-8)
            {
                float3 voxelDir = voxelDirRaw / voxelScale;
                Ray    voxelRay = Ray::create(voxelOrigin, voxelDir, tEnter * voxelScale, tExit * voxelScale);

                RayHit hit = traverseSVO(voxelRay, storage, lodPolicy, maxStepsPerTet);
                if (hit.hit)
                {
                    float  tHitLocal   = hit.t / voxelScale;
                    float3 hitLocalPos = localOrigin + localDir * tHitLocal;

                    float3 rTn         = float3(dot(r1, hit.normalLocal), dot(r2, hit.normalLocal), dot(r3, hit.normalLocal));
                    float3 normalLocal = (c0 * rTn.x + c1 * rTn.y + c2 * rTn.z) * invDet;

                    result.hit            = true;
                    result.svoHit         = hit;
                    result.hitWorldPos    = mul(modelMatrix, float4(hitLocalPos, 1.0)).xyz;
                    result.hitNormalWorld = normalize(mul((float3x3)normalMatrix, normalLocal));
                    return result;
                }
            }

            if (exitFace < 0)
            {
                break;
            }

            int nextTet = tet.neighborTets[exitFace];
            if (nextTet == prevTet)
            {
                break;
            }

            prevTet    = currentTet;
            currentTet = nextTet;
            tProgress  = tExit;
        }

        return result;
    }

    public float4x4                         modelMatrix;
    public float4x4                         invModelMatrix;
    public float4x4                         normalMatrix;
    public StructuredBuffer<GPUTetraVertex> tetraVertices;
    public StructuredBuffer<GPUTetrahedron> tetrahedra;
};
