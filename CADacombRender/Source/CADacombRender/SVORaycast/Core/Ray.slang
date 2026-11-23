module Ray;

// Ray representation for SVO raycasting.
public struct Ray
{
    public static Ray create(float3 originInput, float3 directionInput, float tMinInput, float tMaxInput)
    {
        Ray ray;
        ray.origin    = originInput;
        ray.direction = directionInput;

        float3 safeDir = float3(
            abs(directionInput.x) < 1e-8 ? (directionInput.x < 0.0 ? -1e-8 : 1e-8) : directionInput.x,
            abs(directionInput.y) < 1e-8 ? (directionInput.y < 0.0 ? -1e-8 : 1e-8) : directionInput.y,
            abs(directionInput.z) < 1e-8 ? (directionInput.z < 0.0 ? -1e-8 : 1e-8) : directionInput.z
        );
        ray.invDirection = 1.0 / safeDir;
        ray.tMin         = tMinInput;
        ray.tMax         = tMaxInput;
        return ray;
    }

    public float3 at(float t)
    {
        return origin + direction * t;
    }

    public float3 origin;
    public float3 direction;
    public float3 invDirection;
    public float  tMin;
    public float  tMax;
};
