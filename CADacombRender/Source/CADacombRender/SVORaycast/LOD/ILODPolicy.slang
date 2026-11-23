module ILODPolicy;

// Abstract level-of-detail termination policy for SVO ray traversal.
public interface ILODPolicy
{
    bool shouldStopAtNode(float tVoxel, float nodeSizeVoxels, uint depth);
};
