#pragma once

#include "CADacomb/Animation/Transform.h"

#include <cstddef>
#include <glm/glm.hpp>
#include <vector>

namespace CADacomb::Animation
{
    class Skeleton;

    // Mutable per-joint local pose state that can be driven by clips, procedural solvers, or physics.
    class SkeletonPose
    {
      public:
        SkeletonPose();
        SkeletonPose(size_t jointCount);
        virtual ~SkeletonPose();

        size_t                        getJointCount() const;
        const Transform&              getLocalTransform(size_t jointIndex) const;
        Transform&                    getLocalTransform(size_t jointIndex);
        void                          setLocalTransform(size_t jointIndex, const Transform& transform);
        const std::vector<Transform>& getLocalTransforms() const;

        std::vector<glm::dmat4> computeModelMatrices(const Skeleton& skeleton) const;
        std::vector<glm::dmat4> computeSkinningMatrices(const Skeleton& skeleton) const;

        void setFromModelMatrices(const Skeleton& skeleton, const std::vector<glm::dmat4>& modelMatrices);

        static SkeletonPose blend(const SkeletonPose& from, const SkeletonPose& to, double weight);

      private:
        void resolveModelMatrix(const Skeleton& skeleton, size_t jointIndex, std::vector<glm::dmat4>& modelMatrices, std::vector<bool>& resolved) const;

        std::vector<Transform> localTransforms;
    };
}
