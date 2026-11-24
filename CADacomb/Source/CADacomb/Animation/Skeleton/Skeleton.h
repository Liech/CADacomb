#pragma once

#include "CADacomb/Animation/Skeleton/Joint.h"

#include <cstddef>
#include <string>
#include <vector>

namespace CADacomb::Animation
{
    class SkeletonPose;

    // Geometry-agnostic joint hierarchy supporting keyframe, procedural, and physics-driven poses.
    class Skeleton
    {
      public:
        Skeleton();
        virtual ~Skeleton();

        int addJoint(const Joint& joint);

        size_t                    getJointCount() const;
        const Joint&              getJoint(size_t index) const;
        Joint&                    getJoint(size_t index);
        const std::vector<Joint>& getJoints() const;

        int                 findJointIndex(const std::string& name) const;
        std::vector<size_t> getChildIndices(size_t jointIndex) const;

        void         computeInverseBindMatrices();
        SkeletonPose createRestPose() const;

      private:
        std::vector<Joint> joints;
    };
}
