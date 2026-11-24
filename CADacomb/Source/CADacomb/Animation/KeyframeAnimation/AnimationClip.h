#pragma once

#include "CADacomb/Animation/KeyframeAnimation/Track/JointTrack.h"
#include "CADacomb/Animation/Skeleton/SkeletonPose.h"

#include <string>
#include <vector>

namespace CADacomb::Animation
{
    class Skeleton;

    // Decoupled keyframe animation clip targeting joints by name and evaluating into a SkeletonPose.
    class AnimationClip
    {
      public:
        AnimationClip();
        AnimationClip(const std::string& name);
        virtual ~AnimationClip();

        const std::string& getName() const;
        void               setName(const std::string& name);

        double getDuration() const;
        void   setDuration(double duration);
        void   recomputeDuration();

        void                           addTrack(const JointTrack& track);
        const std::vector<JointTrack>& getTracks() const;
        std::vector<JointTrack>&       getTracks();

        SkeletonPose evaluate(const Skeleton& skeleton, double time, bool looping) const;
        void         evaluateInto(const Skeleton& skeleton, double time, bool looping, SkeletonPose& pose) const;

      private:
        double resolveSampleTime(double time, bool looping) const;

        std::string             name;
        double                  duration = 0.0;
        std::vector<JointTrack> tracks;
    };
}
