#pragma once

#include "CADacomb/Animation/KeyframeAnimation/InterpolationMode.h"
#include "CADacomb/Animation/KeyframeAnimation/Track/QuatKeyframe.h"

#include <glm/gtc/quaternion.hpp>
#include <vector>

namespace CADacomb::Animation
{
    // Keyframed quaternion rotation track with shortest-path slerp and step sampling.
    class QuatTrack
    {
      public:
        QuatTrack();
        virtual ~QuatTrack();

        void addKeyframe(double time, const glm::dquat& value);
        void sortKeyframes();

        bool                             isEmpty() const;
        double                           getDuration() const;
        const std::vector<QuatKeyframe>& getKeyframes() const;

        InterpolationMode getInterpolationMode() const;
        void              setInterpolationMode(InterpolationMode mode);

        glm::dquat evaluate(double time, const glm::dquat& fallback) const;

      private:
        std::vector<QuatKeyframe> keyframes;
        InterpolationMode         interpolationMode = InterpolationMode::Linear;
    };
}
