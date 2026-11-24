#pragma once

#include "CADacomb/Animation/KeyframeAnimation/InterpolationMode.h"
#include "CADacomb/Animation/KeyframeAnimation/Track/Vec3Keyframe.h"

#include <glm/glm.hpp>
#include <vector>

namespace CADacomb::Animation
{
    // Keyframed 3D vector track with time-based sampling and interpolation.
    class Vec3Track
    {
      public:
        Vec3Track();
        virtual ~Vec3Track();

        void addKeyframe(double time, const glm::dvec3& value);
        void sortKeyframes();

        bool                             isEmpty() const;
        double                           getDuration() const;
        const std::vector<Vec3Keyframe>& getKeyframes() const;

        InterpolationMode getInterpolationMode() const;
        void              setInterpolationMode(InterpolationMode mode);

        glm::dvec3 evaluate(double time, const glm::dvec3& fallback) const;

      private:
        std::vector<Vec3Keyframe> keyframes;
        InterpolationMode         interpolationMode = InterpolationMode::Linear;
    };
}
