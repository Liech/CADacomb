#pragma once

#include "CADacomb/Animation/KeyframeAnimation/Track/QuatTrack.h"
#include "CADacomb/Animation/KeyframeAnimation/Track/Vec3Track.h"
#include "CADacomb/Animation/Transform.h"

#include <string>

namespace CADacomb::Animation
{
    // Named per-joint animation channel combining optional translation, rotation, and scale tracks.
    class JointTrack
    {
      public:
        JointTrack();
        JointTrack(const std::string& jointName);
        virtual ~JointTrack();

        const std::string& getJointName() const;
        void               setJointName(const std::string& jointName);

        Vec3Track&       getTranslationTrack();
        const Vec3Track& getTranslationTrack() const;

        QuatTrack&       getRotationTrack();
        const QuatTrack& getRotationTrack() const;

        Vec3Track&       getScaleTrack();
        const Vec3Track& getScaleTrack() const;

        double    getDuration() const;
        Transform evaluate(double time, const Transform& restTransform) const;

      private:
        std::string jointName;
        Vec3Track   translationTrack;
        QuatTrack   rotationTrack;
        Vec3Track   scaleTrack;
    };
}
