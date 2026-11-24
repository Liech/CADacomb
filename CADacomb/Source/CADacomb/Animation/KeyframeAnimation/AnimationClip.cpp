#include "CADacomb/Animation/KeyframeAnimation/AnimationClip.h"

#include "CADacomb/Animation/Skeleton/Skeleton.h"

#include <algorithm>
#include <cmath>

namespace CADacomb::Animation
{
    AnimationClip::AnimationClip() {}

    AnimationClip::AnimationClip(const std::string& nameInput)
      : name(nameInput)
    {
    }

    AnimationClip::~AnimationClip() {}

    const std::string& AnimationClip::getName() const
    {
        return name;
    }

    void AnimationClip::setName(const std::string& nameInput)
    {
        name = nameInput;
    }

    double AnimationClip::getDuration() const
    {
        return duration;
    }

    void AnimationClip::setDuration(double durationInput)
    {
        duration = durationInput;
    }

    void AnimationClip::recomputeDuration()
    {
        double maxDuration = 0.0;
        for (const auto& track : tracks)
        {
            maxDuration = std::max(maxDuration, track.getDuration());
        }
        duration = maxDuration;
    }

    void AnimationClip::addTrack(const JointTrack& track)
    {
        tracks.push_back(track);
        duration = std::max(duration, track.getDuration());
    }

    const std::vector<JointTrack>& AnimationClip::getTracks() const
    {
        return tracks;
    }

    std::vector<JointTrack>& AnimationClip::getTracks()
    {
        return tracks;
    }

    SkeletonPose AnimationClip::evaluate(const Skeleton& skeleton, double time, bool looping) const
    {
        SkeletonPose pose = skeleton.createRestPose();
        evaluateInto(skeleton, time, looping, pose);
        return pose;
    }

    void AnimationClip::evaluateInto(const Skeleton& skeleton, double time, bool looping, SkeletonPose& pose) const
    {
        double sampleTime = resolveSampleTime(time, looping);
        for (const auto& track : tracks)
        {
            int jointIndex = skeleton.findJointIndex(track.getJointName());
            if (jointIndex < 0)
            {
                continue;
            }
            size_t idx = static_cast<size_t>(jointIndex);
            pose.setLocalTransform(idx, track.evaluate(sampleTime, skeleton.getJoint(idx).getRestLocalTransform()));
        }
    }

    double AnimationClip::resolveSampleTime(double time, bool looping) const
    {
        if (duration <= 0.0)
        {
            return 0.0;
        }
        if (!looping)
        {
            return std::clamp(time, 0.0, duration);
        }
        double wrapped = std::fmod(time, duration);
        if (wrapped < 0.0)
        {
            wrapped += duration;
        }
        return wrapped;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("AnimationClip/EvaluateAndLooping")
{
    CADacomb::Animation::Skeleton skeleton;
    int                           rootIdx = skeleton.addJoint(CADacomb::Animation::Joint("Root", -1, CADacomb::Animation::Transform::identity()));
    int childIdx = skeleton.addJoint(CADacomb::Animation::Joint("Child", rootIdx, CADacomb::Animation::Transform(glm::dvec3(0.0, 1.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(1.0))));
    skeleton.computeInverseBindMatrices();

    CADacomb::Animation::JointTrack rootTrack("Root");
    rootTrack.getTranslationTrack().addKeyframe(0.0, glm::dvec3(0.0, 0.0, 0.0));
    rootTrack.getTranslationTrack().addKeyframe(2.0, glm::dvec3(4.0, 0.0, 0.0));

    CADacomb::Animation::AnimationClip clip("Slide");
    clip.addTrack(rootTrack);

    CHECK(clip.getDuration() == Catch::Approx(2.0));

    CADacomb::Animation::SkeletonPose poseMid  = clip.evaluate(skeleton, 1.0, false);
    std::vector<glm::dmat4>           modelMid = poseMid.computeModelMatrices(skeleton);
    CHECK(modelMid[childIdx][3].x == Catch::Approx(2.0));
    CHECK(modelMid[childIdx][3].y == Catch::Approx(1.0));

    CADacomb::Animation::SkeletonPose poseClamped = clip.evaluate(skeleton, 3.0, false);
    CHECK(poseClamped.getLocalTransform(rootIdx).getTranslation().x == Catch::Approx(4.0));

    CADacomb::Animation::SkeletonPose poseLooped = clip.evaluate(skeleton, 2.5, true);
    CHECK(poseLooped.getLocalTransform(rootIdx).getTranslation().x == Catch::Approx(1.0));
}
#endif
