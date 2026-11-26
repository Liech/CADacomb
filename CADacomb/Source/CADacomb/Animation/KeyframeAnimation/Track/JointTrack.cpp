#include "CADacomb/Animation/KeyframeAnimation/Track/JointTrack.h"

#include <algorithm>

namespace CADacomb::Animation
{
    JointTrack::JointTrack() {}

    JointTrack::JointTrack(const std::string& jointNameInput)
      : jointName(jointNameInput)
    {
    }

    JointTrack::~JointTrack() {}

    const std::string& JointTrack::getJointName() const
    {
        return jointName;
    }

    void JointTrack::setJointName(const std::string& jointNameInput)
    {
        jointName = jointNameInput;
    }

    Vec3Track& JointTrack::getTranslationTrack()
    {
        return translationTrack;
    }

    const Vec3Track& JointTrack::getTranslationTrack() const
    {
        return translationTrack;
    }

    QuatTrack& JointTrack::getRotationTrack()
    {
        return rotationTrack;
    }

    const QuatTrack& JointTrack::getRotationTrack() const
    {
        return rotationTrack;
    }

    Vec3Track& JointTrack::getScaleTrack()
    {
        return scaleTrack;
    }

    const Vec3Track& JointTrack::getScaleTrack() const
    {
        return scaleTrack;
    }

    double JointTrack::getDuration() const
    {
        return std::max({ translationTrack.getDuration(), rotationTrack.getDuration(), scaleTrack.getDuration() });
    }

    Transform JointTrack::evaluate(double time, const Transform& restTransform) const
    {
        glm::dvec3 translation = translationTrack.evaluate(time, restTransform.getTranslation());
        glm::dquat rotation    = rotationTrack.evaluate(time, restTransform.getRotation());
        glm::dvec3 scale       = scaleTrack.evaluate(time, restTransform.getScale());
        return Transform(translation, rotation, scale);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("AnimationJointTrack/EvaluatesWithRestFallback")
{
    CADacomb::Animation::JointTrack track("Arm");
    track.getTranslationTrack().addKeyframe(0.0, glm::dvec3(0.0, 0.0, 0.0));
    track.getTranslationTrack().addKeyframe(2.0, glm::dvec3(0.0, 4.0, 0.0));

    CADacomb::Animation::Transform rest(glm::dvec3(1.0, 1.0, 1.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(2.0, 2.0, 2.0));
    CADacomb::Animation::Transform sampled = track.evaluate(1.0, rest);

    CHECK(track.getDuration() == Catch::Approx(2.0));
    CHECK(sampled.getTranslation().y == Catch::Approx(2.0));
    CHECK(sampled.getScale() == glm::dvec3(2.0, 2.0, 2.0));
}
#endif
