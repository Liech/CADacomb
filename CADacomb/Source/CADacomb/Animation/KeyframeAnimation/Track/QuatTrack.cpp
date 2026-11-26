#include "CADacomb/Animation/KeyframeAnimation/Track/QuatTrack.h"

#include <algorithm>

namespace CADacomb::Animation
{
    QuatTrack::QuatTrack() {}

    QuatTrack::~QuatTrack() {}

    void QuatTrack::addKeyframe(double time, const glm::dquat& value)
    {
        keyframes.emplace_back(time, value);
    }

    void QuatTrack::sortKeyframes()
    {
        std::sort(keyframes.begin(), keyframes.end(), [](const QuatKeyframe& a, const QuatKeyframe& b) { return a.getTime() < b.getTime(); });
    }

    bool QuatTrack::isEmpty() const
    {
        return keyframes.empty();
    }

    double QuatTrack::getDuration() const
    {
        if (keyframes.empty())
        {
            return 0.0;
        }
        return keyframes.back().getTime();
    }

    const std::vector<QuatKeyframe>& QuatTrack::getKeyframes() const
    {
        return keyframes;
    }

    InterpolationMode QuatTrack::getInterpolationMode() const
    {
        return interpolationMode;
    }

    void QuatTrack::setInterpolationMode(InterpolationMode mode)
    {
        interpolationMode = mode;
    }

    glm::dquat QuatTrack::evaluate(double time, const glm::dquat& fallback) const
    {
        if (keyframes.empty())
        {
            return fallback;
        }

        if (keyframes.size() == 1 || time <= keyframes.front().getTime())
        {
            return keyframes.front().getValue();
        }

        if (time >= keyframes.back().getTime())
        {
            return keyframes.back().getValue();
        }

        auto upper = std::upper_bound(keyframes.begin(), keyframes.end(), time, [](double t, const QuatKeyframe& kf) { return t < kf.getTime(); });
        auto prev  = upper - 1;

        if (interpolationMode == InterpolationMode::Step)
        {
            return prev->getValue();
        }

        double span = upper->getTime() - prev->getTime();
        if (span <= 0.0)
        {
            return prev->getValue();
        }

        double     factor = (time - prev->getTime()) / span;
        glm::dquat from   = prev->getValue();
        glm::dquat to     = upper->getValue();
        if (glm::dot(from, to) < 0.0)
        {
            to = -to;
        }
        return glm::normalize(glm::slerp(from, to, factor));
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <glm/gtc/constants.hpp>

TEST_CASE("AnimationQuatTrack/SlerpEvaluation")
{
    CADacomb::Animation::QuatTrack track;
    glm::dquat                     start = glm::dquat(1.0, 0.0, 0.0, 0.0);
    glm::dquat                     end   = glm::angleAxis(glm::half_pi<double>(), glm::dvec3(0.0, 1.0, 0.0));

    track.addKeyframe(0.0, start);
    track.addKeyframe(1.0, end);

    glm::dquat mid      = track.evaluate(0.5, start);
    glm::dquat expected = glm::angleAxis(glm::quarter_pi<double>(), glm::dvec3(0.0, 1.0, 0.0));
    CHECK(std::abs(glm::dot(mid, expected)) == Catch::Approx(1.0));
}
#endif
