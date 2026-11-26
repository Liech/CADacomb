#include "CADacomb/Animation/KeyframeAnimation/Track/Vec3Track.h"

#include <algorithm>

namespace CADacomb::Animation
{
    Vec3Track::Vec3Track() {}

    Vec3Track::~Vec3Track() {}

    void Vec3Track::addKeyframe(double time, const glm::dvec3& value)
    {
        keyframes.emplace_back(time, value);
    }

    void Vec3Track::sortKeyframes()
    {
        std::sort(keyframes.begin(), keyframes.end(), [](const Vec3Keyframe& a, const Vec3Keyframe& b) { return a.getTime() < b.getTime(); });
    }

    bool Vec3Track::isEmpty() const
    {
        return keyframes.empty();
    }

    double Vec3Track::getDuration() const
    {
        if (keyframes.empty())
        {
            return 0.0;
        }
        return keyframes.back().getTime();
    }

    const std::vector<Vec3Keyframe>& Vec3Track::getKeyframes() const
    {
        return keyframes;
    }

    InterpolationMode Vec3Track::getInterpolationMode() const
    {
        return interpolationMode;
    }

    void Vec3Track::setInterpolationMode(InterpolationMode mode)
    {
        interpolationMode = mode;
    }

    glm::dvec3 Vec3Track::evaluate(double time, const glm::dvec3& fallback) const
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

        auto upper = std::upper_bound(keyframes.begin(), keyframes.end(), time, [](double t, const Vec3Keyframe& kf) { return t < kf.getTime(); });
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

        double factor = (time - prev->getTime()) / span;
        return glm::mix(prev->getValue(), upper->getValue(), factor);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("AnimationVec3Track/LinearAndStepEvaluation")
{
    CADacomb::Animation::Vec3Track track;
    CHECK(track.isEmpty());
    CHECK(track.evaluate(0.5, glm::dvec3(9.0, 8.0, 7.0)) == glm::dvec3(9.0, 8.0, 7.0));

    track.addKeyframe(2.0, glm::dvec3(10.0, 0.0, 0.0));
    track.addKeyframe(0.0, glm::dvec3(0.0, 0.0, 0.0));
    track.sortKeyframes();

    CHECK(track.getDuration() == Catch::Approx(2.0));
    CHECK(track.evaluate(-1.0, glm::dvec3(0.0)).x == Catch::Approx(0.0));
    CHECK(track.evaluate(1.0, glm::dvec3(0.0)).x == Catch::Approx(5.0));
    CHECK(track.evaluate(3.0, glm::dvec3(0.0)).x == Catch::Approx(10.0));

    track.setInterpolationMode(CADacomb::Animation::InterpolationMode::Step);
    CHECK(track.evaluate(1.5, glm::dvec3(0.0)).x == Catch::Approx(0.0));
}
#endif
