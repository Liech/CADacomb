#include "CADacomb/Animation/KeyframeAnimation/Track/QuatKeyframe.h"

namespace CADacomb::Animation
{
    QuatKeyframe::QuatKeyframe() {}

    QuatKeyframe::QuatKeyframe(double timeInput, const glm::dquat& valueInput)
      : time(timeInput)
      , value(glm::normalize(valueInput))
    {
    }

    QuatKeyframe::~QuatKeyframe() {}

    double QuatKeyframe::getTime() const
    {
        return time;
    }

    void QuatKeyframe::setTime(double timeInput)
    {
        time = timeInput;
    }

    const glm::dquat& QuatKeyframe::getValue() const
    {
        return value;
    }

    void QuatKeyframe::setValue(const glm::dquat& valueInput)
    {
        value = glm::normalize(valueInput);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <glm/gtc/constants.hpp>

TEST_CASE("AnimationQuatKeyframe/Properties")
{
    glm::dquat                        rot = glm::angleAxis(glm::half_pi<double>(), glm::dvec3(0.0, 1.0, 0.0));
    CADacomb::Animation::QuatKeyframe kf(0.5, rot);
    CHECK(kf.getTime() == 0.5);
    CHECK(std::abs(glm::dot(kf.getValue(), rot)) == Catch::Approx(1.0));
}
#endif
