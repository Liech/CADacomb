#include "CADacomb/Animation/KeyframeAnimation/Track/Vec3Keyframe.h"

namespace CADacomb::Animation
{
    Vec3Keyframe::Vec3Keyframe() {}

    Vec3Keyframe::Vec3Keyframe(double timeInput, const glm::dvec3& valueInput)
      : time(timeInput)
      , value(valueInput)
    {
    }

    Vec3Keyframe::~Vec3Keyframe() {}

    double Vec3Keyframe::getTime() const
    {
        return time;
    }

    void Vec3Keyframe::setTime(double timeInput)
    {
        time = timeInput;
    }

    const glm::dvec3& Vec3Keyframe::getValue() const
    {
        return value;
    }

    void Vec3Keyframe::setValue(const glm::dvec3& valueInput)
    {
        value = valueInput;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("AnimationVec3Keyframe/Properties")
{
    CADacomb::Animation::Vec3Keyframe kf(1.5, glm::dvec3(1.0, 2.0, 3.0));
    CHECK(kf.getTime() == 1.5);
    CHECK(kf.getValue() == glm::dvec3(1.0, 2.0, 3.0));
}
#endif
