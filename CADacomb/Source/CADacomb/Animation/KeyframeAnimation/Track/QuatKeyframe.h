#pragma once

#include <glm/gtc/quaternion.hpp>

namespace CADacomb::Animation
{
    // Timed quaternion keyframe used for rotation animation tracks.
    class QuatKeyframe
    {
      public:
        QuatKeyframe();
        QuatKeyframe(double time, const glm::dquat& value);
        virtual ~QuatKeyframe();

        double getTime() const;
        void   setTime(double time);

        const glm::dquat& getValue() const;
        void              setValue(const glm::dquat& value);

      private:
        double     time  = 0.0;
        glm::dquat value = glm::dquat(1.0, 0.0, 0.0, 0.0);
    };
}
