#pragma once

#include <glm/glm.hpp>

namespace CADacomb::Animation
{
    // Timed 3D vector keyframe used for translation and scale animation tracks.
    class Vec3Keyframe
    {
      public:
        Vec3Keyframe();
        Vec3Keyframe(double time, const glm::dvec3& value);
        virtual ~Vec3Keyframe();

        double getTime() const;
        void   setTime(double time);

        const glm::dvec3& getValue() const;
        void              setValue(const glm::dvec3& value);

      private:
        double     time  = 0.0;
        glm::dvec3 value = glm::dvec3(0.0, 0.0, 0.0);
    };
}
