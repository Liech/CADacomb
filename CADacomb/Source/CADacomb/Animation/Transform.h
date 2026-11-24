#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace CADacomb::Animation
{
    // Local or model-space TRS transform using double precision for animation and skeletal math.
    class Transform
    {
      public:
        Transform();
        Transform(const glm::dvec3& translation, const glm::dquat& rotation, const glm::dvec3& scale);
        virtual ~Transform();

        glm::dmat4 toMatrix() const;

        const glm::dvec3& getTranslation() const;
        void              setTranslation(const glm::dvec3& translation);

        const glm::dquat& getRotation() const;
        void              setRotation(const glm::dquat& rotation);

        const glm::dvec3& getScale() const;
        void              setScale(const glm::dvec3& scale);

        static Transform identity();
        static Transform fromMatrix(const glm::dmat4& matrix);
        static Transform interpolate(const Transform& from, const Transform& to, double factor);

      private:
        glm::dvec3 translation = glm::dvec3(0.0, 0.0, 0.0);
        glm::dquat rotation    = glm::dquat(1.0, 0.0, 0.0, 0.0);
        glm::dvec3 scale       = glm::dvec3(1.0, 1.0, 1.0);
    };
}
