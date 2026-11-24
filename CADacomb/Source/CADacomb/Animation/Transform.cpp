#include "CADacomb/Animation/Transform.h"

#include <glm/gtc/matrix_transform.hpp>

namespace CADacomb::Animation
{
    Transform::Transform() {}

    Transform::Transform(const glm::dvec3& translationInput, const glm::dquat& rotationInput, const glm::dvec3& scaleInput)
      : translation(translationInput)
      , rotation(glm::normalize(rotationInput))
      , scale(scaleInput)
    {
    }

    Transform::~Transform() {}

    glm::dmat4 Transform::toMatrix() const
    {
        glm::dmat4 translationMatrix = glm::translate(glm::dmat4(1.0), translation);
        glm::dmat4 rotationMatrix    = glm::mat4_cast(glm::normalize(rotation));
        glm::dmat4 scaleMatrix       = glm::scale(glm::dmat4(1.0), scale);
        return translationMatrix * rotationMatrix * scaleMatrix;
    }

    const glm::dvec3& Transform::getTranslation() const
    {
        return translation;
    }

    void Transform::setTranslation(const glm::dvec3& translationInput)
    {
        translation = translationInput;
    }

    const glm::dquat& Transform::getRotation() const
    {
        return rotation;
    }

    void Transform::setRotation(const glm::dquat& rotationInput)
    {
        rotation = glm::normalize(rotationInput);
    }

    const glm::dvec3& Transform::getScale() const
    {
        return scale;
    }

    void Transform::setScale(const glm::dvec3& scaleInput)
    {
        scale = scaleInput;
    }

    Transform Transform::identity()
    {
        return Transform();
    }

    Transform Transform::fromMatrix(const glm::dmat4& matrix)
    {
        glm::dvec3 extractedTranslation(matrix[3].x, matrix[3].y, matrix[3].z);

        glm::dvec3 col0(matrix[0].x, matrix[0].y, matrix[0].z);
        glm::dvec3 col1(matrix[1].x, matrix[1].y, matrix[1].z);
        glm::dvec3 col2(matrix[2].x, matrix[2].y, matrix[2].z);

        double scaleX = glm::length(col0);
        double scaleY = glm::length(col1);
        double scaleZ = glm::length(col2);

        glm::dmat3 rotationMatrix(1.0);
        if (scaleX > 0.0)
        {
            rotationMatrix[0] = col0 / scaleX;
        }
        if (scaleY > 0.0)
        {
            rotationMatrix[1] = col1 / scaleY;
        }
        if (scaleZ > 0.0)
        {
            rotationMatrix[2] = col2 / scaleZ;
        }

        if (glm::determinant(rotationMatrix) < 0.0)
        {
            scaleX            = -scaleX;
            rotationMatrix[0] = -rotationMatrix[0];
        }

        glm::dquat extractedRotation = glm::normalize(glm::quat_cast(rotationMatrix));
        return Transform(extractedTranslation, extractedRotation, glm::dvec3(scaleX, scaleY, scaleZ));
    }

    Transform Transform::interpolate(const Transform& from, const Transform& to, double factor)
    {
        glm::dvec3 blendedTranslation = glm::mix(from.translation, to.translation, factor);
        glm::dquat targetRotation     = to.rotation;
        if (glm::dot(from.rotation, targetRotation) < 0.0)
        {
            targetRotation = -targetRotation;
        }
        glm::dquat blendedRotation = glm::normalize(glm::slerp(from.rotation, targetRotation, factor));
        glm::dvec3 blendedScale    = glm::mix(from.scale, to.scale, factor);
        return Transform(blendedTranslation, blendedRotation, blendedScale);
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <glm/gtc/constants.hpp>

TEST_CASE("AnimationTransform/MatrixRoundTrip")
{
    glm::dvec3                     translation(1.5, -2.0, 3.25);
    glm::dquat                     rotation = glm::angleAxis(glm::half_pi<double>(), glm::dvec3(0.0, 1.0, 0.0));
    glm::dvec3                     scale(2.0, 0.5, 3.0);
    CADacomb::Animation::Transform original(translation, rotation, scale);

    glm::dmat4                     matrix        = original.toMatrix();
    CADacomb::Animation::Transform reconstructed = CADacomb::Animation::Transform::fromMatrix(matrix);

    CHECK(reconstructed.getTranslation().x == Catch::Approx(translation.x));
    CHECK(reconstructed.getTranslation().y == Catch::Approx(translation.y));
    CHECK(reconstructed.getTranslation().z == Catch::Approx(translation.z));
    CHECK(reconstructed.getScale().x == Catch::Approx(scale.x));
    CHECK(reconstructed.getScale().y == Catch::Approx(scale.y));
    CHECK(reconstructed.getScale().z == Catch::Approx(scale.z));
    CHECK(std::abs(glm::dot(reconstructed.getRotation(), rotation)) == Catch::Approx(1.0));
}

TEST_CASE("AnimationTransform/InterpolateMidpoint")
{
    CADacomb::Animation::Transform a(glm::dvec3(0.0, 0.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(1.0, 1.0, 1.0));
    CADacomb::Animation::Transform b(glm::dvec3(4.0, 0.0, 0.0), glm::angleAxis(glm::half_pi<double>(), glm::dvec3(0.0, 0.0, 1.0)), glm::dvec3(3.0, 3.0, 3.0));

    CADacomb::Animation::Transform mid = CADacomb::Animation::Transform::interpolate(a, b, 0.5);
    CHECK(mid.getTranslation().x == Catch::Approx(2.0));
    CHECK(mid.getScale().x == Catch::Approx(2.0));

    glm::dquat expectedRot = glm::angleAxis(glm::quarter_pi<double>(), glm::dvec3(0.0, 0.0, 1.0));
    CHECK(std::abs(glm::dot(mid.getRotation(), expectedRot)) == Catch::Approx(1.0));
}
#endif
