#include "CADacomb/Animation/Skeleton/Joint.h"

namespace CADacomb::Animation
{
    Joint::Joint() {}

    Joint::Joint(const std::string& nameInput, int parentIndexInput, const Transform& restLocalTransformInput)
      : name(nameInput)
      , parentIndex(parentIndexInput)
      , restLocalTransform(restLocalTransformInput)
    {
    }

    Joint::~Joint() {}

    const std::string& Joint::getName() const
    {
        return name;
    }

    void Joint::setName(const std::string& nameInput)
    {
        name = nameInput;
    }

    int Joint::getParentIndex() const
    {
        return parentIndex;
    }

    void Joint::setParentIndex(int parentIndexInput)
    {
        parentIndex = parentIndexInput;
    }

    const Transform& Joint::getRestLocalTransform() const
    {
        return restLocalTransform;
    }

    void Joint::setRestLocalTransform(const Transform& restLocalTransformInput)
    {
        restLocalTransform = restLocalTransformInput;
    }

    const glm::dmat4& Joint::getInverseBindMatrix() const
    {
        return inverseBindMatrix;
    }

    void Joint::setInverseBindMatrix(const glm::dmat4& inverseBindMatrixInput)
    {
        inverseBindMatrix = inverseBindMatrixInput;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

TEST_CASE("AnimationJoint/Properties")
{
    CADacomb::Animation::Transform rest(glm::dvec3(0.0, 2.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(1.0, 1.0, 1.0));
    CADacomb::Animation::Joint     joint("Spine", 0, rest);

    CHECK(joint.getName() == "Spine");
    CHECK(joint.getParentIndex() == 0);
    CHECK(joint.getRestLocalTransform().getTranslation().y == 2.0);
    CHECK(joint.getInverseBindMatrix() == glm::dmat4(1.0));
}
#endif
