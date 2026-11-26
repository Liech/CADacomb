#include "CADacomb/Animation/Skeleton/SkeletonPose.h"

#include "CADacomb/Animation/Skeleton/Skeleton.h"

namespace CADacomb::Animation
{
    SkeletonPose::SkeletonPose() {}

    SkeletonPose::SkeletonPose(size_t jointCount)
      : localTransforms(jointCount, Transform::identity())
    {
    }

    SkeletonPose::~SkeletonPose() {}

    size_t SkeletonPose::getJointCount() const
    {
        return localTransforms.size();
    }

    const Transform& SkeletonPose::getLocalTransform(size_t jointIndex) const
    {
        return localTransforms[jointIndex];
    }

    Transform& SkeletonPose::getLocalTransform(size_t jointIndex)
    {
        return localTransforms[jointIndex];
    }

    void SkeletonPose::setLocalTransform(size_t jointIndex, const Transform& transform)
    {
        localTransforms[jointIndex] = transform;
    }

    const std::vector<Transform>& SkeletonPose::getLocalTransforms() const
    {
        return localTransforms;
    }

    std::vector<glm::dmat4> SkeletonPose::computeModelMatrices(const Skeleton& skeleton) const
    {
        size_t                  jointCount = skeleton.getJointCount();
        std::vector<glm::dmat4> modelMatrices(jointCount, glm::dmat4(1.0));
        std::vector<bool>       resolved(jointCount, false);

        for (size_t i = 0; i < jointCount; i++)
        {
            resolveModelMatrix(skeleton, i, modelMatrices, resolved);
        }

        return modelMatrices;
    }

    std::vector<glm::dmat4> SkeletonPose::computeSkinningMatrices(const Skeleton& skeleton) const
    {
        std::vector<glm::dmat4> modelMatrices = computeModelMatrices(skeleton);
        std::vector<glm::dmat4> skinningMatrices(modelMatrices.size(), glm::dmat4(1.0));

        for (size_t i = 0; i < modelMatrices.size(); i++)
        {
            skinningMatrices[i] = modelMatrices[i] * skeleton.getJoint(i).getInverseBindMatrix();
        }

        return skinningMatrices;
    }

    void SkeletonPose::setFromModelMatrices(const Skeleton& skeleton, const std::vector<glm::dmat4>& modelMatrices)
    {
        size_t jointCount = skeleton.getJointCount();
        localTransforms.resize(jointCount, Transform::identity());

        for (size_t i = 0; i < jointCount; i++)
        {
            int parentIndex = skeleton.getJoint(i).getParentIndex();
            if (parentIndex < 0)
            {
                localTransforms[i] = Transform::fromMatrix(modelMatrices[i]);
            }
            else
            {
                glm::dmat4 localMatrix = glm::inverse(modelMatrices[static_cast<size_t>(parentIndex)]) * modelMatrices[i];
                localTransforms[i]     = Transform::fromMatrix(localMatrix);
            }
        }
    }

    SkeletonPose SkeletonPose::blend(const SkeletonPose& from, const SkeletonPose& to, double weight)
    {
        size_t       jointCount = from.localTransforms.size();
        SkeletonPose result(jointCount);
        for (size_t i = 0; i < jointCount; i++)
        {
            result.localTransforms[i] = Transform::interpolate(from.localTransforms[i], to.localTransforms[i], weight);
        }
        return result;
    }

    void SkeletonPose::resolveModelMatrix(const Skeleton& skeleton, size_t jointIndex, std::vector<glm::dmat4>& modelMatrices, std::vector<bool>& resolved) const
    {
        if (resolved[jointIndex])
        {
            return;
        }

        glm::dmat4 localMatrix = localTransforms[jointIndex].toMatrix();
        int        parentIndex = skeleton.getJoint(jointIndex).getParentIndex();
        if (parentIndex >= 0)
        {
            size_t parentIdx = static_cast<size_t>(parentIndex);
            resolveModelMatrix(skeleton, parentIdx, modelMatrices, resolved);
            modelMatrices[jointIndex] = modelMatrices[parentIdx] * localMatrix;
        }
        else
        {
            modelMatrices[jointIndex] = localMatrix;
        }

        resolved[jointIndex] = true;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <glm/gtc/constants.hpp>

TEST_CASE("AnimationSkeletonPose/ProceduralRotationAndModelRoundTrip")
{
    CADacomb::Animation::Skeleton skeleton;
    int rootIdx  = skeleton.addJoint(CADacomb::Animation::Joint("Root", -1, CADacomb::Animation::Transform(glm::dvec3(0.0, 0.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(1.0))));
    int childIdx = skeleton.addJoint(CADacomb::Animation::Joint("Tip", rootIdx, CADacomb::Animation::Transform(glm::dvec3(0.0, 2.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(1.0))));
    skeleton.computeInverseBindMatrices();

    CADacomb::Animation::SkeletonPose pose = skeleton.createRestPose();
    pose.getLocalTransform(rootIdx).setRotation(glm::angleAxis(-glm::half_pi<double>(), glm::dvec3(0.0, 0.0, 1.0)));

    std::vector<glm::dmat4> modelMats = pose.computeModelMatrices(skeleton);
    CHECK(modelMats[childIdx][3].x == Catch::Approx(2.0).margin(1e-9));
    CHECK(modelMats[childIdx][3].y == Catch::Approx(0.0).margin(1e-9));

    CADacomb::Animation::SkeletonPose fromPhysics;
    fromPhysics.setFromModelMatrices(skeleton, modelMats);
    std::vector<glm::dmat4> roundTripMats = fromPhysics.computeModelMatrices(skeleton);
    CHECK(roundTripMats[childIdx][3].x == Catch::Approx(2.0).margin(1e-9));
    CHECK(roundTripMats[childIdx][3].y == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("AnimationSkeletonPose/BlendPoses")
{
    CADacomb::Animation::Skeleton skeleton;
    skeleton.addJoint(CADacomb::Animation::Joint("Root", -1, CADacomb::Animation::Transform::identity()));

    CADacomb::Animation::SkeletonPose poseA(1);
    poseA.setLocalTransform(0, CADacomb::Animation::Transform(glm::dvec3(0.0, 0.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(1.0)));

    CADacomb::Animation::SkeletonPose poseB(1);
    poseB.setLocalTransform(0, CADacomb::Animation::Transform(glm::dvec3(10.0, 0.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(1.0)));

    CADacomb::Animation::SkeletonPose blended = CADacomb::Animation::SkeletonPose::blend(poseA, poseB, 0.25);
    CHECK(blended.getLocalTransform(0).getTranslation().x == Catch::Approx(2.5));
}
#endif
