#include "CADacomb/Animation/Skeleton/Skeleton.h"

#include "CADacomb/Animation/Skeleton/SkeletonPose.h"

namespace CADacomb::Animation
{
    Skeleton::Skeleton() {}

    Skeleton::~Skeleton() {}

    int Skeleton::addJoint(const Joint& joint)
    {
        joints.push_back(joint);
        return static_cast<int>(joints.size() - 1);
    }

    size_t Skeleton::getJointCount() const
    {
        return joints.size();
    }

    const Joint& Skeleton::getJoint(size_t index) const
    {
        return joints[index];
    }

    Joint& Skeleton::getJoint(size_t index)
    {
        return joints[index];
    }

    const std::vector<Joint>& Skeleton::getJoints() const
    {
        return joints;
    }

    int Skeleton::findJointIndex(const std::string& name) const
    {
        for (size_t i = 0; i < joints.size(); i++)
        {
            if (joints[i].getName() == name)
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    std::vector<size_t> Skeleton::getChildIndices(size_t jointIndex) const
    {
        std::vector<size_t> children;
        for (size_t i = 0; i < joints.size(); i++)
        {
            if (joints[i].getParentIndex() == static_cast<int>(jointIndex))
            {
                children.push_back(i);
            }
        }
        return children;
    }

    void Skeleton::computeInverseBindMatrices()
    {
        SkeletonPose            restPose      = createRestPose();
        std::vector<glm::dmat4> modelMatrices = restPose.computeModelMatrices(*this);
        for (size_t i = 0; i < joints.size(); i++)
        {
            joints[i].setInverseBindMatrix(glm::inverse(modelMatrices[i]));
        }
    }

    SkeletonPose Skeleton::createRestPose() const
    {
        SkeletonPose pose(joints.size());
        for (size_t i = 0; i < joints.size(); i++)
        {
            pose.setLocalTransform(i, joints[i].getRestLocalTransform());
        }
        return pose;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("AnimationSkeleton/HierarchyAndInverseBindMatrices")
{
    CADacomb::Animation::Skeleton skeleton;
    int rootIdx  = skeleton.addJoint(CADacomb::Animation::Joint("Root", -1, CADacomb::Animation::Transform(glm::dvec3(0.0, 1.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(1.0))));
    int childIdx = skeleton.addJoint(CADacomb::Animation::Joint("Child", rootIdx, CADacomb::Animation::Transform(glm::dvec3(0.0, 2.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(1.0))));

    CHECK(skeleton.getJointCount() == 2);
    CHECK(skeleton.findJointIndex("Root") == 0);
    CHECK(skeleton.findJointIndex("Child") == 1);
    CHECK(skeleton.findJointIndex("Missing") == -1);
    CHECK(skeleton.getChildIndices(0) == std::vector<size_t>{ 1 });
    CHECK(skeleton.getChildIndices(1).empty());

    skeleton.computeInverseBindMatrices();
    CADacomb::Animation::SkeletonPose restPose = skeleton.createRestPose();
    std::vector<glm::dmat4>           skinMats = restPose.computeSkinningMatrices(skeleton);

    CHECK(skinMats.size() == 2);
    CHECK(skinMats[childIdx][3].x == Catch::Approx(0.0).margin(1e-9));
    CHECK(skinMats[childIdx][3].y == Catch::Approx(0.0).margin(1e-9));
    CHECK(skinMats[childIdx][3].z == Catch::Approx(0.0).margin(1e-9));
}
#endif
