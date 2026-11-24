#include "CADacomb/Animation/Skinning/TriangulationSkinning.h"

namespace CADacomb::Animation
{
    TriangulationSkinning::TriangulationSkinning() {}

    TriangulationSkinning::TriangulationSkinning(size_t vertexCount)
      : vertexWeights(vertexCount)
    {
    }

    TriangulationSkinning::~TriangulationSkinning() {}

    size_t TriangulationSkinning::getVertexCount() const
    {
        return vertexWeights.size();
    }

    void TriangulationSkinning::resize(size_t vertexCount)
    {
        vertexWeights.resize(vertexCount);
    }

    const VertexSkinWeights& TriangulationSkinning::getVertexWeights(size_t vertexIndex) const
    {
        return vertexWeights[vertexIndex];
    }

    VertexSkinWeights& TriangulationSkinning::getVertexWeights(size_t vertexIndex)
    {
        return vertexWeights[vertexIndex];
    }

    void TriangulationSkinning::setVertexWeights(size_t vertexIndex, const VertexSkinWeights& weights)
    {
        vertexWeights[vertexIndex] = weights;
    }

    const std::vector<VertexSkinWeights>& TriangulationSkinning::getAllVertexWeights() const
    {
        return vertexWeights;
    }

    void TriangulationSkinning::normalizeAll()
    {
        for (auto& entry : vertexWeights)
        {
            entry.normalize();
        }
    }

    std::vector<glm::dvec3> TriangulationSkinning::computeSkinnedPositions(const std::vector<glm::dvec3>& restPositions, const std::vector<glm::dmat4>& skinningMatrices) const
    {
        std::vector<glm::dvec3> skinnedPositions(restPositions.size(), glm::dvec3(0.0, 0.0, 0.0));

        for (size_t i = 0; i < restPositions.size(); i++)
        {
            const VertexSkinWeights& entry   = vertexWeights[i];
            const glm::uvec4&        indices = entry.getJointIndices();
            const glm::dvec4&        weights = entry.getWeights();
            glm::dvec4               restPos(restPositions[i], 1.0);
            glm::dvec4               blended(0.0, 0.0, 0.0, 0.0);

            for (int slot = 0; slot < 4; slot++)
            {
                double w = weights[slot];
                if (w > 0.0)
                {
                    blended += w * (skinningMatrices[indices[slot]] * restPos);
                }
            }

            skinnedPositions[i] = glm::dvec3(blended.x, blended.y, blended.z);
        }

        return skinnedPositions;
    }
}

#ifdef ISTESTPROJECT
#include "CADacomb/Animation/Skeleton/Skeleton.h"
#include "CADacomb/Animation/Skeleton/SkeletonPose.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("AnimationTriangulationSkinning/LinearBlendSkinning")
{
    CADacomb::Animation::Skeleton skeleton;
    int                           joint0 = skeleton.addJoint(CADacomb::Animation::Joint("Root", -1, CADacomb::Animation::Transform::identity()));
    int joint1 = skeleton.addJoint(CADacomb::Animation::Joint("Child", joint0, CADacomb::Animation::Transform(glm::dvec3(0.0, 2.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(1.0))));
    skeleton.computeInverseBindMatrices();

    CADacomb::Animation::SkeletonPose pose = skeleton.createRestPose();
    pose.getLocalTransform(joint1).setTranslation(glm::dvec3(4.0, 2.0, 0.0));
    std::vector<glm::dmat4> skinMats = pose.computeSkinningMatrices(skeleton);

    CADacomb::Animation::TriangulationSkinning skinning(2);
    skinning.getVertexWeights(0).addInfluence(static_cast<uint32_t>(joint0), 1.0);
    skinning.getVertexWeights(1).addInfluence(static_cast<uint32_t>(joint0), 0.5);
    skinning.getVertexWeights(1).addInfluence(static_cast<uint32_t>(joint1), 0.5);
    skinning.normalizeAll();

    std::vector<glm::dvec3> restVerts = { glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.0, 1.0, 0.0) };
    std::vector<glm::dvec3> skinned   = skinning.computeSkinnedPositions(restVerts, skinMats);

    CHECK(skinned[0].x == Catch::Approx(0.0));
    CHECK(skinned[1].x == Catch::Approx(2.0));
    CHECK(skinned[1].y == Catch::Approx(1.0));
}
#endif
