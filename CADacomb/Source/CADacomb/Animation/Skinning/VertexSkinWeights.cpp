#include "CADacomb/Animation/Skinning/VertexSkinWeights.h"

namespace CADacomb::Animation
{
    VertexSkinWeights::VertexSkinWeights() {}

    VertexSkinWeights::VertexSkinWeights(const glm::uvec4& jointIndicesInput, const glm::dvec4& weightsInput)
      : jointIndices(jointIndicesInput)
      , weights(weightsInput)
    {
    }

    VertexSkinWeights::~VertexSkinWeights() {}

    void VertexSkinWeights::addInfluence(uint32_t jointIndex, double weight)
    {
        if (weight <= 0.0)
        {
            return;
        }

        int    minSlot   = 0;
        double minWeight = weights[0];
        for (int i = 1; i < 4; i++)
        {
            if (weights[i] < minWeight)
            {
                minWeight = weights[i];
                minSlot   = i;
            }
        }

        if (weight > minWeight)
        {
            jointIndices[minSlot] = jointIndex;
            weights[minSlot]      = weight;
        }
    }

    void VertexSkinWeights::normalize()
    {
        double sum = weights.x + weights.y + weights.z + weights.w;
        if (sum > 0.0)
        {
            weights /= sum;
        }
    }

    const glm::uvec4& VertexSkinWeights::getJointIndices() const
    {
        return jointIndices;
    }

    void VertexSkinWeights::setJointIndices(const glm::uvec4& jointIndicesInput)
    {
        jointIndices = jointIndicesInput;
    }

    const glm::dvec4& VertexSkinWeights::getWeights() const
    {
        return weights;
    }

    void VertexSkinWeights::setWeights(const glm::dvec4& weightsInput)
    {
        weights = weightsInput;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("AnimationVertexSkinWeights/AddTopFourAndNormalize")
{
    CADacomb::Animation::VertexSkinWeights vsw;
    vsw.addInfluence(1, 0.1);
    vsw.addInfluence(2, 0.4);
    vsw.addInfluence(3, 0.3);
    vsw.addInfluence(4, 0.2);
    vsw.addInfluence(5, 0.6);
    vsw.normalize();

    double sum = vsw.getWeights().x + vsw.getWeights().y + vsw.getWeights().z + vsw.getWeights().w;
    CHECK(sum == Catch::Approx(1.0));
    CHECK(vsw.getJointIndices()[0] == 5);
    CHECK(vsw.getWeights()[0] == Catch::Approx(0.6 / 1.5));
}
#endif
