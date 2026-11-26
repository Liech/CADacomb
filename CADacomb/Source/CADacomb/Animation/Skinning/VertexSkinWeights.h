#pragma once

#include <cstdint>
#include <glm/glm.hpp>

namespace CADacomb::Animation
{
    // Up to four joint indices and normalized weights describing skeletal influence on a single vertex.
    class VertexSkinWeights
    {
      public:
        VertexSkinWeights();
        VertexSkinWeights(const glm::uvec4& jointIndices, const glm::dvec4& weights);
        virtual ~VertexSkinWeights();

        void addInfluence(uint32_t jointIndex, double weight);
        void normalize();

        const glm::uvec4& getJointIndices() const;
        void              setJointIndices(const glm::uvec4& jointIndices);

        const glm::dvec4& getWeights() const;
        void              setWeights(const glm::dvec4& weights);

      private:
        glm::uvec4 jointIndices = glm::uvec4(0, 0, 0, 0);
        glm::dvec4 weights      = glm::dvec4(0.0, 0.0, 0.0, 0.0);
    };
}
