#pragma once

#include <cstddef>
#include <cstdint>
#include <glm/glm.hpp>

namespace CADacomb
{
    // Candidate edge collapse entry ordered by quadric error.
    struct EdgeCollapseCandidate
    {
      public:
        EdgeCollapseCandidate();
        EdgeCollapseCandidate(double error, size_t vA, size_t vB, uint32_t verA, uint32_t verB, const glm::dvec3& targetPos);
        virtual ~EdgeCollapseCandidate();

        bool operator>(const EdgeCollapseCandidate& other) const;

        double     error     = 0.0;
        size_t     vA        = 0;
        size_t     vB        = 0;
        uint32_t   verA      = 0;
        uint32_t   verB      = 0;
        glm::dvec3 targetPos = glm::dvec3(0.0);
    };
}
