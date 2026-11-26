#include "CADacomb/Triangle/EdgeCollapseCandidate.h"

namespace CADacomb
{
    EdgeCollapseCandidate::EdgeCollapseCandidate() {}

    EdgeCollapseCandidate::EdgeCollapseCandidate(double errorInput, size_t vAInput, size_t vBInput, uint32_t verAInput, uint32_t verBInput, const glm::dvec3& targetPosInput)
      : error(errorInput)
      , vA(vAInput)
      , vB(vBInput)
      , verA(verAInput)
      , verB(verBInput)
      , targetPos(targetPosInput)
    {
    }

    EdgeCollapseCandidate::~EdgeCollapseCandidate() {}

    bool EdgeCollapseCandidate::operator>(const EdgeCollapseCandidate& other) const
    {
        return error > other.error;
    }
}
