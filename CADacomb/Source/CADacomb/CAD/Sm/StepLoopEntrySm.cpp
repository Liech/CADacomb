#include "StepLoopEntrySm.h"

namespace CADacomb::CAD
{
    StepLoopEntrySm::StepLoopEntrySm()
      : outerFlag(false)
      , loopValue()
    {
    }

    StepLoopEntrySm::StepLoopEntrySm(bool isOuterInput, const smlibpy::Value& loopValueInput)
      : outerFlag(isOuterInput)
      , loopValue(loopValueInput)
    {
    }

    StepLoopEntrySm::~StepLoopEntrySm() {}

    bool StepLoopEntrySm::isOuter() const
    {
        return outerFlag;
    }

    const smlibpy::Value& StepLoopEntrySm::getLoopValue() const
    {
        return loopValue;
    }
}
