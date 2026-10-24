#pragma once

#include "value.h"

namespace CADacomb::CAD
{
    class StepLoopEntrySm
    {
      public:
        StepLoopEntrySm();
        StepLoopEntrySm(bool isOuterInput, const smlibpy::Value& loopValueInput);
        virtual ~StepLoopEntrySm();

        bool                  isOuter() const;
        const smlibpy::Value& getLoopValue() const;

      private:
        bool           outerFlag;
        smlibpy::Value loopValue;
    };
}
