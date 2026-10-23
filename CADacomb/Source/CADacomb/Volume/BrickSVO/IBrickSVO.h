#pragma once

#include <string>

namespace CADacomb
{
    class IBrickSVO
    {
      public:
        IBrickSVO();
        virtual ~IBrickSVO();

        virtual std::string getType() = 0;
    };
}
