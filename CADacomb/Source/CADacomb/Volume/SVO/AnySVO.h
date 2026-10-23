#pragma once

#include <string>

namespace CADacomb
{
    class AnySVO
    {
      public:
        AnySVO();
        virtual ~AnySVO();

        virtual std::string getType() = 0;
    };
}
