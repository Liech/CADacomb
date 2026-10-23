#pragma once

#include <string>

namespace Library::Volume
{
    class IBrickSVO
    {
      public:
        IBrickSVO();
        virtual ~IBrickSVO();

        virtual std::string getType() = 0;
    };
}
