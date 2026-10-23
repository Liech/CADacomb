#pragma once

#include <string>

namespace Library::Volume
{
    class AnySVO
    {
      public:
        AnySVO();
        virtual ~AnySVO();

        virtual std::string getType() = 0;
    };
}
