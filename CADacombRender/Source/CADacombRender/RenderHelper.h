#pragma once

#include <string>

namespace CADacomb::Render
{
    class RenderHelper
    {
      public:
        RenderHelper();
        virtual ~RenderHelper();

        std::string name() const;
    };
}
