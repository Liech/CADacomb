#pragma once

#include <string>

class RenderHelper
{
  public:
    RenderHelper();
    virtual ~RenderHelper();

    std::string name() const;
};
