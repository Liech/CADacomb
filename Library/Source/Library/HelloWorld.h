#pragma once

#include <string>

class HelloWorld
{
  public:
    HelloWorld();
    virtual ~HelloWorld();

    std::string message() const;
};
