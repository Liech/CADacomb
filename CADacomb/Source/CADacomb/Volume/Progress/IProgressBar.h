#pragma once

#include <string>

namespace CADacomb
{
    class IProgressBar
    {
      public:
        IProgressBar();
        virtual ~IProgressBar();

        virtual void        setProgress(float progress, const std::string& message) = 0;
        virtual float       getProgress() const                                     = 0;
        virtual std::string getMessage() const                                      = 0;
    };
}
