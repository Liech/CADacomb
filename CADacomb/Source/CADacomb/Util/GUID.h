#pragma once

#include <string>

namespace CADacomb
{
    // Globally unique identifier used for tracking operations across threads.
    class GUID
    {
      public:
        GUID();
        GUID(const std::string& value);
        virtual ~GUID();

        static GUID random();

        const std::string& toString() const;

        bool operator==(const GUID& other) const;
        bool operator!=(const GUID& other) const;
        bool operator<(const GUID& other) const;

      private:
        std::string value;
    };
}
