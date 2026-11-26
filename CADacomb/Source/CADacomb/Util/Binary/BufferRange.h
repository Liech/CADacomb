#pragma once

#include <cstddef>

namespace CADacomb
{
    struct BufferRange
    {
      public:
        BufferRange();
        BufferRange(size_t offset, size_t count);

        bool operator==(const BufferRange& other) const;

        size_t offset = 0;
        size_t count  = 0;
    };
}
