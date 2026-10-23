#pragma once

#include <cstddef>

namespace Library::Binary
{
    struct BufferRange
    {
      public:
        BufferRange() = default;
        BufferRange(size_t offset, size_t count);
        ~BufferRange() = default;

        bool operator==(const BufferRange& other) const;

        size_t offset = 0;
        size_t count  = 0;
    };
}
