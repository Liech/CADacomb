#pragma once

#include <cstddef>

namespace Library::Binary
{
    struct BufferSpan
    {
      public:
        BufferSpan() = default;
        BufferSpan(size_t offset, size_t count, size_t packageId);
        ~BufferSpan() = default;

        bool operator==(const BufferSpan& other) const;

        size_t offset    = 0;
        size_t count     = 0;
        size_t packageId = 0;
    };
}
