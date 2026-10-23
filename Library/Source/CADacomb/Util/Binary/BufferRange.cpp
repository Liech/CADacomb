#include "BufferRange.h"

namespace Library::Binary
{
    BufferRange::BufferRange(size_t offset, size_t count)
      : offset(offset)
      , count(count)
    {
    }

    bool BufferRange::operator==(const BufferRange& other) const
    {
        return offset == other.offset && count == other.count;
    }
}
