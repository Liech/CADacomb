#include "BufferSpan.h"

namespace CADacomb
{
    BufferSpan::BufferSpan(size_t offset, size_t count, size_t packageId)
      : offset(offset)
      , count(count)
      , packageId(packageId)
    {
    }

    bool BufferSpan::operator==(const BufferSpan& other) const
    {
        return offset == other.offset && count == other.count && packageId == other.packageId;
    }
}
