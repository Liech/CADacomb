#include "SVOResult.h"

namespace Library::Volume
{
    SVOResult::SVOResult() {}

    SVOResult::~SVOResult() {}

    SVOResult::SVOResult(SVOResult&& other) noexcept
      : tree(std::move(other.tree))
      , transformation(other.transformation)
    {
    }

    SVOResult& SVOResult::operator=(SVOResult&& other) noexcept
    {
        if (this != &other)
        {
            tree           = std::move(other.tree);
            transformation = other.transformation;
        }
        return *this;
    }
}
