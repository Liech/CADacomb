#pragma once

#include <array>

namespace Library::Volume
{
    template<typename T>
    class MergeStrategy
    {
      public:
        MergeStrategy();
        virtual ~MergeStrategy();

        virtual T merge(const std::array<T, 8>& inputs) const = 0;
    };

    template<typename T>
    MergeStrategy<T>::MergeStrategy()
    {
    }

    template<typename T>
    MergeStrategy<T>::~MergeStrategy()
    {
    }
}
