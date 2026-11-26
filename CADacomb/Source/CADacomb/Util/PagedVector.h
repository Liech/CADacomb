#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace CADacomb
{
    // Fixed-size paged container that grows without relocating existing elements or causing 2x reallocation spikes.
    template<typename T, size_t pageShift = 12>
    class PagedVector
    {
      public:
        PagedVector();
        explicit PagedVector(size_t initialCount);
        PagedVector(const PagedVector& other);
        PagedVector(PagedVector&& other) noexcept;
        virtual ~PagedVector();

        PagedVector& operator=(const PagedVector& other);
        PagedVector& operator=(PagedVector&& other) noexcept;

        T&       operator[](size_t index);
        const T& operator[](size_t index) const;

        T&       back();
        const T& back() const;

        size_t size() const;
        size_t capacity() const;
        bool   empty() const;
        size_t pageCount() const;

        T*       getPageData(size_t pageIndex);
        const T* getPageData(size_t pageIndex) const;

        void reserve(size_t newCapacity);
        void reservePageTable(size_t maxPages);
        void resize(size_t newSize);
        void clear();
        void shrinkToFit();

        void push_back(const T& value);
        void push_back(T&& value);

        template<typename... Args>
        T& emplace_back(Args&&... args);

        void pop_back();

        static constexpr size_t getPageSize();
        static constexpr size_t getPageMask();

      private:
        void ensurePageForIndex(size_t index);

        static constexpr size_t pageSize = size_t(1) << pageShift;
        static constexpr size_t pageMask = pageSize - 1;

        std::vector<std::unique_ptr<T[]>> pages;
        size_t                            elementCount = 0;
    };

    template<typename T, size_t pageShift>
    PagedVector<T, pageShift>::PagedVector()
    {
    }

    template<typename T, size_t pageShift>
    PagedVector<T, pageShift>::PagedVector(size_t initialCount)
    {
        resize(initialCount);
    }

    template<typename T, size_t pageShift>
    PagedVector<T, pageShift>::PagedVector(const PagedVector& other)
      : elementCount(other.elementCount)
    {
        pages.reserve(other.pages.size());
        for (size_t p = 0; p < other.pages.size(); p++)
        {
            auto page = std::make_unique<T[]>(pageSize);
            for (size_t i = 0; i < pageSize; i++)
            {
                page[i] = other.pages[p][i];
            }
            pages.push_back(std::move(page));
        }
    }

    template<typename T, size_t pageShift>
    PagedVector<T, pageShift>::PagedVector(PagedVector&& other) noexcept
      : pages(std::move(other.pages))
      , elementCount(other.elementCount)
    {
        other.elementCount = 0;
    }

    template<typename T, size_t pageShift>
    PagedVector<T, pageShift>::~PagedVector()
    {
    }

    template<typename T, size_t pageShift>
    PagedVector<T, pageShift>& PagedVector<T, pageShift>::operator=(const PagedVector& other)
    {
        if (this == &other)
        {
            return *this;
        }

        pages.clear();
        pages.reserve(other.pages.size());
        for (size_t p = 0; p < other.pages.size(); p++)
        {
            auto page = std::make_unique<T[]>(pageSize);
            for (size_t i = 0; i < pageSize; i++)
            {
                page[i] = other.pages[p][i];
            }
            pages.push_back(std::move(page));
        }
        elementCount = other.elementCount;
        return *this;
    }

    template<typename T, size_t pageShift>
    PagedVector<T, pageShift>& PagedVector<T, pageShift>::operator=(PagedVector&& other) noexcept
    {
        if (this == &other)
        {
            return *this;
        }

        pages              = std::move(other.pages);
        elementCount       = other.elementCount;
        other.elementCount = 0;
        return *this;
    }

    template<typename T, size_t pageShift>
    T& PagedVector<T, pageShift>::operator[](size_t index)
    {
        assert(index < elementCount);
        return pages[index >> pageShift][index & pageMask];
    }

    template<typename T, size_t pageShift>
    const T& PagedVector<T, pageShift>::operator[](size_t index) const
    {
        assert(index < elementCount);
        return pages[index >> pageShift][index & pageMask];
    }

    template<typename T, size_t pageShift>
    T& PagedVector<T, pageShift>::back()
    {
        assert(elementCount > 0);
        return (*this)[elementCount - 1];
    }

    template<typename T, size_t pageShift>
    const T& PagedVector<T, pageShift>::back() const
    {
        assert(elementCount > 0);
        return (*this)[elementCount - 1];
    }

    template<typename T, size_t pageShift>
    size_t PagedVector<T, pageShift>::size() const
    {
        return elementCount;
    }

    template<typename T, size_t pageShift>
    size_t PagedVector<T, pageShift>::capacity() const
    {
        return pages.size() * pageSize;
    }

    template<typename T, size_t pageShift>
    bool PagedVector<T, pageShift>::empty() const
    {
        return elementCount == 0;
    }

    template<typename T, size_t pageShift>
    size_t PagedVector<T, pageShift>::pageCount() const
    {
        return pages.size();
    }

    template<typename T, size_t pageShift>
    T* PagedVector<T, pageShift>::getPageData(size_t pageIndex)
    {
        assert(pageIndex < pages.size());
        return pages[pageIndex].get();
    }

    template<typename T, size_t pageShift>
    const T* PagedVector<T, pageShift>::getPageData(size_t pageIndex) const
    {
        assert(pageIndex < pages.size());
        return pages[pageIndex].get();
    }

    template<typename T, size_t pageShift>
    void PagedVector<T, pageShift>::reserve(size_t newCapacity)
    {
        size_t requiredPages = (newCapacity + pageMask) >> pageShift;
        while (pages.size() < requiredPages)
        {
            pages.push_back(std::make_unique<T[]>(pageSize));
        }
    }

    template<typename T, size_t pageShift>
    void PagedVector<T, pageShift>::reservePageTable(size_t maxPages)
    {
        pages.reserve(maxPages);
    }

    template<typename T, size_t pageShift>
    void PagedVector<T, pageShift>::resize(size_t newSize)
    {
        if (newSize > elementCount)
        {
            reserve(newSize);
        }
        elementCount = newSize;
    }

    template<typename T, size_t pageShift>
    void PagedVector<T, pageShift>::clear()
    {
        pages.clear();
        elementCount = 0;
    }

    template<typename T, size_t pageShift>
    void PagedVector<T, pageShift>::shrinkToFit()
    {
        size_t requiredPages = (elementCount + pageMask) >> pageShift;
        if (pages.size() > requiredPages)
        {
            pages.resize(requiredPages);
        }
        pages.shrink_to_fit();
    }

    template<typename T, size_t pageShift>
    void PagedVector<T, pageShift>::push_back(const T& value)
    {
        ensurePageForIndex(elementCount);
        pages[elementCount >> pageShift][elementCount & pageMask] = value;
        elementCount++;
    }

    template<typename T, size_t pageShift>
    void PagedVector<T, pageShift>::push_back(T&& value)
    {
        ensurePageForIndex(elementCount);
        pages[elementCount >> pageShift][elementCount & pageMask] = std::move(value);
        elementCount++;
    }

    template<typename T, size_t pageShift>
    template<typename... Args>
    T& PagedVector<T, pageShift>::emplace_back(Args&&... args)
    {
        ensurePageForIndex(elementCount);
        T& slot = pages[elementCount >> pageShift][elementCount & pageMask];
        slot    = T(std::forward<Args>(args)...);
        elementCount++;
        return slot;
    }

    template<typename T, size_t pageShift>
    void PagedVector<T, pageShift>::pop_back()
    {
        assert(elementCount > 0);
        elementCount--;
    }

    template<typename T, size_t pageShift>
    constexpr size_t PagedVector<T, pageShift>::getPageSize()
    {
        return pageSize;
    }

    template<typename T, size_t pageShift>
    constexpr size_t PagedVector<T, pageShift>::getPageMask()
    {
        return pageMask;
    }

    template<typename T, size_t pageShift>
    void PagedVector<T, pageShift>::ensurePageForIndex(size_t index)
    {
        size_t pageIndex = index >> pageShift;
        while (pages.size() <= pageIndex)
        {
            pages.push_back(std::make_unique<T[]>(pageSize));
        }
    }
}
