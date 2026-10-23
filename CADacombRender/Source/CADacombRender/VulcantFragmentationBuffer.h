#pragma once

#include "CADacomb/Util/Binary/DefragmentationBuffer.h"
#include "Vulcant/Interface/VulcantBuffer.h"
#include "Vulcant/Interface/VulcantBufferType.h"
#include "Vulcant/Interface/VulcantDevice.h"
#include <algorithm>
#include <cstddef>
#include <memory>
#include <span>

namespace CADacomb::Render
{
    template<typename T>
    class VulcantFragmentationBuffer
    {
      public:
        VulcantFragmentationBuffer(Vulcant::VulcantDevice& device, size_t capacity, Vulcant::VulcantBufferType bufferType = Vulcant::VulcantBufferType::Vertex);
        virtual ~VulcantFragmentationBuffer();

        size_t set(std::span<const T> package);
        bool   remove(size_t id);
        void   clear();

        void syncGPU();

        size_t                  size() const;
        size_t                  getCapacity() const;
        Vulcant::VulcantBuffer& getGPUBuffer() const;

      private:
        std::unique_ptr<Vulcant::VulcantBuffer> createGPUBuffer(size_t count);

        CADacomb::DefragmentationBuffer<T>      cpuBuffer;
        std::unique_ptr<Vulcant::VulcantBuffer> gpuBuffer;
        Vulcant::VulcantDevice&                 device;
        size_t                                  capacity   = 0;
        Vulcant::VulcantBufferType              bufferType = Vulcant::VulcantBufferType::Vertex;
    };

    template<typename T>
    VulcantFragmentationBuffer<T>::VulcantFragmentationBuffer(Vulcant::VulcantDevice& device, size_t capacity, Vulcant::VulcantBufferType bufferType)
      : device(device)
      , capacity(capacity)
      , bufferType(bufferType)
    {
        gpuBuffer = createGPUBuffer(capacity);
        cpuBuffer.reserve(capacity);
    }

    template<typename T>
    VulcantFragmentationBuffer<T>::~VulcantFragmentationBuffer()
    {
    }

    template<typename T>
    size_t VulcantFragmentationBuffer<T>::set(std::span<const T> package)
    {
        if (package.empty() || cpuBuffer.size() + package.size() > capacity)
        {
            return 0;
        }

        return cpuBuffer.set(package);
    }

    template<typename T>
    bool VulcantFragmentationBuffer<T>::remove(size_t id)
    {
        return cpuBuffer.remove(id);
    }

    template<typename T>
    void VulcantFragmentationBuffer<T>::clear()
    {
        cpuBuffer.clear();
    }

    template<typename T>
    void VulcantFragmentationBuffer<T>::syncGPU()
    {
        if (gpuBuffer == nullptr)
        {
            cpuBuffer.popDirty();
            return;
        }

        auto dirtyRanges = cpuBuffer.popDirty();
        for (const auto& range : dirtyRanges)
        {
            if (range.count > 0)
            {
                gpuBuffer->uploadToGPU(cpuBuffer.getData().data() + range.offset, range.count, range.offset);
            }
        }
    }

    template<typename T>
    size_t VulcantFragmentationBuffer<T>::size() const
    {
        return cpuBuffer.size();
    }

    template<typename T>
    size_t VulcantFragmentationBuffer<T>::getCapacity() const
    {
        return capacity;
    }

    template<typename T>
    Vulcant::VulcantBuffer& VulcantFragmentationBuffer<T>::getGPUBuffer() const
    {
        return *gpuBuffer;
    }

    template<typename T>
    std::unique_ptr<Vulcant::VulcantBuffer> VulcantFragmentationBuffer<T>::createGPUBuffer(size_t count)
    {
        if (count == 0)
        {
            return nullptr;
        }

        switch (bufferType)
        {
            case Vulcant::VulcantBufferType::Storage:
                return device.createBuffer(count, sizeof(T), false);
            case Vulcant::VulcantBufferType::Vertex:
                return device.createVertexBuffer(count, sizeof(T), false);
            case Vulcant::VulcantBufferType::Indirect:
                return device.createIndirectBuffer(count, sizeof(T), false);
            default:
                return device.createBuffer(count, sizeof(T), false);
        }
    }
}
