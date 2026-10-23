#include "VulcantFragmentationBuffer.h"

#ifdef ISTESTPROJECT
#include "Vulcant/VulcantV/VulcantVDevice.h"
#include <catch2/catch_test_macros.hpp>
#include <vector>

namespace Library
{
    TEST_CASE("VulcantFragmentationBuffer/PackageManagementAndDefragmentation", "[VulcantFragmentationBuffer]")
    {
        Vulcant::VulcantV::VulcantVDevice device({}, false);
        VulcantFragmentationBuffer<int>   buffer(device, 100, Vulcant::VulcantBufferType::Storage);

        CHECK(buffer.size() == 0);
        CHECK(buffer.getCapacity() == 100);

        std::vector<int> pkg1 = { 10, 20, 30 };
        std::vector<int> pkg2 = { 40, 50 };
        std::vector<int> pkg3 = { 60, 70, 80 };

        size_t id1 = buffer.set(pkg1);
        size_t id2 = buffer.set(pkg2);
        size_t id3 = buffer.set(pkg3);

        CHECK(buffer.size() == 8);

        buffer.syncGPU();

        std::vector<int> gpuData(buffer.size());
        buffer.getGPUBuffer().downloadFromGPU(gpuData.data(), buffer.size(), 0);
        std::vector<int> expectedInitial = { 10, 20, 30, 40, 50, 60, 70, 80 };
        CHECK(gpuData == expectedInitial);

        bool removed = buffer.remove(id2);
        CHECK(removed);
        CHECK(buffer.size() == 6);

        buffer.syncGPU();

        gpuData.resize(buffer.size());
        buffer.getGPUBuffer().downloadFromGPU(gpuData.data(), buffer.size(), 0);
        std::vector<int> expectedAfterRemove = { 10, 20, 30, 70, 80, 60 };
        CHECK(gpuData == expectedAfterRemove);

        buffer.clear();
        CHECK(buffer.size() == 0);
    }

    TEST_CASE("VulcantFragmentationBuffer/CapacityLimitExceeded", "[VulcantFragmentationBuffer]")
    {
        Vulcant::VulcantV::VulcantVDevice device({}, false);
        VulcantFragmentationBuffer<int>   buffer(device, 5, Vulcant::VulcantBufferType::Storage);

        std::vector<int> pkg1 = { 1, 2, 3 };
        std::vector<int> pkg2 = { 4, 5, 6 };

        size_t id1 = buffer.set(pkg1);
        CHECK(id1 != 0);
        CHECK(buffer.size() == 3);

        size_t id2 = buffer.set(pkg2);
        CHECK(id2 == 0);
        CHECK(buffer.size() == 3);
    }

    TEST_CASE("VulcantFragmentationBuffer/VertexTypeBuffer", "[VulcantFragmentationBuffer]")
    {
        Vulcant::VulcantV::VulcantVDevice device({}, false);
        VulcantFragmentationBuffer<float> buffer(device, 50, Vulcant::VulcantBufferType::Vertex);

        std::vector<float> batch = { 1.5f, 2.5f, 3.5f, 4.5f };
        size_t             id    = buffer.set(batch);
        CHECK(id != 0);

        buffer.syncGPU();

        std::vector<float> gpuData(buffer.size());
        buffer.getGPUBuffer().downloadFromGPU(gpuData.data(), buffer.size(), 0);
        CHECK(gpuData == batch);
    }
}
#endif
