#include "BinaryVolumeToSVO.h"

namespace Library::Volume
{
    BinaryVolumeToSVO::BinaryVolumeToSVO() {}

    BinaryVolumeToSVO::~BinaryVolumeToSVO() {}

    glm::dmat4 BinaryVolumeToSVO::getTransformation(const BinaryVolume& volume)
    {
        if (volume.dimension.x == 0 || volume.dimension.y == 0 || volume.dimension.z == 0)
        {
            return glm::dmat4(1.0);
        }

        glm::dvec3 voxelSize = volume.size / glm::dvec3(volume.dimension);
        glm::dmat4 mat       = glm::translate(glm::dmat4(1.0), volume.origin);
        mat                  = glm::scale(mat, voxelSize);
        return mat;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace Library::Volume
{
    TEST_CASE("BinaryVolumeToSVO/CubeBasic", "[SVO][Converter]")
    {
        Library::BinaryVolume vol;
        vol.dimension = glm::u64vec3(4, 4, 4);
        vol.origin    = glm::dvec3(0.0, 0.0, 0.0);
        vol.size      = glm::dvec3(4.0, 4.0, 4.0);
        vol.data.resize(64, false);

        vol.data[1 + 1 * 4 + 1 * 16] = true;
        vol.data[2 + 2 * 4 + 2 * 16] = true;

        auto svo = BinaryVolumeToSVO::convert<bool>(vol, true, false);
        REQUIRE(svo != nullptr);
        CHECK(svo->getDimension() == 4);

        CHECK(svo->getVoxel({ 1, 1, 1 }) == true);
        CHECK(svo->getVoxel({ 2, 2, 2 }) == true);
        CHECK(svo->getVoxel({ 0, 0, 0 }) == false);
        CHECK(svo->getVoxel({ 3, 3, 3 }) == false);
    }

    TEST_CASE("BinaryVolumeToSVO/NonPowerOfTwoPadding", "[SVO][Converter]")
    {
        Library::BinaryVolume vol;
        vol.dimension = glm::u64vec3(3, 5, 2);
        vol.origin    = glm::dvec3(-1.0, 0.0, 2.0);
        vol.size      = glm::dvec3(3.0, 5.0, 2.0);
        vol.data.resize(3 * 5 * 2, false);

        vol.data[2 + 4 * 3 + 1 * 15] = true;

        auto svo = BinaryVolumeToSVO::convert<uint8_t>(vol, 255, 0);
        REQUIRE(svo != nullptr);
        CHECK(svo->getDimension() == 8);

        CHECK(svo->getVoxel({ 2, 4, 1 }) == 255);
        CHECK(svo->getVoxel({ 0, 0, 0 }) == 0);
        CHECK(svo->getVoxel({ 7, 7, 7 }) == 0);

        glm::dmat4 mat         = BinaryVolumeToSVO::getTransformation(vol);
        glm::dvec4 localOrigin = glm::dvec4(0.0, 0.0, 0.0, 1.0);
        glm::dvec4 worldOrigin = mat * localOrigin;
        CHECK(worldOrigin.x == -1.0);
        CHECK(worldOrigin.y == 0.0);
        CHECK(worldOrigin.z == 2.0);
    }

    TEST_CASE("BinaryVolumeToSVO/FloatConversionWithResult", "[SVO][Converter]")
    {
        Library::BinaryVolume vol;
        vol.dimension = glm::u64vec3(8, 8, 8);
        vol.origin    = glm::dvec3(0.0);
        vol.size      = glm::dvec3(1.0);
        vol.data.resize(512, false);

        vol.data[3 + 3 * 8 + 3 * 64] = true;

        SVOResult res = BinaryVolumeToSVO::convertWithResult<float>(vol, 1.0f, 0.0f);
        REQUIRE(res.tree != nullptr);
        CHECK(res.tree->getType() == typeid(float).name());

        auto* floatSvo = dynamic_cast<SVO<float>*>(res.tree.get());
        REQUIRE(floatSvo != nullptr);
        CHECK(floatSvo->getVoxel({ 3, 3, 3 }) == 1.0f);
        CHECK(floatSvo->getVoxel({ 0, 0, 0 }) == 0.0f);
    }
}
#endif
