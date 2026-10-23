#include "PackedQuad.h"

namespace CADacomb
{
    PackedQuad::PackedQuad()
      : packed(0)
    {
    }

    PackedQuad::PackedQuad(uint64_t raw)
      : packed(raw)
    {
    }

    PackedQuad PackedQuad::create(uint32_t x, uint32_t y, uint32_t z, uint32_t dir, uint32_t scaleExp, uint32_t matId)
    {
        PackedQuad q;
        q.packed = (static_cast<uint64_t>(x & 0x3FF)) | (static_cast<uint64_t>(y & 0x3FF) << 10) | (static_cast<uint64_t>(z & 0x3FF) << 20) | (static_cast<uint64_t>(dir & 0x7) << 30) |
                   (static_cast<uint64_t>(scaleExp & 0xF) << 33) | (static_cast<uint64_t>(matId & 0x7FFFFFF) << 37);
        return q;
    }

    glm::ivec3 PackedQuad::getNormal(uint32_t dir)
    {
        switch (dir)
        {
            case 0:
                return glm::ivec3(-1, 0, 0);
            case 1:
                return glm::ivec3(1, 0, 0);
            case 2:
                return glm::ivec3(0, -1, 0);
            case 3:
                return glm::ivec3(0, 1, 0);
            case 4:
                return glm::ivec3(0, 0, -1);
            case 5:
                return glm::ivec3(0, 0, 1);
            default:
                return glm::ivec3(0, 0, 0);
        }
    }

    uint32_t PackedQuad::getX() const
    {
        return static_cast<uint32_t>(packed & 0x3FF);
    }

    uint32_t PackedQuad::getY() const
    {
        return static_cast<uint32_t>((packed >> 10) & 0x3FF);
    }

    uint32_t PackedQuad::getZ() const
    {
        return static_cast<uint32_t>((packed >> 20) & 0x3FF);
    }

    glm::uvec3 PackedQuad::getLocalPos() const
    {
        return glm::uvec3(getX(), getY(), getZ());
    }

    uint32_t PackedQuad::getDirection() const
    {
        return static_cast<uint32_t>((packed >> 30) & 0x7);
    }

    uint32_t PackedQuad::getScaleExp() const
    {
        return static_cast<uint32_t>((packed >> 33) & 0xF);
    }

    uint32_t PackedQuad::getScale() const
    {
        return 1u << getScaleExp();
    }

    uint32_t PackedQuad::getMaterial() const
    {
        return static_cast<uint32_t>((packed >> 37) & 0x7FFFFFF);
    }

    glm::ivec3 PackedQuad::getNormal() const
    {
        return getNormal(getDirection());
    }

    std::array<glm::dvec3, 4> PackedQuad::getCorners(const glm::dvec3& chunkWorldOrigin, double voxelSize) const
    {
        glm::dvec3 minP = chunkWorldOrigin + glm::dvec3(getX(), getY(), getZ()) * voxelSize;
        double     s    = static_cast<double>(getScale()) * voxelSize;
        glm::dvec3 maxP = minP + glm::dvec3(s);

        std::array<glm::dvec3, 4> corners;
        switch (getDirection())
        {
            case 0:
                corners[0] = glm::dvec3(minP.x, minP.y, minP.z);
                corners[1] = glm::dvec3(minP.x, minP.y, maxP.z);
                corners[2] = glm::dvec3(minP.x, maxP.y, maxP.z);
                corners[3] = glm::dvec3(minP.x, maxP.y, minP.z);
                break;
            case 1:
                corners[0] = glm::dvec3(maxP.x, minP.y, minP.z);
                corners[1] = glm::dvec3(maxP.x, maxP.y, minP.z);
                corners[2] = glm::dvec3(maxP.x, maxP.y, maxP.z);
                corners[3] = glm::dvec3(maxP.x, minP.y, maxP.z);
                break;
            case 2:
                corners[0] = glm::dvec3(minP.x, minP.y, minP.z);
                corners[1] = glm::dvec3(maxP.x, minP.y, minP.z);
                corners[2] = glm::dvec3(maxP.x, minP.y, maxP.z);
                corners[3] = glm::dvec3(minP.x, minP.y, maxP.z);
                break;
            case 3:
                corners[0] = glm::dvec3(minP.x, maxP.y, minP.z);
                corners[1] = glm::dvec3(minP.x, maxP.y, maxP.z);
                corners[2] = glm::dvec3(maxP.x, maxP.y, maxP.z);
                corners[3] = glm::dvec3(maxP.x, maxP.y, minP.z);
                break;
            case 4:
                corners[0] = glm::dvec3(minP.x, minP.y, minP.z);
                corners[1] = glm::dvec3(minP.x, maxP.y, minP.z);
                corners[2] = glm::dvec3(maxP.x, maxP.y, minP.z);
                corners[3] = glm::dvec3(maxP.x, minP.y, minP.z);
                break;
            case 5:
                corners[0] = glm::dvec3(minP.x, minP.y, maxP.z);
                corners[1] = glm::dvec3(maxP.x, minP.y, maxP.z);
                corners[2] = glm::dvec3(maxP.x, maxP.y, maxP.z);
                corners[3] = glm::dvec3(minP.x, maxP.y, maxP.z);
                break;
            default:
                corners.fill(minP);
                break;
        }
        return corners;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    TEST_CASE("SVOMesher/PackedQuadEncodingDecoding", "[SVOMesher][Quad]")
    {
        static_assert(sizeof(PackedQuad) == 8, "PackedQuad must be exactly 8 bytes (64 bits)");

        PackedQuad q = PackedQuad::create(123, 456, 789, 3, 2, 42);

        CHECK(q.getX() == 123);
        CHECK(q.getY() == 456);
        CHECK(q.getZ() == 789);
        CHECK(q.getDirection() == 3);
        CHECK(q.getScaleExp() == 2);
        CHECK(q.getScale() == 4);
        CHECK(q.getMaterial() == 42);

        glm::ivec3 normal = q.getNormal();
        CHECK(normal == glm::ivec3(0, 1, 0));

        auto corners = q.getCorners(glm::dvec3(0.0), 1.0);
        CHECK(corners.size() == 4);
        for (const auto& c : corners)
        {
            CHECK(c.y == Catch::Approx(460.0));
        }
    }
}
#endif
