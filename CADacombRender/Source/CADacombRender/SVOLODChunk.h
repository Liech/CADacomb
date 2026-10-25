#pragma once

#include "CADacomb/Volume/AABB3.h"

#include <cstdint>
#include <glm/glm.hpp>

namespace CADacomb::Render
{
    class SVOLODChunk
    {
      public:
        SVOLODChunk();
        SVOLODChunk(const glm::ivec3& minBound, uint32_t size, const CADacomb::AABB3& worldBounds, int initialLOD);
        virtual ~SVOLODChunk();

        const CADacomb::AABB3& getWorldBounds() const;
        const glm::ivec3&      getMinBound() const;
        uint32_t               getSize() const;
        int                    getLOD() const;
        void                   setLOD(int lod);

        size_t getPackageId() const;
        void   setPackageId(size_t id);
        bool   hasPackage() const;
        void   clearPackage();

        bool isDirty() const;
        void setDirty(bool dirty);

        double getVoxelLength(double leafVoxelLength) const;

      private:
        CADacomb::AABB3 worldBounds;
        glm::ivec3      minBound{ 0 };
        uint32_t        size       = 32;
        int             lodLevel   = 0;
        size_t          packageId  = 0;
        bool            packageSet = false;
        bool            dirty      = true;
    };
}
