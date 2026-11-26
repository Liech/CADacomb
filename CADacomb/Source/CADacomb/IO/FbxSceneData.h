#pragma once

#include "CADacomb/Texture/Mapping/TriangulationUVs.h"
#include "CADacomb/Texture/Texture.h"
#include "CADacomb/Triangle/Triangulation.h"

#include <memory>

namespace CADacomb::IO
{
    // Container for decoupled CADacomb geometry, UV, and texture data extracted from an FBX scene.
    class FbxSceneData
    {
      public:
        FbxSceneData();
        virtual ~FbxSceneData();

        bool                           hasTriangulation() const;
        const Triangulation*           getTriangulation() const;
        void                           setTriangulation(std::unique_ptr<Triangulation> triangulation);
        std::unique_ptr<Triangulation> takeTriangulation();

        bool                                       hasUVs() const;
        const Texture::TriangulationUVs*           getUVs() const;
        void                                       setUVs(std::unique_ptr<Texture::TriangulationUVs> uvs);
        std::unique_ptr<Texture::TriangulationUVs> takeUVs();

        bool                              hasDiffuseTexture() const;
        const Texture::Texture*           getDiffuseTexture() const;
        void                              setDiffuseTexture(std::unique_ptr<Texture::Texture> texture);
        std::unique_ptr<Texture::Texture> takeDiffuseTexture();

      private:
        std::unique_ptr<Triangulation>             triangulation;
        std::unique_ptr<Texture::TriangulationUVs> uvs;
        std::unique_ptr<Texture::Texture>          diffuseTexture;
    };
}
