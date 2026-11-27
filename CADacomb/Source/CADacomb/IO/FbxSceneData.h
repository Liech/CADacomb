#pragma once

#include "CADacomb/Animation/KeyframeAnimation/AnimationClip.h"
#include "CADacomb/Animation/Skeleton/Skeleton.h"
#include "CADacomb/Animation/Skinning/TriangulationSkinning.h"
#include "CADacomb/Texture/Mapping/TriangulationUVs.h"
#include "CADacomb/Texture/Texture.h"
#include "CADacomb/Triangle/Triangulation.h"

#include <memory>
#include <vector>

namespace CADacomb::IO
{
    // Container for decoupled CADacomb geometry, UV, texture, skeleton, skinning, and animation data extracted from an FBX scene.
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

        bool                                 hasSkeleton() const;
        const Animation::Skeleton*           getSkeleton() const;
        void                                 setSkeleton(std::unique_ptr<Animation::Skeleton> skeleton);
        std::unique_ptr<Animation::Skeleton> takeSkeleton();

        bool                                              hasSkinning() const;
        const Animation::TriangulationSkinning*           getSkinning() const;
        void                                              setSkinning(std::unique_ptr<Animation::TriangulationSkinning> skinning);
        std::unique_ptr<Animation::TriangulationSkinning> takeSkinning();

        bool                                         hasAnimationClips() const;
        const std::vector<Animation::AnimationClip>& getAnimationClips() const;
        void                                         addAnimationClip(const Animation::AnimationClip& clip);
        std::vector<Animation::AnimationClip>        takeAnimationClips();

      private:
        std::unique_ptr<Triangulation>                    triangulation;
        std::unique_ptr<Texture::TriangulationUVs>        uvs;
        std::unique_ptr<Texture::Texture>                 diffuseTexture;
        std::unique_ptr<Animation::Skeleton>              skeleton;
        std::unique_ptr<Animation::TriangulationSkinning> skinning;
        std::vector<Animation::AnimationClip>             animationClips;
    };
}
