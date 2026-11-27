#pragma once

#include "CADacomb/IO/FbxSceneData.h"
#include "CADacomb/Triangle/Triangulation.h"

#include <cstddef>
#include <cstdint>
#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

struct ufbx_matrix;
struct ufbx_node;
struct ufbx_scene;
struct ufbx_texture;
struct ufbx_transform;

namespace CADacomb::IO
{
    // Loads FBX files and memory buffers into CADacomb representations using ufbx.
    class FbxReader
    {
      public:
        FbxReader();
        virtual ~FbxReader();

        static std::unique_ptr<FbxSceneData>  loadSceneFromFile(const std::string& filename);
        static std::unique_ptr<FbxSceneData>  loadSceneFromMemory(const std::vector<uint8_t>& bytes);
        static std::unique_ptr<Triangulation> loadTriangulationFromFile(const std::string& filename);
        static std::unique_ptr<Triangulation> loadTriangulationFromMemory(const std::vector<uint8_t>& bytes);

      private:
        static std::unique_ptr<FbxSceneData> extractScene(const ufbx_scene* scene, const std::string& sourceDirectory);
        static void                          extractSkeletonAndSkinning(const ufbx_scene*                                       scene,
                                                                        const std::vector<std::pair<const ufbx_node*, size_t>>& meshBaseVertices,
                                                                        size_t                                                  totalVertexCount,
                                                                        FbxSceneData&                                           sceneData,
                                                                        std::unordered_map<uint32_t, int>&                      outNodeIdToJointIndex);
        static void extractAnimationClips(const ufbx_scene* scene, const Animation::Skeleton& skeleton, const std::unordered_map<uint32_t, int>& nodeIdToJointIndex, FbxSceneData& sceneData);
        static std::unique_ptr<Texture::Texture> extractDiffuseTexture(const ufbx_scene* scene, const std::string& sourceDirectory);
        static std::unique_ptr<Texture::Texture> loadUfbxTexture(const ufbx_texture* texture, const std::string& sourceDirectory);
        static glm::dmat4                        toGlmMatrix(const ufbx_matrix& matrix);
        static Animation::Transform              toTransform(const ufbx_transform& transform);
    };
}
