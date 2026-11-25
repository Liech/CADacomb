#pragma once

#include "CADacomb/IO/FbxSceneData.h"
#include "CADacomb/Triangle/Triangulation.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct ufbx_scene;
struct ufbx_texture;

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
        static std::unique_ptr<FbxSceneData>     extractScene(const ufbx_scene* scene, const std::string& sourceDirectory);
        static std::unique_ptr<Texture::Texture> extractDiffuseTexture(const ufbx_scene* scene, const std::string& sourceDirectory);
        static std::unique_ptr<Texture::Texture> loadUfbxTexture(const ufbx_texture* texture, const std::string& sourceDirectory);
    };
}
