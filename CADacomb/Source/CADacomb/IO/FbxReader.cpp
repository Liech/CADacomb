#include "CADacomb/IO/FbxReader.h"

#include <filesystem>
#include <ufbx.h>

namespace CADacomb::IO
{
    FbxReader::FbxReader() {}

    FbxReader::~FbxReader() {}

    std::unique_ptr<FbxSceneData> FbxReader::loadSceneFromFile(const std::string& filename)
    {
        ufbx_load_opts opts     = {};
        opts.target_axes        = ufbx_axes_right_handed_y_up;
        opts.target_unit_meters = 1.0;

        ufbx_error  error = {};
        ufbx_scene* scene = ufbx_load_file(filename.c_str(), &opts, &error);
        if (!scene)
        {
            return nullptr;
        }

        std::string                   sourceDirectory = std::filesystem::path(filename).parent_path().string();
        std::unique_ptr<FbxSceneData> result          = extractScene(scene, sourceDirectory);
        ufbx_free_scene(scene);
        return result;
    }

    std::unique_ptr<FbxSceneData> FbxReader::loadSceneFromMemory(const std::vector<uint8_t>& bytes)
    {
        if (bytes.empty())
        {
            return nullptr;
        }

        ufbx_load_opts opts     = {};
        opts.target_axes        = ufbx_axes_right_handed_y_up;
        opts.target_unit_meters = 1.0;

        ufbx_error  error = {};
        ufbx_scene* scene = ufbx_load_memory(bytes.data(), bytes.size(), &opts, &error);
        if (!scene)
        {
            return nullptr;
        }

        std::unique_ptr<FbxSceneData> result = extractScene(scene, "");
        ufbx_free_scene(scene);
        return result;
    }

    std::unique_ptr<Triangulation> FbxReader::loadTriangulationFromFile(const std::string& filename)
    {
        std::unique_ptr<FbxSceneData> sceneData = loadSceneFromFile(filename);
        if (!sceneData)
        {
            return nullptr;
        }
        return sceneData->takeTriangulation();
    }

    std::unique_ptr<Triangulation> FbxReader::loadTriangulationFromMemory(const std::vector<uint8_t>& bytes)
    {
        std::unique_ptr<FbxSceneData> sceneData = loadSceneFromMemory(bytes);
        if (!sceneData)
        {
            return nullptr;
        }
        return sceneData->takeTriangulation();
    }

    std::unique_ptr<FbxSceneData> FbxReader::extractScene(const ufbx_scene* scene, const std::string& sourceDirectory)
    {
        auto                    sceneData     = std::make_unique<FbxSceneData>();
        auto                    triangulation = std::make_unique<Triangulation>();
        std::vector<glm::dvec2> cornerUVs;
        bool                    anyUVs = false;
        std::vector<uint32_t>   triIndices;

        for (size_t nodeIdx = 0; nodeIdx < scene->nodes.count; nodeIdx++)
        {
            const ufbx_node* node = scene->nodes.data[nodeIdx];
            if (!node->mesh)
            {
                continue;
            }

            const ufbx_mesh* mesh       = node->mesh;
            size_t           baseVertex = triangulation->vertices.size();
            triangulation->vertices.reserve(baseVertex + mesh->num_vertices);

            for (size_t v = 0; v < mesh->num_vertices; v++)
            {
                ufbx_vec3 pos = ufbx_transform_position(&node->geometry_to_world, mesh->vertices.data[v]);
                triangulation->vertices.emplace_back(pos.x, pos.y, pos.z);
            }

            size_t maxTriCorners = mesh->max_face_triangles * 3;
            if (triIndices.size() < maxTriCorners)
            {
                triIndices.resize(maxTriCorners);
            }

            bool meshHasUVs = mesh->vertex_uv.exists;
            if (meshHasUVs)
            {
                anyUVs = true;
            }

            for (size_t f = 0; f < mesh->faces.count; f++)
            {
                ufbx_face face         = mesh->faces.data[f];
                uint32_t  numTriangles = ufbx_triangulate_face(triIndices.data(), triIndices.size(), mesh, face);
                size_t    cornerCount  = static_cast<size_t>(numTriangles) * 3;

                for (size_t c = 0; c < cornerCount; c++)
                {
                    uint32_t index       = triIndices[c];
                    uint32_t vertexIndex = mesh->vertex_indices.data[index];
                    triangulation->indices.push_back(baseVertex + static_cast<size_t>(vertexIndex));

                    if (meshHasUVs)
                    {
                        ufbx_vec2 uv = ufbx_get_vertex_vec2(&mesh->vertex_uv, index);
                        cornerUVs.emplace_back(uv.x, 1.0 - uv.y);
                    }
                    else
                    {
                        cornerUVs.emplace_back(0.0, 0.0);
                    }
                }
            }
        }

        if (!triangulation->indices.empty())
        {
            sceneData->setTriangulation(std::move(triangulation));
            if (anyUVs)
            {
                auto uvs = std::make_unique<Texture::TriangulationUVs>(cornerUVs.size());
                for (size_t i = 0; i < cornerUVs.size(); i++)
                {
                    uvs->setUV(i, cornerUVs[i]);
                }
                sceneData->setUVs(std::move(uvs));
            }
        }

        std::unique_ptr<Texture::Texture> diffuseTexture = extractDiffuseTexture(scene, sourceDirectory);
        if (diffuseTexture)
        {
            sceneData->setDiffuseTexture(std::move(diffuseTexture));
        }

        return sceneData;
    }

    std::unique_ptr<Texture::Texture> FbxReader::extractDiffuseTexture(const ufbx_scene* scene, const std::string& sourceDirectory)
    {
        for (size_t i = 0; i < scene->materials.count; i++)
        {
            const ufbx_material* material = scene->materials.data[i];
            if (material->pbr.base_color.texture)
            {
                auto tex = loadUfbxTexture(material->pbr.base_color.texture, sourceDirectory);
                if (tex)
                {
                    return tex;
                }
            }
            if (material->fbx.diffuse_color.texture)
            {
                auto tex = loadUfbxTexture(material->fbx.diffuse_color.texture, sourceDirectory);
                if (tex)
                {
                    return tex;
                }
            }
        }

        for (size_t i = 0; i < scene->textures.count; i++)
        {
            auto tex = loadUfbxTexture(scene->textures.data[i], sourceDirectory);
            if (tex)
            {
                return tex;
            }
        }

        return nullptr;
    }

    std::unique_ptr<Texture::Texture> FbxReader::loadUfbxTexture(const ufbx_texture* texture, const std::string& sourceDirectory)
    {
        if (!texture)
        {
            return nullptr;
        }

        if (texture->content.data && texture->content.size > 0)
        {
            const auto*          rawBytes = static_cast<const uint8_t*>(texture->content.data);
            std::vector<uint8_t> encoded(rawBytes, rawBytes + texture->content.size);
            try
            {
                return std::make_unique<Texture::Texture>(Texture::Texture::loadFromMemory(encoded));
            }
            catch (const std::exception&)
            {
                return nullptr;
            }
        }

        std::vector<std::filesystem::path> candidatePaths;
        if (texture->filename.length > 0)
        {
            candidatePaths.emplace_back(std::string(texture->filename.data, texture->filename.length));
        }
        if (texture->relative_filename.length > 0 && !sourceDirectory.empty())
        {
            candidatePaths.push_back(std::filesystem::path(sourceDirectory) / std::string(texture->relative_filename.data, texture->relative_filename.length));
        }
        if (texture->filename.length > 0 && !sourceDirectory.empty())
        {
            std::filesystem::path rawPath(std::string(texture->filename.data, texture->filename.length));
            candidatePaths.push_back(std::filesystem::path(sourceDirectory) / rawPath.filename());
            candidatePaths.push_back(std::filesystem::path(sourceDirectory) / "tex" / rawPath.filename());
            candidatePaths.push_back(std::filesystem::path(sourceDirectory) / "textures" / rawPath.filename());
        }

        for (const auto& path : candidatePaths)
        {
            if (std::filesystem::exists(path))
            {
                try
                {
                    return std::make_unique<Texture::Texture>(Texture::Texture::loadFromFile(path.string()));
                }
                catch (const std::exception&)
                {
                }
            }
        }

        return nullptr;
    }
}

#ifdef ISTESTPROJECT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("FbxReader/LoadsAsciiTriangleWithUVs")
{
    std::string asciiFbx = "; FBX 7.4.0 project file\n"
                           "FBXHeaderExtension:  {\n"
                           "    FBXHeaderVersion: 1003\n"
                           "    FBXVersion: 7400\n"
                           "}\n"
                           "Objects:  {\n"
                           "    Geometry: 100, \"Geometry::Tri\", \"Mesh\" {\n"
                           "        Vertices: *9 {\n"
                           "            a: 0,0,0, 100,0,0, 0,100,0\n"
                           "        }\n"
                           "        PolygonVertexIndex: *3 {\n"
                           "            a: 0,1,-3\n"
                           "        }\n"
                           "        LayerElementUV: 0 {\n"
                           "            Version: 101\n"
                           "            Name: \"UVMap\"\n"
                           "            MappingInformationType: \"ByPolygonVertex\"\n"
                           "            ReferenceInformationType: \"Direct\"\n"
                           "            UV: *6 {\n"
                           "                a: 0.0,0.0, 1.0,0.0, 0.5,1.0\n"
                           "            }\n"
                           "        }\n"
                           "        Layer: 0 {\n"
                           "            Version: 100\n"
                           "            LayerElement:  {\n"
                           "                Type: \"LayerElementUV\"\n"
                           "                TypedIndex: 0\n"
                           "            }\n"
                           "        }\n"
                           "    }\n"
                           "    Model: 200, \"Model::Tri\", \"Mesh\" {\n"
                           "        Version: 232\n"
                           "    }\n"
                           "}\n"
                           "Connections:  {\n"
                           "    C: \"OO\",100,200\n"
                           "    C: \"OO\",200,0\n"
                           "}\n";

    std::vector<uint8_t> bytes(asciiFbx.begin(), asciiFbx.end());
    auto                 sceneData = CADacomb::IO::FbxReader::loadSceneFromMemory(bytes);
    REQUIRE(sceneData != nullptr);
    REQUIRE(sceneData->hasTriangulation());
    REQUIRE(sceneData->hasUVs());
    REQUIRE(sceneData->getUVs()->getCornerCount() == 3);
    CHECK(sceneData->getUVs()->getUV(0).x == Catch::Approx(0.0));
    CHECK(sceneData->getUVs()->getUV(0).y == Catch::Approx(1.0));
    CHECK(sceneData->getUVs()->getUV(2).x == Catch::Approx(0.5));
    CHECK(sceneData->getUVs()->getUV(2).y == Catch::Approx(0.0));
}
#endif
