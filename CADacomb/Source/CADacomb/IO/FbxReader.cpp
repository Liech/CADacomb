#include "CADacomb/IO/FbxReader.h"

#include <algorithm>
#include <filesystem>
#include <ufbx.h>
#include <unordered_set>

namespace CADacomb::IO
{
    FbxReader::FbxReader() {}

    FbxReader::~FbxReader() {}

    std::unique_ptr<FbxSceneData> FbxReader::loadSceneFromFile(const std::string& filename)
    {
        ufbx_load_opts opts     = {};
        opts.target_axes        = ufbx_axes_right_handed_y_up;
        opts.target_unit_meters = 1.0;
        opts.space_conversion   = UFBX_SPACE_CONVERSION_MODIFY_GEOMETRY;

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
        opts.space_conversion   = UFBX_SPACE_CONVERSION_MODIFY_GEOMETRY;

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
        auto                                             sceneData     = std::make_unique<FbxSceneData>();
        auto                                             triangulation = std::make_unique<Triangulation>();
        std::vector<glm::dvec2>                          cornerUVs;
        bool                                             anyUVs = false;
        std::vector<uint32_t>                            triIndices;
        std::vector<std::pair<const ufbx_node*, size_t>> meshBaseVertices;

        for (size_t nodeIdx = 0; nodeIdx < scene->nodes.count; nodeIdx++)
        {
            const ufbx_node* node = scene->nodes.data[nodeIdx];
            if (!node->mesh)
            {
                continue;
            }

            const ufbx_mesh* mesh       = node->mesh;
            size_t           baseVertex = triangulation->vertices.size();
            meshBaseVertices.emplace_back(node, baseVertex);
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

        size_t totalVertexCount = triangulation->vertices.size();
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

        std::unordered_map<uint32_t, int> nodeIdToJointIndex;
        extractSkeletonAndSkinning(scene, meshBaseVertices, totalVertexCount, *sceneData, nodeIdToJointIndex);
        if (sceneData->hasSkeleton())
        {
            extractAnimationClips(scene, *sceneData->getSkeleton(), nodeIdToJointIndex, *sceneData);
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

    void FbxReader::extractSkeletonAndSkinning(const ufbx_scene*                                       scene,
                                               const std::vector<std::pair<const ufbx_node*, size_t>>& meshBaseVertices,
                                               size_t                                                  totalVertexCount,
                                               FbxSceneData&                                           sceneData,
                                               std::unordered_map<uint32_t, int>&                      outNodeIdToJointIndex)
    {
        std::unordered_set<uint32_t> skeletonNodeIds;

        for (size_t i = 0; i < scene->nodes.count; i++)
        {
            const ufbx_node* node = scene->nodes.data[i];
            if (node->bone)
            {
                skeletonNodeIds.insert(node->typed_id);
            }
        }

        for (const auto& pair : meshBaseVertices)
        {
            const ufbx_mesh* mesh = pair.first->mesh;
            if (!mesh || mesh->skin_deformers.count == 0)
            {
                continue;
            }
            const ufbx_skin_deformer* skin = mesh->skin_deformers.data[0];
            for (size_t c = 0; c < skin->clusters.count; c++)
            {
                const ufbx_skin_cluster* cluster = skin->clusters.data[c];
                if (cluster->bone_node)
                {
                    skeletonNodeIds.insert(cluster->bone_node->typed_id);
                }
            }
        }

        if (skeletonNodeIds.empty())
        {
            return;
        }

        std::vector<uint32_t> seedIds(skeletonNodeIds.begin(), skeletonNodeIds.end());
        for (uint32_t nodeId : seedIds)
        {
            const ufbx_node* parent = scene->nodes.data[nodeId]->parent;
            while (parent && !parent->is_root)
            {
                skeletonNodeIds.insert(parent->typed_id);
                parent = parent->parent;
            }
        }

        auto skeleton = std::make_unique<Animation::Skeleton>();
        for (size_t i = 0; i < scene->nodes.count; i++)
        {
            const ufbx_node* node = scene->nodes.data[i];
            if (skeletonNodeIds.find(node->typed_id) == skeletonNodeIds.end())
            {
                continue;
            }

            int parentIndex = -1;
            if (node->parent && !node->parent->is_root)
            {
                auto it = outNodeIdToJointIndex.find(node->parent->typed_id);
                if (it != outNodeIdToJointIndex.end())
                {
                    parentIndex = it->second;
                }
            }

            std::string      jointName(node->name.data, node->name.length);
            Animation::Joint joint(jointName, parentIndex, toTransform(node->local_transform));
            int              jointIndex           = skeleton->addJoint(joint);
            outNodeIdToJointIndex[node->typed_id] = jointIndex;
        }

        skeleton->computeInverseBindMatrices();

        bool hasAnySkinning = false;
        auto skinning       = std::make_unique<Animation::TriangulationSkinning>(totalVertexCount);

        for (const auto& pair : meshBaseVertices)
        {
            const ufbx_node* meshNode   = pair.first;
            size_t           baseVertex = pair.second;
            const ufbx_mesh* mesh       = meshNode->mesh;
            if (!mesh || mesh->skin_deformers.count == 0)
            {
                continue;
            }

            const ufbx_skin_deformer* skin           = mesh->skin_deformers.data[0];
            glm::dmat4                invGeomToWorld = glm::inverse(toGlmMatrix(meshNode->geometry_to_world));

            for (size_t c = 0; c < skin->clusters.count; c++)
            {
                const ufbx_skin_cluster* cluster = skin->clusters.data[c];
                if (!cluster->bone_node)
                {
                    continue;
                }

                auto it = outNodeIdToJointIndex.find(cluster->bone_node->typed_id);
                if (it != outNodeIdToJointIndex.end())
                {
                    glm::dmat4 worldToBone = toGlmMatrix(cluster->geometry_to_bone) * invGeomToWorld;
                    skeleton->getJoint(static_cast<size_t>(it->second)).setInverseBindMatrix(worldToBone);
                }
            }

            size_t vertexCount = std::min<size_t>(mesh->num_vertices, skin->vertices.count);
            for (size_t v = 0; v < vertexCount; v++)
            {
                ufbx_skin_vertex sv = skin->vertices.data[v];
                for (uint32_t k = 0; k < sv.num_weights; k++)
                {
                    ufbx_skin_weight         sw      = skin->weights.data[sv.weight_begin + k];
                    const ufbx_skin_cluster* cluster = skin->clusters.data[sw.cluster_index];
                    if (!cluster->bone_node || sw.weight <= 0.0)
                    {
                        continue;
                    }

                    auto it = outNodeIdToJointIndex.find(cluster->bone_node->typed_id);
                    if (it != outNodeIdToJointIndex.end())
                    {
                        skinning->getVertexWeights(baseVertex + v).addInfluence(static_cast<uint32_t>(it->second), sw.weight);
                        hasAnySkinning = true;
                    }
                }
            }
        }

        sceneData.setSkeleton(std::move(skeleton));
        if (hasAnySkinning)
        {
            skinning->normalizeAll();
            sceneData.setSkinning(std::move(skinning));
        }
    }

    void FbxReader::extractAnimationClips(const ufbx_scene* scene, const Animation::Skeleton& skeleton, const std::unordered_map<uint32_t, int>& nodeIdToJointIndex, FbxSceneData& sceneData)
    {
        for (size_t s = 0; s < scene->anim_stacks.count; s++)
        {
            const ufbx_anim_stack* stack    = scene->anim_stacks.data[s];
            ufbx_bake_opts         bakeOpts = {};
            ufbx_error             error    = {};
            ufbx_baked_anim*       baked    = ufbx_bake_anim(scene, stack->anim, &bakeOpts, &error);
            if (!baked)
            {
                continue;
            }

            std::string              clipName(stack->name.data, stack->name.length);
            Animation::AnimationClip clip(clipName);
            double                   timeBegin = baked->playback_time_begin;

            for (size_t n = 0; n < baked->nodes.count; n++)
            {
                const ufbx_baked_node& bakedNode = baked->nodes.data[n];
                auto                   it        = nodeIdToJointIndex.find(bakedNode.typed_id);
                if (it == nodeIdToJointIndex.end())
                {
                    continue;
                }

                const std::string&    jointName = skeleton.getJoint(static_cast<size_t>(it->second)).getName();
                Animation::JointTrack track(jointName);

                for (size_t k = 0; k < bakedNode.translation_keys.count; k++)
                {
                    const ufbx_baked_vec3& key  = bakedNode.translation_keys.data[k];
                    double                 time = std::max(0.0, key.time - timeBegin);
                    track.getTranslationTrack().addKeyframe(time, glm::dvec3(key.value.x, key.value.y, key.value.z));
                }

                for (size_t k = 0; k < bakedNode.rotation_keys.count; k++)
                {
                    const ufbx_baked_quat& key  = bakedNode.rotation_keys.data[k];
                    double                 time = std::max(0.0, key.time - timeBegin);
                    track.getRotationTrack().addKeyframe(time, glm::dquat(key.value.w, key.value.x, key.value.y, key.value.z));
                }

                for (size_t k = 0; k < bakedNode.scale_keys.count; k++)
                {
                    const ufbx_baked_vec3& key  = bakedNode.scale_keys.data[k];
                    double                 time = std::max(0.0, key.time - timeBegin);
                    track.getScaleTrack().addKeyframe(time, glm::dvec3(key.value.x, key.value.y, key.value.z));
                }

                if (!track.getTranslationTrack().isEmpty() || !track.getRotationTrack().isEmpty() || !track.getScaleTrack().isEmpty())
                {
                    clip.addTrack(track);
                }
            }

            if (baked->playback_duration > 0.0)
            {
                clip.setDuration(baked->playback_duration);
            }
            else
            {
                clip.recomputeDuration();
            }

            if (!clip.getTracks().empty())
            {
                sceneData.addAnimationClip(clip);
            }

            ufbx_free_baked_anim(baked);
        }
    }

    glm::dmat4 FbxReader::toGlmMatrix(const ufbx_matrix& matrix)
    {
        glm::dmat4 result(1.0);
        result[0] = glm::dvec4(matrix.cols[0].x, matrix.cols[0].y, matrix.cols[0].z, 0.0);
        result[1] = glm::dvec4(matrix.cols[1].x, matrix.cols[1].y, matrix.cols[1].z, 0.0);
        result[2] = glm::dvec4(matrix.cols[2].x, matrix.cols[2].y, matrix.cols[2].z, 0.0);
        result[3] = glm::dvec4(matrix.cols[3].x, matrix.cols[3].y, matrix.cols[3].z, 1.0);
        return result;
    }

    Animation::Transform FbxReader::toTransform(const ufbx_transform& transform)
    {
        glm::dvec3 translation(transform.translation.x, transform.translation.y, transform.translation.z);
        glm::dquat rotation(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
        glm::dvec3 scale(transform.scale.x, transform.scale.y, transform.scale.z);
        return Animation::Transform(translation, rotation, scale);
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
