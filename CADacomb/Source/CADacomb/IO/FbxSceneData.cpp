#include "CADacomb/IO/FbxSceneData.h"

namespace CADacomb::IO
{
    FbxSceneData::FbxSceneData() {}

    FbxSceneData::~FbxSceneData() {}

    bool FbxSceneData::hasTriangulation() const
    {
        return triangulation != nullptr;
    }

    const Triangulation* FbxSceneData::getTriangulation() const
    {
        return triangulation.get();
    }

    void FbxSceneData::setTriangulation(std::unique_ptr<Triangulation> newTriangulation)
    {
        triangulation = std::move(newTriangulation);
    }

    std::unique_ptr<Triangulation> FbxSceneData::takeTriangulation()
    {
        return std::move(triangulation);
    }

    bool FbxSceneData::hasUVs() const
    {
        return uvs != nullptr;
    }

    const Texture::TriangulationUVs* FbxSceneData::getUVs() const
    {
        return uvs.get();
    }

    void FbxSceneData::setUVs(std::unique_ptr<Texture::TriangulationUVs> newUvs)
    {
        uvs = std::move(newUvs);
    }

    std::unique_ptr<Texture::TriangulationUVs> FbxSceneData::takeUVs()
    {
        return std::move(uvs);
    }

    bool FbxSceneData::hasDiffuseTexture() const
    {
        return diffuseTexture != nullptr;
    }

    const Texture::Texture* FbxSceneData::getDiffuseTexture() const
    {
        return diffuseTexture.get();
    }

    void FbxSceneData::setDiffuseTexture(std::unique_ptr<Texture::Texture> newTexture)
    {
        diffuseTexture = std::move(newTexture);
    }

    std::unique_ptr<Texture::Texture> FbxSceneData::takeDiffuseTexture()
    {
        return std::move(diffuseTexture);
    }
}
