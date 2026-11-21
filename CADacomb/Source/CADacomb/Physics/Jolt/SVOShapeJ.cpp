#include "CADacomb/Physics/Jolt/SVOShapeJ.h"

#include "CADacomb/Volume/SVO/SVOColliderQuery.h"

#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <algorithm>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>

namespace CADacomb::Physics
{
    SVOShapeJ::SVOShapeJ(const SVO<bool>& svoInput, const AABB3& localBoundsInput)
      : JPH::Shape(JPH::EShapeType::User1, JPH::EShapeSubType::User1)
      , svo(&svoInput)
      , localBounds(localBoundsInput)
    {
        refreshMassProperties();
    }

    SVOShapeJ::~SVOShapeJ() {}

    void SVOShapeJ::registerCollisionDispatch()
    {
        JPH::ShapeFunctions& funcs = JPH::ShapeFunctions::sGet(JPH::EShapeSubType::User1);
        funcs.mConstruct           = nullptr;
        funcs.mColor               = JPH::Color::sCyan;

        for (JPH::EShapeSubType subType : JPH::sAllSubShapeTypes)
        {
            if (subType == JPH::EShapeSubType::User1)
            {
                JPH::CollisionDispatch::sRegisterCollideShape(JPH::EShapeSubType::User1, JPH::EShapeSubType::User1, collideSVOVsSVO);
                JPH::CollisionDispatch::sRegisterCastShape(JPH::EShapeSubType::User1, JPH::EShapeSubType::User1, castShapeVsSVO);
            }
            else
            {
                JPH::CollisionDispatch::sRegisterCollideShape(JPH::EShapeSubType::User1, subType, collideSVOVsShape);
                JPH::CollisionDispatch::sRegisterCollideShape(subType, JPH::EShapeSubType::User1, JPH::CollisionDispatch::sReversedCollideShape);
                JPH::CollisionDispatch::sRegisterCastShape(subType, JPH::EShapeSubType::User1, castShapeVsSVO);
                JPH::CollisionDispatch::sRegisterCastShape(JPH::EShapeSubType::User1, subType, JPH::CollisionDispatch::sReversedCastShape);
            }
        }
    }

    void SVOShapeJ::refreshMassProperties()
    {
        if (svo)
        {
            massProperties = SVOMassProperties::compute(*svo, localBounds, 1000.0);
        }
    }

    const SVO<bool>* SVOShapeJ::getSVO() const
    {
        return svo;
    }

    const AABB3& SVOShapeJ::getLocalBounds() const
    {
        return localBounds;
    }

    glm::dvec3 SVOShapeJ::getCenterOfMassDouble() const
    {
        return massProperties.getCenterOfMass();
    }

    JPH::Vec3 SVOShapeJ::GetCenterOfMass() const
    {
        const glm::dvec3& com = massProperties.getCenterOfMass();
        return JPH::Vec3(static_cast<float>(com.x), static_cast<float>(com.y), static_cast<float>(com.z));
    }

    JPH::AABox SVOShapeJ::GetLocalBounds() const
    {
        const glm::dvec3& com  = massProperties.getCenterOfMass();
        glm::dvec3        bMin = localBounds.getMin() - com;
        glm::dvec3        bMax = localBounds.getMax() - com;
        return JPH::AABox(JPH::Vec3(static_cast<float>(bMin.x), static_cast<float>(bMin.y), static_cast<float>(bMin.z)),
                          JPH::Vec3(static_cast<float>(bMax.x), static_cast<float>(bMax.y), static_cast<float>(bMax.z)));
    }

    JPH::uint SVOShapeJ::GetSubShapeIDBitsRecursive() const
    {
        return 28;
    }

    float SVOShapeJ::GetInnerRadius() const
    {
        if (!svo || svo->getDimension() == 0)
        {
            return 0.05f;
        }
        glm::dvec3 voxelScale = localBounds.getSize() / static_cast<double>(svo->getDimension());
        double     minSide    = std::min({ voxelScale.x, voxelScale.y, voxelScale.z });
        return std::max(1e-4f, static_cast<float>(minSide * 0.5));
    }

    JPH::MassProperties SVOShapeJ::GetMassProperties() const
    {
        JPH::MassProperties mp;
        mp.mMass                 = static_cast<float>(massProperties.getMass());
        const glm::dmat3& tensor = massProperties.getInertiaTensor();
        mp.mInertia              = JPH::Mat44(JPH::Vec4(static_cast<float>(tensor[0][0]), static_cast<float>(tensor[0][1]), static_cast<float>(tensor[0][2]), 0.0f),
                                              JPH::Vec4(static_cast<float>(tensor[1][0]), static_cast<float>(tensor[1][1]), static_cast<float>(tensor[1][2]), 0.0f),
                                              JPH::Vec4(static_cast<float>(tensor[2][0]), static_cast<float>(tensor[2][1]), static_cast<float>(tensor[2][2]), 0.0f),
                                              JPH::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
        return mp;
    }

    const JPH::PhysicsMaterial* SVOShapeJ::GetMaterial(const JPH::SubShapeID&) const
    {
        return JPH::PhysicsMaterial::sDefault;
    }

    JPH::Vec3 SVOShapeJ::GetSurfaceNormal(const JPH::SubShapeID&, JPH::Vec3Arg inLocalSurfacePosition) const
    {
        JPH::Vec3 absPos = inLocalSurfacePosition.Abs();
        if (absPos.GetX() >= absPos.GetY() && absPos.GetX() >= absPos.GetZ())
        {
            return JPH::Vec3(inLocalSurfacePosition.GetX() >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
        }
        if (absPos.GetY() >= absPos.GetX() && absPos.GetY() >= absPos.GetZ())
        {
            return JPH::Vec3(0.0f, inLocalSurfacePosition.GetY() >= 0.0f ? 1.0f : -1.0f, 0.0f);
        }
        return JPH::Vec3(0.0f, 0.0f, inLocalSurfacePosition.GetZ() >= 0.0f ? 1.0f : -1.0f);
    }

    void SVOShapeJ::GetSubmergedVolume(JPH::Mat44Arg,
                                       JPH::Vec3Arg,
                                       const JPH::Plane&,
                                       float&     outTotalVolume,
                                       float&     outSubmergedVolume,
                                       JPH::Vec3& outCenterOfBuoyancy
#ifdef JPH_DEBUG_RENDERER
                                       ,
                                       JPH::RVec3Arg
#endif
    ) const
    {
        outTotalVolume      = GetVolume();
        outSubmergedVolume  = 0.0f;
        outCenterOfBuoyancy = JPH::Vec3::sZero();
    }

#ifdef JPH_DEBUG_RENDERER
    void SVOShapeJ::Draw(JPH::DebugRenderer*, JPH::RMat44Arg, JPH::Vec3Arg, JPH::ColorArg, bool, bool) const {}
#endif

    bool SVOShapeJ::CastRay(const JPH::RayCast& inRay, const JPH::SubShapeIDCreator& inSubShapeIDCreator, JPH::RayCastResult& ioHit) const
    {
        if (!svo)
        {
            return false;
        }

        const glm::dvec3& com    = massProperties.getCenterOfMass();
        JPH::AABox        rayBox = JPH::AABox::sFromTwoPoints(inRay.mOrigin, inRay.mOrigin + inRay.mDirection * ioHit.mFraction);
        AABB3             queryBox(glm::dvec3(static_cast<double>(rayBox.mMin.GetX()), static_cast<double>(rayBox.mMin.GetY()), static_cast<double>(rayBox.mMin.GetZ())) + com,
                                   glm::dvec3(static_cast<double>(rayBox.mMax.GetX()), static_cast<double>(rayBox.mMax.GetY()), static_cast<double>(rayBox.mMax.GetZ())) + com);

        bool            hitAny = false;
        const JPH::uint idBits = std::min<JPH::uint>(28U, JPH::SubShapeID::MaxBits - inSubShapeIDCreator.GetNumBitsWritten());
        const JPH::uint idMask = idBits > 0 ? ((1U << idBits) - 1U) : 0U;

        SVOColliderQuery::queryOverlappingLeaves(
          *svo,
          localBounds,
          queryBox,
          [&](const AABB3& leafBox, uint32_t leafNodeIdx)
          {
              glm::dvec3 relCenter = leafBox.getCenter() - com;
              glm::dvec3 halfSize  = leafBox.getSize() * 0.5;
              JPH::Vec3  halfExt(std::max(1e-4f, static_cast<float>(halfSize.x)), std::max(1e-4f, static_cast<float>(halfSize.y)), std::max(1e-4f, static_cast<float>(halfSize.z)));
              float      convexRad = std::min({ JPH::cDefaultConvexRadius, halfExt.GetX() * 0.5f, halfExt.GetY() * 0.5f, halfExt.GetZ() * 0.5f });

              JPH::SubShapeIDCreator leafId = idBits > 0 ? inSubShapeIDCreator.PushID(leafNodeIdx & idMask, idBits) : inSubShapeIDCreator;
              JPH::BoxShape          leafShape(halfExt, convexRad);
              JPH::RayCast           localRay(inRay.mOrigin - JPH::Vec3(static_cast<float>(relCenter.x), static_cast<float>(relCenter.y), static_cast<float>(relCenter.z)), inRay.mDirection);
              if (leafShape.CastRay(localRay, leafId, ioHit))
              {
                  hitAny = true;
              }
          });
        return hitAny;
    }

    void SVOShapeJ::CastRay(const JPH::RayCast&           inRay,
                            const JPH::RayCastSettings&   inRayCastSettings,
                            const JPH::SubShapeIDCreator& inSubShapeIDCreator,
                            JPH::CastRayCollector&        ioCollector,
                            const JPH::ShapeFilter&       inShapeFilter) const
    {
        if (!svo || !inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
        {
            return;
        }

        const glm::dvec3& com    = massProperties.getCenterOfMass();
        JPH::AABox        rayBox = JPH::AABox::sFromTwoPoints(inRay.mOrigin, inRay.mOrigin + inRay.mDirection);
        AABB3             queryBox(glm::dvec3(static_cast<double>(rayBox.mMin.GetX()), static_cast<double>(rayBox.mMin.GetY()), static_cast<double>(rayBox.mMin.GetZ())) + com,
                                   glm::dvec3(static_cast<double>(rayBox.mMax.GetX()), static_cast<double>(rayBox.mMax.GetY()), static_cast<double>(rayBox.mMax.GetZ())) + com);

        const JPH::uint idBits = std::min<JPH::uint>(28U, JPH::SubShapeID::MaxBits - inSubShapeIDCreator.GetNumBitsWritten());
        const JPH::uint idMask = idBits > 0 ? ((1U << idBits) - 1U) : 0U;

        SVOColliderQuery::queryOverlappingLeaves(
          *svo,
          localBounds,
          queryBox,
          [&](const AABB3& leafBox, uint32_t leafNodeIdx)
          {
              glm::dvec3 relCenter = leafBox.getCenter() - com;
              glm::dvec3 halfSize  = leafBox.getSize() * 0.5;
              JPH::Vec3  halfExt(std::max(1e-4f, static_cast<float>(halfSize.x)), std::max(1e-4f, static_cast<float>(halfSize.y)), std::max(1e-4f, static_cast<float>(halfSize.z)));
              float      convexRad = std::min({ JPH::cDefaultConvexRadius, halfExt.GetX() * 0.5f, halfExt.GetY() * 0.5f, halfExt.GetZ() * 0.5f });

              JPH::SubShapeIDCreator leafId = idBits > 0 ? inSubShapeIDCreator.PushID(leafNodeIdx & idMask, idBits) : inSubShapeIDCreator;
              JPH::BoxShape          leafShape(halfExt, convexRad);
              JPH::RayCast           localRay(inRay.mOrigin - JPH::Vec3(static_cast<float>(relCenter.x), static_cast<float>(relCenter.y), static_cast<float>(relCenter.z)), inRay.mDirection);
              leafShape.CastRay(localRay, inRayCastSettings, leafId, ioCollector, inShapeFilter);
          });
    }

    void SVOShapeJ::CollidePoint(JPH::Vec3Arg inPoint, const JPH::SubShapeIDCreator& inSubShapeIDCreator, JPH::CollidePointCollector& ioCollector, const JPH::ShapeFilter& inShapeFilter) const
    {
        if (!svo || !inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
        {
            return;
        }

        const glm::dvec3& com = massProperties.getCenterOfMass();
        glm::dvec3        pt  = glm::dvec3(static_cast<double>(inPoint.GetX()), static_cast<double>(inPoint.GetY()), static_cast<double>(inPoint.GetZ())) + com;
        AABB3             queryBox(pt, pt);

        const JPH::uint idBits = std::min<JPH::uint>(28U, JPH::SubShapeID::MaxBits - inSubShapeIDCreator.GetNumBitsWritten());
        const JPH::uint idMask = idBits > 0 ? ((1U << idBits) - 1U) : 0U;

        SVOColliderQuery::queryOverlappingLeaves(*svo,
                                                 localBounds,
                                                 queryBox,
                                                 [&](const AABB3&, uint32_t leafNodeIdx)
                                                 {
                                                     JPH::SubShapeIDCreator leafId = idBits > 0 ? inSubShapeIDCreator.PushID(leafNodeIdx & idMask, idBits) : inSubShapeIDCreator;
                                                     ioCollector.AddHit(JPH::CollidePointResult(JPH::BodyID(), leafId.GetID()));
                                                 });
    }

    void SVOShapeJ::CollideSoftBodyVertices(JPH::Mat44Arg, JPH::Vec3Arg, const JPH::CollideSoftBodyVertexIterator&, JPH::uint, int) const {}

    void SVOShapeJ::GetTrianglesStart(GetTrianglesContext&, const JPH::AABox&, JPH::Vec3Arg, JPH::QuatArg, JPH::Vec3Arg) const {}

    int SVOShapeJ::GetTrianglesNext(GetTrianglesContext&, int, JPH::Float3*, const JPH::PhysicsMaterial**) const
    {
        return 0;
    }

    JPH::Shape::Stats SVOShapeJ::GetStats() const
    {
        return Stats(sizeof(SVOShapeJ), 0);
    }

    float SVOShapeJ::GetVolume() const
    {
        return static_cast<float>(massProperties.getMass() / 1000.0);
    }

    void SVOShapeJ::collideSVOVsShape(const JPH::Shape*                inShape1,
                                      const JPH::Shape*                inShape2,
                                      JPH::Vec3Arg                     inScale1,
                                      JPH::Vec3Arg                     inScale2,
                                      JPH::Mat44Arg                    inCenterOfMassTransform1,
                                      JPH::Mat44Arg                    inCenterOfMassTransform2,
                                      const JPH::SubShapeIDCreator&    inSubShapeIDCreator1,
                                      const JPH::SubShapeIDCreator&    inSubShapeIDCreator2,
                                      const JPH::CollideShapeSettings& inCollideShapeSettings,
                                      JPH::CollideShapeCollector&      ioCollector,
                                      const JPH::ShapeFilter&          inShapeFilter)
    {
        const auto* svoShape = static_cast<const SVOShapeJ*>(inShape1);
        if (!svoShape->svo)
        {
            return;
        }

        JPH::Mat44 transform2To1 = inCenterOfMassTransform1.InversedRotationTranslation() * inCenterOfMassTransform2;
        JPH::AABox bounds2In1    = inShape2->GetWorldSpaceBounds(transform2To1, inScale2);
        bounds2In1.ExpandBy(JPH::Vec3::sReplicate(inCollideShapeSettings.mMaxSeparationDistance));

        const glm::dvec3& com = svoShape->massProperties.getCenterOfMass();
        glm::dvec3        invScale1(1.0 / static_cast<double>(inScale1.GetX()), 1.0 / static_cast<double>(inScale1.GetY()), 1.0 / static_cast<double>(inScale1.GetZ()));
        glm::dvec3        bMin(static_cast<double>(bounds2In1.mMin.GetX()), static_cast<double>(bounds2In1.mMin.GetY()), static_cast<double>(bounds2In1.mMin.GetZ()));
        glm::dvec3        bMax(static_cast<double>(bounds2In1.mMax.GetX()), static_cast<double>(bounds2In1.mMax.GetY()), static_cast<double>(bounds2In1.mMax.GetZ()));
        glm::dvec3        p0 = bMin * invScale1 + com;
        glm::dvec3        p1 = bMax * invScale1 + com;
        AABB3             queryBox(glm::min(p0, p1), glm::max(p0, p1));

        const JPH::uint idBits1 = std::min<JPH::uint>(28U, JPH::SubShapeID::MaxBits - inSubShapeIDCreator1.GetNumBitsWritten());
        const JPH::uint idMask1 = idBits1 > 0 ? ((1U << idBits1) - 1U) : 0U;

        SVOColliderQuery::queryOverlappingLeaves(
          *svoShape->svo,
          svoShape->localBounds,
          queryBox,
          [&](const AABB3& leafBox, uint32_t leafNodeIdx)
          {
              glm::dvec3 relCenter = leafBox.getCenter() - com;
              JPH::Vec3  scaledOffset(static_cast<float>(relCenter.x) * inScale1.GetX(), static_cast<float>(relCenter.y) * inScale1.GetY(), static_cast<float>(relCenter.z) * inScale1.GetZ());

              glm::dvec3 halfSize = leafBox.getSize() * 0.5;
              JPH::Vec3  halfExt(std::max(1e-4f, static_cast<float>(halfSize.x)), std::max(1e-4f, static_cast<float>(halfSize.y)), std::max(1e-4f, static_cast<float>(halfSize.z)));
              float      convexRad = std::min({ JPH::cDefaultConvexRadius, halfExt.GetX() * 0.5f, halfExt.GetY() * 0.5f, halfExt.GetZ() * 0.5f });

              JPH::SubShapeIDCreator leafId1 = idBits1 > 0 ? inSubShapeIDCreator1.PushID(leafNodeIdx & idMask1, idBits1) : inSubShapeIDCreator1;
              JPH::BoxShape          leafBoxShape(halfExt, convexRad);
              JPH::Mat44             leafTransform1 = inCenterOfMassTransform1 * JPH::Mat44::sTranslation(scaledOffset);

              JPH::CollisionDispatch::sCollideShapeVsShape(
                &leafBoxShape, inShape2, inScale1, inScale2, leafTransform1, inCenterOfMassTransform2, leafId1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
          });
    }

    void SVOShapeJ::collideSVOVsSVO(const JPH::Shape*                inShape1,
                                    const JPH::Shape*                inShape2,
                                    JPH::Vec3Arg                     inScale1,
                                    JPH::Vec3Arg                     inScale2,
                                    JPH::Mat44Arg                    inCenterOfMassTransform1,
                                    JPH::Mat44Arg                    inCenterOfMassTransform2,
                                    const JPH::SubShapeIDCreator&    inSubShapeIDCreator1,
                                    const JPH::SubShapeIDCreator&    inSubShapeIDCreator2,
                                    const JPH::CollideShapeSettings& inCollideShapeSettings,
                                    JPH::CollideShapeCollector&      ioCollector,
                                    const JPH::ShapeFilter&          inShapeFilter)
    {
        const auto* svoShape1 = static_cast<const SVOShapeJ*>(inShape1);
        const auto* svoShape2 = static_cast<const SVOShapeJ*>(inShape2);
        if (!svoShape1->svo || !svoShape2->svo)
        {
            return;
        }

        const glm::dvec3& com1 = svoShape1->massProperties.getCenterOfMass();
        const glm::dvec3& com2 = svoShape2->massProperties.getCenterOfMass();

        glm::dmat4 world1        = toGlmMat4(inCenterOfMassTransform1);
        glm::dmat4 world2        = toGlmMat4(inCenterOfMassTransform2);
        glm::dmat4 scaleMat1     = glm::scale(glm::dmat4(1.0), glm::dvec3(static_cast<double>(inScale1.GetX()), static_cast<double>(inScale1.GetY()), static_cast<double>(inScale1.GetZ())));
        glm::dmat4 scaleMat2     = glm::scale(glm::dmat4(1.0), glm::dvec3(static_cast<double>(inScale2.GetX()), static_cast<double>(inScale2.GetY()), static_cast<double>(inScale2.GetZ())));
        glm::dmat4 local1ToWorld = world1 * scaleMat1 * glm::translate(glm::dmat4(1.0), -com1);
        glm::dmat4 local2ToWorld = world2 * scaleMat2 * glm::translate(glm::dmat4(1.0), -com2);
        glm::dmat4 transformBToA = glm::inverse(local1ToWorld) * local2ToWorld;

        const JPH::uint idBits1 = std::min<JPH::uint>(28U, JPH::SubShapeID::MaxBits - inSubShapeIDCreator1.GetNumBitsWritten());
        const JPH::uint idMask1 = idBits1 > 0 ? ((1U << idBits1) - 1U) : 0U;
        const JPH::uint idBits2 = std::min<JPH::uint>(28U, JPH::SubShapeID::MaxBits - inSubShapeIDCreator2.GetNumBitsWritten());
        const JPH::uint idMask2 = idBits2 > 0 ? ((1U << idBits2) - 1U) : 0U;

        SVOColliderQuery::queryOverlappingLeafPairs(
          *svoShape1->svo,
          svoShape1->localBounds,
          *svoShape2->svo,
          svoShape2->localBounds,
          transformBToA,
          [&](const AABB3& leafA, uint32_t leafNodeIdxA, const AABB3& leafB, uint32_t leafNodeIdxB)
          {
              glm::dvec3 relCenterA = leafA.getCenter() - com1;
              JPH::Vec3  scaledOffA(static_cast<float>(relCenterA.x) * inScale1.GetX(), static_cast<float>(relCenterA.y) * inScale1.GetY(), static_cast<float>(relCenterA.z) * inScale1.GetZ());
              glm::dvec3 halfSizeA = leafA.getSize() * 0.5;
              JPH::Vec3  halfExtA(std::max(1e-4f, static_cast<float>(halfSizeA.x)), std::max(1e-4f, static_cast<float>(halfSizeA.y)), std::max(1e-4f, static_cast<float>(halfSizeA.z)));
              float      convexRadA = std::min({ JPH::cDefaultConvexRadius, halfExtA.GetX() * 0.5f, halfExtA.GetY() * 0.5f, halfExtA.GetZ() * 0.5f });

              glm::dvec3 relCenterB = leafB.getCenter() - com2;
              JPH::Vec3  scaledOffB(static_cast<float>(relCenterB.x) * inScale2.GetX(), static_cast<float>(relCenterB.y) * inScale2.GetY(), static_cast<float>(relCenterB.z) * inScale2.GetZ());
              glm::dvec3 halfSizeB = leafB.getSize() * 0.5;
              JPH::Vec3  halfExtB(std::max(1e-4f, static_cast<float>(halfSizeB.x)), std::max(1e-4f, static_cast<float>(halfSizeB.y)), std::max(1e-4f, static_cast<float>(halfSizeB.z)));
              float      convexRadB = std::min({ JPH::cDefaultConvexRadius, halfExtB.GetX() * 0.5f, halfExtB.GetY() * 0.5f, halfExtB.GetZ() * 0.5f });

              JPH::SubShapeIDCreator leafId1 = idBits1 > 0 ? inSubShapeIDCreator1.PushID(leafNodeIdxA & idMask1, idBits1) : inSubShapeIDCreator1;
              JPH::SubShapeIDCreator leafId2 = idBits2 > 0 ? inSubShapeIDCreator2.PushID(leafNodeIdxB & idMask2, idBits2) : inSubShapeIDCreator2;

              JPH::BoxShape boxShapeA(halfExtA, convexRadA);
              JPH::BoxShape boxShapeB(halfExtB, convexRadB);

              JPH::Mat44 leafTransform1 = inCenterOfMassTransform1 * JPH::Mat44::sTranslation(scaledOffA);
              JPH::Mat44 leafTransform2 = inCenterOfMassTransform2 * JPH::Mat44::sTranslation(scaledOffB);

              JPH::CollisionDispatch::sCollideShapeVsShape(
                &boxShapeA, &boxShapeB, inScale1, inScale2, leafTransform1, leafTransform2, leafId1, leafId2, inCollideShapeSettings, ioCollector, inShapeFilter);
          });
    }

    void SVOShapeJ::castShapeVsSVO(const JPH::ShapeCast&         inShapeCast,
                                   const JPH::ShapeCastSettings& inShapeCastSettings,
                                   const JPH::Shape*             inShape,
                                   JPH::Vec3Arg                  inScale,
                                   const JPH::ShapeFilter&       inShapeFilter,
                                   JPH::Mat44Arg                 inCenterOfMassTransform2,
                                   const JPH::SubShapeIDCreator& inSubShapeIDCreator1,
                                   const JPH::SubShapeIDCreator& inSubShapeIDCreator2,
                                   JPH::CastShapeCollector&      ioCollector)
    {
        const auto* svoShape = static_cast<const SVOShapeJ*>(inShape);
        if (!svoShape->svo)
        {
            return;
        }

        JPH::AABox sweptBounds = inShapeCast.mShapeWorldBounds;
        JPH::AABox endBounds   = inShapeCast.mShapeWorldBounds;
        endBounds.Translate(inShapeCast.mDirection);
        sweptBounds.Encapsulate(endBounds);

        const glm::dvec3& com = svoShape->massProperties.getCenterOfMass();
        glm::dvec3        invScale(1.0 / static_cast<double>(inScale.GetX()), 1.0 / static_cast<double>(inScale.GetY()), 1.0 / static_cast<double>(inScale.GetZ()));
        glm::dvec3        bMin(static_cast<double>(sweptBounds.mMin.GetX()), static_cast<double>(sweptBounds.mMin.GetY()), static_cast<double>(sweptBounds.mMin.GetZ()));
        glm::dvec3        bMax(static_cast<double>(sweptBounds.mMax.GetX()), static_cast<double>(sweptBounds.mMax.GetY()), static_cast<double>(sweptBounds.mMax.GetZ()));
        glm::dvec3        p0 = bMin * invScale + com;
        glm::dvec3        p1 = bMax * invScale + com;
        AABB3             queryBox(glm::min(p0, p1), glm::max(p0, p1));

        const JPH::uint idBits2 = std::min<JPH::uint>(28U, JPH::SubShapeID::MaxBits - inSubShapeIDCreator2.GetNumBitsWritten());
        const JPH::uint idMask2 = idBits2 > 0 ? ((1U << idBits2) - 1U) : 0U;

        SVOColliderQuery::queryOverlappingLeaves(
          *svoShape->svo,
          svoShape->localBounds,
          queryBox,
          [&](const AABB3& leafBox, uint32_t leafNodeIdx)
          {
              glm::dvec3 relCenter = leafBox.getCenter() - com;
              JPH::Vec3  scaledOffset(static_cast<float>(relCenter.x) * inScale.GetX(), static_cast<float>(relCenter.y) * inScale.GetY(), static_cast<float>(relCenter.z) * inScale.GetZ());

              glm::dvec3 halfSize = leafBox.getSize() * 0.5;
              JPH::Vec3  halfExt(std::max(1e-4f, static_cast<float>(halfSize.x)), std::max(1e-4f, static_cast<float>(halfSize.y)), std::max(1e-4f, static_cast<float>(halfSize.z)));
              float      convexRad = std::min({ JPH::cDefaultConvexRadius, halfExt.GetX() * 0.5f, halfExt.GetY() * 0.5f, halfExt.GetZ() * 0.5f });

              JPH::SubShapeIDCreator leafId2 = idBits2 > 0 ? inSubShapeIDCreator2.PushID(leafNodeIdx & idMask2, idBits2) : inSubShapeIDCreator2;
              JPH::BoxShape          leafBoxShape(halfExt, convexRad);
              JPH::ShapeCast         leafCast      = inShapeCast.PostTransformed(JPH::Mat44::sTranslation(-scaledOffset));
              JPH::Mat44             leafTransform = inCenterOfMassTransform2 * JPH::Mat44::sTranslation(scaledOffset);

              JPH::CollisionDispatch::sCastShapeVsShapeLocalSpace(leafCast, inShapeCastSettings, &leafBoxShape, inScale, inShapeFilter, leafTransform, inSubShapeIDCreator1, leafId2, ioCollector);
          });
    }

    glm::dmat4 SVOShapeJ::toGlmMat4(JPH::Mat44Arg mat)
    {
        glm::dmat4 res(1.0);
        for (int c = 0; c < 4; c++)
        {
            JPH::Vec4 col = mat.GetColumn4(c);
            res[c][0]     = static_cast<double>(col.GetX());
            res[c][1]     = static_cast<double>(col.GetY());
            res[c][2]     = static_cast<double>(col.GetZ());
            res[c][3]     = static_cast<double>(col.GetW());
        }
        return res;
    }
}
