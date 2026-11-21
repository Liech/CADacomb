#pragma once

#include "CADacomb/Util/AABB3.h"
#include "CADacomb/Volume/SVO/SVO.h"
#include "CADacomb/Volume/SVO/SVOMassProperties.h"

#include <Jolt/Jolt.h>
#include <glm/glm.hpp>

#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>

namespace CADacomb::Physics
{
    // Custom Jolt shape backed directly by a live SVO<bool> octree without triangle conversion.
    class SVOShapeJ : public JPH::Shape
    {
      public:
        JPH_OVERRIDE_NEW_DELETE

        SVOShapeJ(const SVO<bool>& svo, const AABB3& localBounds);
        virtual ~SVOShapeJ();

        static void registerCollisionDispatch();

        void             refreshMassProperties();
        const SVO<bool>* getSVO() const;
        const AABB3&     getLocalBounds() const;
        glm::dvec3       getCenterOfMassDouble() const;

        JPH::Vec3                   GetCenterOfMass() const override;
        JPH::AABox                  GetLocalBounds() const override;
        JPH::uint                   GetSubShapeIDBitsRecursive() const override;
        float                       GetInnerRadius() const override;
        JPH::MassProperties         GetMassProperties() const override;
        const JPH::PhysicsMaterial* GetMaterial(const JPH::SubShapeID& inSubShapeID) const override;
        JPH::Vec3                   GetSurfaceNormal(const JPH::SubShapeID& inSubShapeID, JPH::Vec3Arg inLocalSurfacePosition) const override;
        void                        GetSubmergedVolume(JPH::Mat44Arg     inCenterOfMassTransform,
                                                       JPH::Vec3Arg      inScale,
                                                       const JPH::Plane& inSurface,
                                                       float&            outTotalVolume,
                                                       float&            outSubmergedVolume,
                                                       JPH::Vec3&        outCenterOfBuoyancy
#ifdef JPH_DEBUG_RENDERER
                                ,
                                JPH::RVec3Arg inBaseOffset
#endif
        ) const override;
#ifdef JPH_DEBUG_RENDERER
        void Draw(JPH::DebugRenderer* inRenderer, JPH::RMat44Arg inCenterOfMassTransform, JPH::Vec3Arg inScale, JPH::ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif
        bool  CastRay(const JPH::RayCast& inRay, const JPH::SubShapeIDCreator& inSubShapeIDCreator, JPH::RayCastResult& ioHit) const override;
        void  CastRay(const JPH::RayCast&           inRay,
                      const JPH::RayCastSettings&   inRayCastSettings,
                      const JPH::SubShapeIDCreator& inSubShapeIDCreator,
                      JPH::CastRayCollector&        ioCollector,
                      const JPH::ShapeFilter&       inShapeFilter = {}) const override;
        void  CollidePoint(JPH::Vec3Arg inPoint, const JPH::SubShapeIDCreator& inSubShapeIDCreator, JPH::CollidePointCollector& ioCollector, const JPH::ShapeFilter& inShapeFilter = {}) const override;
        void  CollideSoftBodyVertices(JPH::Mat44Arg                             inCenterOfMassTransform,
                                      JPH::Vec3Arg                              inScale,
                                      const JPH::CollideSoftBodyVertexIterator& inVertices,
                                      JPH::uint                                 inNumVertices,
                                      int                                       inCollidingShapeIndex) const override;
        void  GetTrianglesStart(GetTrianglesContext& ioContext, const JPH::AABox& inBox, JPH::Vec3Arg inPositionCOM, JPH::QuatArg inRotation, JPH::Vec3Arg inScale) const override;
        int   GetTrianglesNext(GetTrianglesContext& ioContext, int inMaxTrianglesRequested, JPH::Float3* outTriangleVertices, const JPH::PhysicsMaterial** outMaterials = nullptr) const override;
        Stats GetStats() const override;
        float GetVolume() const override;

      private:
        static void collideSVOVsShape(const JPH::Shape*                inShape1,
                                      const JPH::Shape*                inShape2,
                                      JPH::Vec3Arg                     inScale1,
                                      JPH::Vec3Arg                     inScale2,
                                      JPH::Mat44Arg                    inCenterOfMassTransform1,
                                      JPH::Mat44Arg                    inCenterOfMassTransform2,
                                      const JPH::SubShapeIDCreator&    inSubShapeIDCreator1,
                                      const JPH::SubShapeIDCreator&    inSubShapeIDCreator2,
                                      const JPH::CollideShapeSettings& inCollideShapeSettings,
                                      JPH::CollideShapeCollector&      ioCollector,
                                      const JPH::ShapeFilter&          inShapeFilter);

        static void collideSVOVsSVO(const JPH::Shape*                inShape1,
                                    const JPH::Shape*                inShape2,
                                    JPH::Vec3Arg                     inScale1,
                                    JPH::Vec3Arg                     inScale2,
                                    JPH::Mat44Arg                    inCenterOfMassTransform1,
                                    JPH::Mat44Arg                    inCenterOfMassTransform2,
                                    const JPH::SubShapeIDCreator&    inSubShapeIDCreator1,
                                    const JPH::SubShapeIDCreator&    inSubShapeIDCreator2,
                                    const JPH::CollideShapeSettings& inCollideShapeSettings,
                                    JPH::CollideShapeCollector&      ioCollector,
                                    const JPH::ShapeFilter&          inShapeFilter);

        static void castShapeVsSVO(const JPH::ShapeCast&         inShapeCast,
                                   const JPH::ShapeCastSettings& inShapeCastSettings,
                                   const JPH::Shape*             inShape,
                                   JPH::Vec3Arg                  inScale,
                                   const JPH::ShapeFilter&       inShapeFilter,
                                   JPH::Mat44Arg                 inCenterOfMassTransform2,
                                   const JPH::SubShapeIDCreator& inSubShapeIDCreator1,
                                   const JPH::SubShapeIDCreator& inSubShapeIDCreator2,
                                   JPH::CastShapeCollector&      ioCollector);

        static glm::dmat4 toGlmMat4(JPH::Mat44Arg mat);

        const SVO<bool>*  svo = nullptr;
        AABB3             localBounds;
        SVOMassProperties massProperties;
    };
}
