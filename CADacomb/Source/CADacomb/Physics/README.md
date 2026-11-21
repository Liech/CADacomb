# CADacomb Physics Engine Wrapper Architecture

## Overview

The `CADacomb::Physics` module provides a backend-agnostic physics simulation abstraction modeled after the `Vulcant` (`Interface` / `VulcantV` / `VulcantG`) and `CADacomb::CAD` (`Interface` / `Sm`) architectures. Higher-level libraries and applications interact exclusively with pure abstract interfaces in `CADacomb/Physics/Interface/`, allowing different physics backends (such as Jolt Physics or MuJoCo) to be swapped or added without changing application or database code.

## Folder Structure

- `CADacomb/Physics/Interface/`
  - Contains **only** abstract base classes defining the physics contract:
    - `Physics`: Abstract instancing hub and simulation world interface. Responsible for creating rigid bodies from `CADacomb::Triangulation` geometry or directly from `CADacomb::SVO<bool>` octrees, creating deformable soft bodies from `CADacomb::Triangulation` meshes, configuring world parameters (such as gravity), and stepping the simulation forward in time.
    - `RigidBody`: Abstract handle representing a physical rigid body in the simulation world. Exposes whether the body is static or dynamic, provides access to its 4x4 world transformation matrix (`glm::dmat4`), and supports `notifyGeometryChanged()` for live geometry mutations (such as dynamically modified SVOs).
    - `SoftBody`: Abstract handle representing a deformable soft body in the simulation world. Provides access to its 4x4 world transformation matrix (`glm::dmat4`) and deformed local vertex positions (`getDeformedVertices()`).
- `CADacomb/Volume/SVO/` (Engine-Agnostic SVO Physics Queries)
  - `SVOColliderQuery`: Engine-agnostic octree pruning and traversal (`queryOverlappingLeaves` for AABB-pruned solid leaf discovery in $O(\log N)$, and `queryOverlappingLeafPairs` for dual-SVO collision traversal).
  - `SVOMassProperties`: Analytical mass, center of mass, and $3\times 3$ inertia tensor calculation over solid octree leaves using the parallel-axis theorem.
- `CADacomb/Physics/Jolt/`
  - Contains the Jolt Physics backend implementation:
    - `PhysicsJ`: Concrete subclass of `Physics` (`class PhysicsJ : public Physics`). Serves as the sole instantiation entry point for the Jolt backend and encapsulates `JPH::PhysicsSystem`, `JPH::TempAllocatorImpl`, `JPH::JobSystemSingleThreaded`, and collision layer filters.
    - `RigidBodyJ`: Concrete subclass of `RigidBody` (`class RigidBodyJ : public RigidBody`). Wraps a `JPH::BodyID` managed by `PhysicsJ`, translating between `CADacomb` double-precision `glm::dmat4` transformations (preserving non-uniform scale) and Jolt rigid body poses.
    - `SoftBodyJ`: Concrete subclass of `SoftBody` (`class SoftBodyJ : public SoftBody`). Welds duplicate mesh vertices into a connected XPBD particle/edge/bend/volume-pressure soft body (`JPH::SoftBodySharedSettings` / `JPH::SoftBodyCreationSettings`) and maps deformed particle coordinates back to the asset's original vertex array.
    - `SVOShapeJ`: Custom `JPH::Shape` (`EShapeType::User1`) that queries a live `SVO<bool>` directly via `SVOColliderQuery` and `SVOMassProperties` without triangulating or duplicating voxel data, and dispatches overlapping solid octree leaf boxes to `JPH::CollisionDispatch` and `CollideSoftBodyVertices`.
    - `BroadPhaseLayerInterfaceJ`, `ObjectVsBroadPhaseLayerFilterJ`, `ObjectLayerPairFilterJ`: Jolt broad-phase and object-layer collision filtering adapters separating static (`nonMoving`) and dynamic (`moving`) bodies.

## Usage Pattern

Consumers instantiate `PhysicsJ` once and immediately upcast or hold it through the abstract `Physics` interface:

```cpp
std::unique_ptr<CADacomb::Physics::Physics> physics = std::make_unique<CADacomb::Physics::PhysicsJ>();
auto meshBody = physics->createRigidBody(triangulation, initialTransform, isStatic);
auto svoBody  = physics->createRigidBody(svo, localBounds, svoTransform, isStatic);
auto softBody = physics->createSoftBody(softTriangulation, softTransform, 1.0, 2000.0, 0.0001, 0.001, 0.2, 0.3, 0.0, 10);
physics->step(1.0 / 60.0);
glm::dmat4              updatedTransform = softBody->getTransformation();
std::vector<glm::dvec3> deformedVertices = softBody->getDeformedVertices();
```
