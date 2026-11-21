# CADacomb Physics Engine Wrapper Architecture

## Overview

The `CADacomb::Physics` module provides a backend-agnostic physics simulation abstraction modeled after the `Vulcant` (`Interface` / `VulcantV` / `VulcantG`) and `CADacomb::CAD` (`Interface` / `Sm`) architectures. Higher-level libraries and applications interact exclusively with pure abstract interfaces in `CADacomb/Physics/Interface/`, allowing different physics backends (such as Jolt Physics or MuJoCo) to be swapped or added without changing application or database code.

## Folder Structure

- `CADacomb/Physics/Interface/`
  - Contains **only** abstract base classes defining the physics contract:
    - `Physics`: Abstract instancing hub and simulation world interface. Responsible for creating rigid bodies from `CADacomb::Triangulation` geometry, configuring world parameters (such as gravity), and stepping the simulation forward in time.
    - `RigidBody`: Abstract handle representing a physical body in the simulation world. Exposes whether the body is static or dynamic and provides access to its 4x4 world transformation matrix (`glm::dmat4`).
- `CADacomb/Physics/Jolt/`
  - Contains the Jolt Physics backend implementation:
    - `PhysicsJ`: Concrete subclass of `Physics` (`class PhysicsJ : public Physics`). Serves as the sole instantiation entry point for the Jolt backend and encapsulates `JPH::PhysicsSystem`, `JPH::TempAllocatorImpl`, `JPH::JobSystemThreadPool`, and collision layer filters.
    - `RigidBodyJ`: Concrete subclass of `RigidBody` (`class RigidBodyJ : public RigidBody`). Wraps a `JPH::BodyID` managed by `PhysicsJ`, translating between `CADacomb` double-precision `glm::dmat4` transformations (preserving non-uniform scale) and Jolt rigid body poses.
    - `BroadPhaseLayerInterfaceJ`, `ObjectVsBroadPhaseLayerFilterJ`, `ObjectLayerPairFilterJ`: Jolt broad-phase and object-layer collision filtering adapters separating static (`nonMoving`) and dynamic (`moving`) bodies.

## Usage Pattern

Consumers instantiate `PhysicsJ` once and immediately upcast or hold it through the abstract `Physics` interface:

```cpp
std::unique_ptr<CADacomb::Physics::Physics> physics = std::make_unique<CADacomb::Physics::PhysicsJ>();
auto body = physics->createRigidBody(triangulation, initialTransform, isStatic);
physics->step(1.0 / 60.0);
glm::dmat4 updatedTransform = body->getTransformation();
```
