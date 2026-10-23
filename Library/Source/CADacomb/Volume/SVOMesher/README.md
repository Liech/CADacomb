# SVOMesher: Architecture Plan

## 1. System Overview

SVOMesher is an architecture for meshing and dynamically updating large-scale Sparse Voxel Octree (SVO) volumes.

The SVO represents spatial geometry in pure discrete integer coordinates where the finest leaf node represents 1 grid unit (e.g. 1 millimeter or 1 centimeter in physical world space, depending on the volume's world-scale transform).

The system decouples the continuous spatial octree from the rendered polygonal geometry by partitioning the visible world into bounded rendering units called MeshChunks. Each MeshChunk corresponds to a fixed-resolution sub-volume of the octree evaluated down to a specific Level of Detail (LOD) depth.

### Bounded Voxel Mass Invariant
To ensure deterministic execution times and predictable memory consumption, every MeshChunk evaluates at most a constant voxel resolution (e.g. up to 128³ cells) regardless of its physical footprint:
- Fine LOD (Near observer): A chunk covers a compact spatial volume (e.g. 128 discrete units, which may represent 128 mm at millimeter scale) evaluated down to leaf voxels (1 unit scale).
- Coarse LOD (Far observer): A chunk covers an exponentially larger spatial volume (e.g. 1024 or 8192 units), but traversal stops at a coarser octree depth, yielding the exact same upper bound of at most 128³ cells.

Because the maximum number of cell evaluations per chunk is strictly bounded:
- Chunk generation has a deterministic maximum execution time.
- GPU vertex buffer allocations can be sized uniformly.
- Incremental updates are time-boxed by scheduling a fixed number of chunk rebuilds per frame.

---

## 2. Integration with Core SVO Systems

### Volume Construction and Hierarchy (`BinaryVolumeToSVO.h`)
SVO data generation and hierarchy construction rely on `Volume/SVO/BinaryVolumeToSVO.h`:
- **SVO Generation**: Volumes are ingested into power-of-two octree dimensions with integer coordinate offsets.
- **Hierarchical LOD Merging**: During SVO construction, `generateHierarchy()` propagates node values from finest leaves up to the root using pluggable merge strategies:
  - `MostPrevalentMergeStrategy`: Picks the statistically dominant child material.
  - `MostPrevalentExceptMergeStrategy`: Merges child values while ignoring empty space, ensuring fine solid features and thin surfaces are preserved when viewed from coarse LOD distances.
- **LOD Value Ingestion**: When a MeshChunk stops traversal early at its cutoff depth, the internal node's merged value directly provides the voxel material for that LOD cell without inspecting descendant nodes.

### Dual Traversal Foundation (`SVODualTraversal.h`)
The neighbor-pairing and boundary-detection logic is based on `Volume/SVO/SVODualTraversal.h`:
- **Lockstep Recursive Pairing**: `SVODualTraversal` traverses adjacent sibling and cousin nodes across all three axes (X, Y, Z) in $O(N)$ time, discovering every touching leaf-pair interface without hash maps or spatial search structures.
- **Asymmetric Depth Handling**: In `traverseFace`, when a leaf node touches an internal node with children, the leaf is held constant while recursing down the 4 touching child quadrants of the neighbor. This handles scale transitions naturally.
- **Exterior Boundaries**: `traverseBoundaryFace` projects outer faces against virtual exterior boundary nodes, ensuring chunks correctly detect outer surfaces at chunk or world limits.
- **Role in SVOMesher**: `SVODualTraversal` serves as the core CPU meshing engine and validation reference, establishing the exact pairing rules that are mirrored in the GPU compute shader.

---

## 3. Meshing Method

### Traversal and Node Selection
Meshing operates over an SVO sub-tree bounded by the spatial extent and depth cutoff of the target MeshChunk:
1. **Root Binding**: The chunk references the octree node representing its spatial bounding box, or traverses from the global root restricted to the chunk's integer bounding box.
2. **Depth Cutoff**: Traversal descends the tree until reaching either:
   - A leaf node (a node with no children).
   - An internal node whose depth equals the chunk's target LOD cutoff depth. In this case, the node's merged value (computed during SVO hierarchy generation via merge strategies) is treated as the effective voxel value.
3. **Active Node Filtering**: Empty (air) nodes are skipped. Solid or material-bearing nodes proceed to neighbor evaluation.

### Neighbor Evaluation: The 6-Side Material Difference Rule
For every active solid node of scale S, all six primary directional faces (-X, +X, -Y, +Y, -Z, +Z) are tested against adjacent volume space:
1. **Neighbor Identification**:
   - The adjacent region across the face boundary is queried within the octree.
   - Due to the adaptive octree structure, the neighbor across a face of size S may be:
     - Equal size (S): A single adjacent node at the same tree depth.
     - Coarser (> S): An ancestor node spanning a larger volume than the current node.
     - Finer (< S): Multiple smaller child nodes sharing the face area.
2. **Material Difference Test**:
   - If the neighbor region across the face consists entirely of identical material, the face is internal and culled (no geometry emitted).
   - If any portion of the neighbor region differs in material (including transitions to empty/air), a quad is emitted covering that face.
3. **Face Emission**:
   - Each emitted quad is defined by its corner origin, its two tangent basis vectors scaled by S, and its normal vector corresponding to the face direction.
   - Winding order is counter-clockwise relative to the outward face normal to ensure standard backface culling.

### Watertightness Across LOD Boundaries
When adjacent MeshChunks operate at differing LOD levels (e.g. LOD 0 next to LOD 1):
- The fine chunk (scale S) and the coarse chunk (scale 2S) independently evaluate the boundary plane against the master SVO.
- Both chunks evaluate the material state at the interface. If an interface exists between solid and air, the fine chunk emits four coplanar quads of scale S, while the coarse chunk emits one coplanar quad of scale 2S facing in the opposite direction.
- Visually, the boundary is watertight because the geometry forms a closed surface with zero gaps or light leaks, avoiding cracks without requiring crack-stitching skirts or T-junction triangulation.

---

## 4. Implementation in Slang

The compute meshing pipeline is implemented using Slang targeting Vulkan SPIR-V.

### Compute Pipeline Structure
The meshing pipeline runs as an asynchronous compute pass:
1. **Data Buffers**:
   - **SVO Storage Buffer**: Read-only structured buffer containing the linear SVO nodes (child pointers and packed material values).
   - **Chunk Parameter Uniforms**: Push constants or parameter blocks providing the chunk origin, scale, LOD cutoff depth, and maximum quad allocation.
   - **Quad Pool Storage Buffer**: Read-write structured buffer storing packed quad descriptions.
   - **Indirect Draw Buffer**: Read-write storage buffer matching the Vulkan indirect draw command layout (vertex count, instance count, first vertex, first instance).

### Thread Mapping and Traversal Model
Two execution models are viable in Slang compute shaders:
1. **Thread-per-Leaf Model**:
   - The active surface leaves of the chunk are collected or indexed.
   - Each compute thread inspects one active node and its six directional faces.
   - For each face, the thread traverses the SVO buffer using bitwise coordinate descent:
     - At each tree level, the target octant index is extracted from the coordinate bits using bitwise shift and mask operations.
     - Traversal follows child pointers down to the cutoff depth or leaf level.
     - Because the tree depth is shallow (typically 7 to 10 levels within a chunk), this descent is a fixed, non-recursive loop with no dynamic call stack.
     - Spatially coherent threads in the same warp/wavefront query overlapping upper-level octree nodes, resulting in high GPU cache hit rates.
2. **Hierarchical Workgroup Model**:
   - Workgroups cooperatively descend the octree levels.
   - Shared memory (`groupshared`) holds frontier node lists, and child tasks are expanded cooperatively, mirroring `SVODualTraversal` face tests in local workgroup memory.

### Quad Compaction and Subgroup Intrinsics
To avoid atomic contention when appending quads into the global output buffer:
1. Threads determine which of their 6 faces require quad emission.
2. Rather than each thread calling global atomics individually, the compute shader uses Slang subgroup intrinsics (wave-level operations):
   - `WaveActiveCountBits` computes the total number of quads produced by all active lanes in the warp.
   - A single lane executes one atomic addition on the global counter to reserve a contiguous block of quad slots for the entire wave.
   - `WavePrefixCountBits` computes the local slot offset within that reserved block for each individual lane.
3. Threads write their packed quads directly to their reserved contiguous memory locations, maintaining fully coalesced memory writes.
4. The final workgroup updates the indirect draw command's vertex count by multiplying the total quad count by 6.

### Packed Quad Representation
Quads are stored in an 8-byte (64-bit) packed format:
- 30 bits: Local coordinate offsets (10 bits each for X, Y, Z within the chunk grid).
- 3 bits: Face direction index (0 to 5 indicating -X, +X, -Y, +Y, -Z, +Z).
- 4 bits: Scale exponent (power-of-two size multiplier).
- 27 bits: Material identifier and lighting/shading flags.

### Vertex Shader Decoding
The rasterization pipeline renders the mesh using indirect draws without an index buffer:
1. The draw call invokes 6 vertices per quad.
2. In the Slang vertex shader:
   - The quad index is computed by dividing the system vertex ID by 6.
   - The vertex corner index within the quad is computed using the vertex ID modulo 6.
   - The shader fetches the 64-bit packed quad from the storage buffer.
   - Bitfield extraction retrieves the coordinate origin, scale, normal direction, and material.
   - A fixed lookup table converts the corner index (0 to 5) into local unit quad corner offsets, generating the two triangles of the quad.
   - The vertex position is transformed into world space and projected to clip space.

---

## 5. Work Orchestration and Time-Boxing

### Chunk Lifecycle and Dirty Queue
MeshChunks transition through states:
- **Clean**: Mesh geometry is valid and actively rendered.
- **Dirty**: The underlying SVO voxels changed, or the camera distance triggered an LOD transition.
- **Meshing**: Enqueued for compute shader execution.

### Frame-Budgeted Updates
1. The system maintains a priority queue of dirty chunks sorted by distance to the camera (closest and finest LOD chunks prioritized).
2. Each frame, the engine pops at most N chunks according to the frame time budget.
3. The selected chunks are recorded into a Vulkan command buffer and dispatched on the compute queue.
4. While a chunk is regenerating, its existing mesh buffer remains visible (double buffering), preventing visual popping or geometry holes. Once the compute pass finishes, the active indirect draw pointer is updated.
