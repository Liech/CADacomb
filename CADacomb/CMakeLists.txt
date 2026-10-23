cmake_minimum_required(VERSION 3.20...3.30)

project(CADacomb)

find_package(glm REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenMP)

set(Binary_Files
    "Source/CADacomb/Util/Binary/base64.h"
    "Source/CADacomb/Util/Binary/base64.cpp"
    "Source/CADacomb/Util/Binary/BufferRange.h"
    "Source/CADacomb/Util/Binary/BufferRange.cpp"
    "Source/CADacomb/Util/Binary/BufferSpan.h"
    "Source/CADacomb/Util/Binary/BufferSpan.cpp"
    "Source/CADacomb/Util/Binary/DefragmentationBuffer.h"
    "Source/CADacomb/Util/Binary/DefragmentationBuffer.cpp"
)
source_group("Source/Util/Binary" FILES ${Binary_Files})

set(Triangle_Files
    "Source/CADacomb/Triangle/BaseShapeGenerator.h"
    "Source/CADacomb/Triangle/BaseShapeGenerator.cpp"
    "Source/CADacomb/Triangle/Triangulation.h"
    "Source/CADacomb/Triangle/Triangulation.cpp"
    "Source/CADacomb/Triangle/sreiter_stl_reader_LICENSE"
)
source_group("Source/Triangle" FILES ${Triangle_Files})

set(Triangle_HalfEdge_Files
    "Source/CADacomb/Triangle/HalfEdge/HalfEdge.h"
    "Source/CADacomb/Triangle/HalfEdge/HalfEdge.cpp"
    "Source/CADacomb/Triangle/HalfEdge/HalfEdgeHealth.h"
    "Source/CADacomb/Triangle/HalfEdge/HalfEdgeHealth.cpp"
    "Source/CADacomb/Triangle/HalfEdge/mesh2halfedge.h"
    "Source/CADacomb/Triangle/HalfEdge/mesh2halfedge.cpp"
)
source_group("Source/Triangle/HalfEdge" FILES ${Triangle_HalfEdge_Files})

set(Triangle_Util_Files
    "Source/CADacomb/Triangle/Util/STLWriter.h"
    "Source/CADacomb/Triangle/Util/STLWriter.cpp"
    "Source/CADacomb/Triangle/Util/stl_reader.h"
)
source_group("Source/Triangle/Util" FILES ${Triangle_Util_Files})

set(Volume_Files
    "Source/CADacomb/Volume/AABB3.h"
    "Source/CADacomb/Volume/AABB3.cpp"
)
source_group("Source/Volume" FILES ${Volume_Files})

set(Volume_BrickSVO_Files
    "Source/CADacomb/Volume/BrickSVO/IBrickSVO.h"
    "Source/CADacomb/Volume/BrickSVO/IBrickSVO.cpp"
    "Source/CADacomb/Volume/BrickSVO/BrickSVONode.h"
    "Source/CADacomb/Volume/BrickSVO/BrickSVONode.cpp"
    "Source/CADacomb/Volume/BrickSVO/Brick.h"
    "Source/CADacomb/Volume/BrickSVO/Brick.cpp"
    "Source/CADacomb/Volume/BrickSVO/BrickSVO.h"
    "Source/CADacomb/Volume/BrickSVO/BrickSVO.cpp"
)
source_group("Source/Volume/BrickSVO" FILES ${Volume_BrickSVO_Files})

set(Volume_Flat_Files
    "Source/CADacomb/Volume/Flat/BinaryVolume.h"
    "Source/CADacomb/Volume/Flat/BinaryVolume.cpp"
    "Source/CADacomb/Volume/Flat/IVec3Compare.h"
    "Source/CADacomb/Volume/Flat/IVec3Compare.cpp"
    "Source/CADacomb/Volume/Flat/BlockyVoxelTriangulation.h"
    "Source/CADacomb/Volume/Flat/BlockyVoxelTriangulation.cpp"
    "Source/CADacomb/Volume/Flat/MarchingCubes.h"
    "Source/CADacomb/Volume/Flat/MarchingCubes.cpp"
    "Source/CADacomb/Volume/Flat/SurfaceNets.h"
    "Source/CADacomb/Volume/Flat/SurfaceNets.cpp"
    "Source/CADacomb/Volume/Flat/Voxelizer.h"
    "Source/CADacomb/Volume/Flat/Voxelizer.cpp"
)
source_group("Source/Volume/Flat" FILES ${Volume_Flat_Files})

set(Volume_MergeStrategy_Files
    "Source/CADacomb/Volume/MergeStrategy/MergeStrategy.h"
    "Source/CADacomb/Volume/MergeStrategy/MergeStrategy.cpp"
    "Source/CADacomb/Volume/MergeStrategy/MeanMergeStrategy.h"
    "Source/CADacomb/Volume/MergeStrategy/MeanMergeStrategy.cpp"
    "Source/CADacomb/Volume/MergeStrategy/MostPrevalentMergeStrategy.h"
    "Source/CADacomb/Volume/MergeStrategy/MostPrevalentMergeStrategy.cpp"
    "Source/CADacomb/Volume/MergeStrategy/MostPrevalentExceptMergeStrategy.h"
    "Source/CADacomb/Volume/MergeStrategy/MostPrevalentExceptMergeStrategy.cpp"
)
source_group("Source/Volume/MergeStrategy" FILES ${Volume_MergeStrategy_Files})

set(Volume_Progress_Files
    "Source/CADacomb/Volume/Progress/IProgressBar.h"
    "Source/CADacomb/Volume/Progress/IProgressBar.cpp"
)
source_group("Source/Volume/Progress" FILES ${Volume_Progress_Files})

set(Volume_SVO_Files
    "Source/CADacomb/Volume/SVO/AnySVO.h"
    "Source/CADacomb/Volume/SVO/AnySVO.cpp"
    "Source/CADacomb/Volume/SVO/SVONode.h"
    "Source/CADacomb/Volume/SVO/SVOResult.h"
    "Source/CADacomb/Volume/SVO/SVOResult.cpp"
    "Source/CADacomb/Volume/SVO/SVOAxis.h"
    "Source/CADacomb/Volume/SVO/SVODualNode.h"
    "Source/CADacomb/Volume/SVO/SVODualTraversal.h"
    "Source/CADacomb/Volume/SVO/SVODualTraversal.cpp"
    "Source/CADacomb/Volume/SVO/BinaryVolumeToSVO.h"
    "Source/CADacomb/Volume/SVO/BinaryVolumeToSVO.cpp"
    "Source/CADacomb/Volume/SVO/SVO.h"
    "Source/CADacomb/Volume/SVO/SVO.cpp"
    "Source/CADacomb/Volume/SVO/SVOVoxelizer.h"
    "Source/CADacomb/Volume/SVO/SVOVoxelizer.cpp"
)
source_group("Source/Volume/SVO" FILES ${Volume_SVO_Files})

set(Volume_SVO_Blocky_Files
    "Source/CADacomb/Volume/SVO/Blocky/SVOBlockyOptions.h"
    "Source/CADacomb/Volume/SVO/Blocky/SVOBlockyOptions.cpp"
    "Source/CADacomb/Volume/SVO/Blocky/SVOBlockyTriangulation.h"
    "Source/CADacomb/Volume/SVO/Blocky/SVOBlockyTriangulation.cpp"
)
source_group("Source/Volume/SVO/Blocky" FILES ${Volume_SVO_Blocky_Files})

set(Volume_SVO_Generator_Files
    "Source/CADacomb/Volume/SVO/Generator/LandscapeConfig.h"
    "Source/CADacomb/Volume/SVO/Generator/LandscapeConfig.cpp"
    "Source/CADacomb/Volume/SVO/Generator/SVOLandscapeGenerator.h"
    "Source/CADacomb/Volume/SVO/Generator/SVOLandscapeGenerator.cpp"
)
source_group("Source/Volume/SVO/Generator" FILES ${Volume_SVO_Generator_Files})

set(Volume_SVOMesher_Files
    "Source/CADacomb/Volume/SVOMesher/README.md"
    "Source/CADacomb/Volume/SVOMesher/PackedQuad.h"
    "Source/CADacomb/Volume/SVOMesher/PackedQuad.cpp"
    "Source/CADacomb/Volume/SVOMesher/MeshChunk.h"
    "Source/CADacomb/Volume/SVOMesher/MeshChunk.cpp"
    "Source/CADacomb/Volume/SVOMesher/SVOMesherOptions.h"
    "Source/CADacomb/Volume/SVOMesher/SVOMesherOptions.cpp"
    "Source/CADacomb/Volume/SVOMesher/SVOCPUMesher.h"
    "Source/CADacomb/Volume/SVOMesher/SVOCPUMesher.cpp"
)
source_group("Source/Volume/SVOMesher" FILES ${Volume_SVOMesher_Files})

set(Volume_LODStrategy_Files
    "Source/CADacomb/Volume/LODStrategy/CapacityTargetLODStrategy.h"
    "Source/CADacomb/Volume/LODStrategy/CapacityTargetLODStrategy.cpp"
    "Source/CADacomb/Volume/LODStrategy/DistanceBandLODStrategy.h"
    "Source/CADacomb/Volume/LODStrategy/DistanceBandLODStrategy.cpp"
    "Source/CADacomb/Volume/LODStrategy/FixedLODStrategy.h"
    "Source/CADacomb/Volume/LODStrategy/FixedLODStrategy.cpp"
    "Source/CADacomb/Volume/LODStrategy/FrustumAdaptiveLODStrategy.h"
    "Source/CADacomb/Volume/LODStrategy/FrustumAdaptiveLODStrategy.cpp"
    "Source/CADacomb/Volume/LODStrategy/LODDecision.h"
    "Source/CADacomb/Volume/LODStrategy/LODDecision.cpp"
    "Source/CADacomb/Volume/LODStrategy/LODStrategy.h"
    "Source/CADacomb/Volume/LODStrategy/LODStrategy.cpp"
    "Source/CADacomb/Volume/LODStrategy/SceneData.h"
    "Source/CADacomb/Volume/LODStrategy/SceneData.cpp"
    "Source/CADacomb/Volume/LODStrategy/ScreenSpaceLODStrategy.h"
    "Source/CADacomb/Volume/LODStrategy/ScreenSpaceLODStrategy.cpp"
)
source_group("Source/Volume/LODStrategy" FILES ${Volume_LODStrategy_Files})

set(ALL_FILES
    ${Binary_Files}
    ${Triangle_Files}
    ${Triangle_HalfEdge_Files}
    ${Triangle_Util_Files}
    ${Volume_Files}
    ${Volume_BrickSVO_Files}
    ${Volume_Flat_Files}
    ${Volume_MergeStrategy_Files}
    ${Volume_Progress_Files}
    ${Volume_SVO_Files}
    ${Volume_SVO_Blocky_Files}
    ${Volume_SVO_Generator_Files}
    ${Volume_SVOMesher_Files}
    ${Volume_LODStrategy_Files}
)

add_library(CADacomb ${ALL_FILES})
add_library(CADacomb::CADacomb ALIAS CADacomb)
set_property(TARGET CADacomb PROPERTY CXX_STANDARD 20)

target_link_libraries(CADacomb PUBLIC 
    glm::glm
    nlohmann_json::nlohmann_json
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(CADacomb PUBLIC OpenMP::OpenMP_CXX)
endif()

target_include_directories(CADacomb PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS CADacomb
    EXPORT CADacombTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY Source/ DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

if(CADACOMB_BUILD_TESTS)
    find_package(Catch2 REQUIRED)

    set(Test_Source_Files
        "Source/CADacomb/testMain.cpp"
        ${ALL_FILES}
    )
    add_executable(CADacombTest ${Test_Source_Files})
    set_property(TARGET CADacombTest PROPERTY CXX_STANDARD 20)
    target_link_libraries(CADacombTest PUBLIC 
        Catch2::Catch2
        glm::glm
        nlohmann_json::nlohmann_json
    )
    if(OpenMP_CXX_FOUND)
        target_link_libraries(CADacombTest PUBLIC OpenMP::OpenMP_CXX)
    endif()
    target_include_directories(CADacombTest PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/Source
    )
    set_property(TARGET CADacombTest PROPERTY COMPILE_DEFINITIONS ISTESTPROJECT=1)

    include(CTest)
    include(Catch)
    catch_discover_tests(CADacombTest)
endif()
