cmake_minimum_required(VERSION 3.20...3.30)

project(CADacombRender)

find_package(glm REQUIRED)
if(NOT TARGET Vulcant::Vulcant)
    find_package(Vulcant CONFIG REQUIRED)
endif()

find_program(SLANG_COMPILER NAMES slangc slangc.exe PATH_SUFFIXES tools/shader-slang REQUIRED)

set(SHADER_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_shaders")
file(MAKE_DIRECTORY "${SHADER_GEN_DIR}")
set(GENERATED_SHADER_HEADERS "")

set(SVORaycast_Core_Files
    "Source/CADacombRender/SVORaycast/Core/Ray.slang"
    "Source/CADacombRender/SVORaycast/Core/RayHit.slang"
)
source_group("SVORaycast/Core" FILES ${SVORaycast_Core_Files})

set(SVORaycast_Storage_Files
    "Source/CADacombRender/SVORaycast/Storage/GPUSVONode.h"
    "Source/CADacombRender/SVORaycast/Storage/GPUSVONode.cpp"
    "Source/CADacombRender/SVORaycast/Storage/GPUSVONode.slang"
    "Source/CADacombRender/SVORaycast/Storage/ISVOStorage.slang"
    "Source/CADacombRender/SVORaycast/Storage/FlatSVOStorage.slang"
)
source_group("SVORaycast/Storage" FILES ${SVORaycast_Storage_Files})

set(SVORaycast_Proxy_Files
    "Source/CADacombRender/SVORaycast/Proxy/ProxyRaySetup.slang"
    "Source/CADacombRender/SVORaycast/Proxy/IRayProxy.slang"
    "Source/CADacombRender/SVORaycast/Proxy/AABBProxy.slang"
)
source_group("SVORaycast/Proxy" FILES ${SVORaycast_Proxy_Files})

set(SVORaycast_LOD_Files
    "Source/CADacombRender/SVORaycast/LOD/ILODPolicy.slang"
    "Source/CADacombRender/SVORaycast/LOD/PixelFootprintLODPolicy.slang"
)
source_group("SVORaycast/LOD" FILES ${SVORaycast_LOD_Files})

set(SVORaycast_Shading_Files
    "Source/CADacombRender/SVORaycast/Shading/SVOShadeContext.slang"
    "Source/CADacombRender/SVORaycast/Shading/ISVOShading.slang"
    "Source/CADacombRender/SVORaycast/Shading/DirectionalSVOShading.slang"
)
source_group("SVORaycast/Shading" FILES ${SVORaycast_Shading_Files})

set(SVORaycast_Traversal_Files
    "Source/CADacombRender/SVORaycast/Traversal/SVODDATraversal.slang"
)
source_group("SVORaycast/Traversal" FILES ${SVORaycast_Traversal_Files})

set(SVORaycast_Pipeline_Files
    "Source/CADacombRender/SVORaycast/Pipeline/SVORaycastParams.h"
    "Source/CADacombRender/SVORaycast/Pipeline/SVORaycastParams.cpp"
    "Source/CADacombRender/SVORaycast/Pipeline/SVORaycastParams.slang"
    "Source/CADacombRender/SVORaycast/Pipeline/SceneUniform.slang"
    "Source/CADacombRender/SVORaycast/Pipeline/ProxyVertexInput.slang"
    "Source/CADacombRender/SVORaycast/Pipeline/ProxyVertexOutput.slang"
    "Source/CADacombRender/SVORaycast/Pipeline/RaycastFragmentOutput.slang"
    "Source/CADacombRender/SVORaycast/Pipeline/SVORaycastPipeline.slang"
)
source_group("SVORaycast/Pipeline" FILES ${SVORaycast_Pipeline_Files})

set(SVORaycast_Renderer_Files
    "Source/CADacombRender/SVORaycast/Renderer/SVORaycastRenderer.h"
    "Source/CADacombRender/SVORaycast/Renderer/SVORaycastRenderer.cpp"
    "Source/CADacombRender/SVORaycast/Renderer/SVORaycast_vert.exe.slang"
    "Source/CADacombRender/SVORaycast/Renderer/SVORaycast_Default_frag.exe.slang"
)
source_group("SVORaycast/Renderer" FILES ${SVORaycast_Renderer_Files})

set(SVORaycast_All_Files
    ${SVORaycast_Core_Files}
    ${SVORaycast_Storage_Files}
    ${SVORaycast_Proxy_Files}
    ${SVORaycast_LOD_Files}
    ${SVORaycast_Shading_Files}
    ${SVORaycast_Traversal_Files}
    ${SVORaycast_Pipeline_Files}
    ${SVORaycast_Renderer_Files}
)

set(SVORaycast_Shader_Modules "")
foreach(CANDIDATE_FILE ${SVORaycast_All_Files})
    if(CANDIDATE_FILE MATCHES "\\.slang$" AND NOT CANDIDATE_FILE MATCHES "\\.exe\\.slang$")
        list(APPEND SVORaycast_Shader_Modules "${CMAKE_CURRENT_SOURCE_DIR}/${CANDIDATE_FILE}")
    endif()
endforeach()

foreach(SHADER_FILE ${SVORaycast_All_Files})
    if(SHADER_FILE MATCHES "\\.exe\\.slang$")
        set(ABS_SHADER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${SHADER_FILE}")
        file(RELATIVE_PATH REL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Source/CADacombRender" "${ABS_SHADER_PATH}")

        get_filename_component(REL_DIR "${REL_PATH}" DIRECTORY)
        string(REPLACE ".exe.slang" "" BASE_NAME "${REL_PATH}")
        get_filename_component(FILE_NAME_WE "${BASE_NAME}" NAME)

        set(SUB_DIR "${SHADER_GEN_DIR}/${REL_DIR}")
        set(OUTPUT_HEADER "${SUB_DIR}/${FILE_NAME_WE}.h")

        file(MAKE_DIRECTORY "${SUB_DIR}")

        set(STAGE_FLAG "")
        if(SHADER_FILE MATCHES "_vert\\.exe\\.slang$")
            set(STAGE_FLAG "-stage" "vertex")
        elseif(SHADER_FILE MATCHES "_frag\\.exe\\.slang$")
            set(STAGE_FLAG "-stage" "fragment")
        elseif(SHADER_FILE MATCHES "_comp\\.exe\\.slang$")
            set(STAGE_FLAG "-stage" "compute")
        endif()

        add_custom_command(
            OUTPUT "${OUTPUT_HEADER}"
            COMMAND "${SLANG_COMPILER}" "${ABS_SHADER_PATH}"
                    -target spirv
                    -entry main
                    ${STAGE_FLAG}
                    -profile spirv_1_3
                    -source-embed-style u32
                    -source-embed-language cpp
                    -source-embed-name "${FILE_NAME_WE}_spirv"
                    -o "${OUTPUT_HEADER}"
                    -I "${CMAKE_CURRENT_SOURCE_DIR}/Source/CADacombRender"
            DEPENDS "${ABS_SHADER_PATH}" ${SVORaycast_Shader_Modules}
            COMMENT "Compiling CADacombRender Slang Entry Point: ${REL_PATH}"
            VERBATIM
        )

        list(APPEND GENERATED_SHADER_HEADERS "${OUTPUT_HEADER}")
    endif()
endforeach()

set(Source_Files
    "Source/CADacombRender/RenderHelper.cpp"
    "Source/CADacombRender/RenderHelper.h"
    "Source/CADacombRender/VulcantFragmentationBuffer.cpp"
    "Source/CADacombRender/VulcantFragmentationBuffer.h"
    "Source/CADacombRender/SceneDataConverter.cpp"
    "Source/CADacombRender/SceneDataConverter.h"
    "Source/CADacombRender/SVOLODChunk.cpp"
    "Source/CADacombRender/SVOLODChunk.h"
    "Source/CADacombRender/SVOLODMesh.cpp"
    "Source/CADacombRender/SVOLODMesh.h"
)
source_group("CADacombRender" FILES ${Source_Files})

set(SVOLOD_Files
    "Source/CADacombRender/SVOLOD/SVOMeshJob.h"
    "Source/CADacombRender/SVOLOD/SVOMeshJob.cpp"
    "Source/CADacombRender/SVOLOD/SVOCompletedMesh.h"
    "Source/CADacombRender/SVOLOD/SVOCompletedMesh.cpp"
)
source_group("SVOLOD" FILES ${SVOLOD_Files})

set(ALL_FILES
    ${Source_Files}
    ${SVOLOD_Files}
    ${SVORaycast_All_Files}
    ${GENERATED_SHADER_HEADERS}
)

add_library(CADacombRender ${ALL_FILES})
add_library(CADacomb::CADacombRender ALIAS CADacombRender)
set_property(TARGET CADacombRender PROPERTY CXX_STANDARD 20)

target_link_libraries(CADacombRender PUBLIC
    CADacomb
    Vulcant::Vulcant
    VulcantToolkit
    glm::glm
)

target_include_directories(CADacombRender PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source/CADacombRender>
    $<BUILD_INTERFACE:${SHADER_GEN_DIR}>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS CADacombRender
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
        "Source/CADacombRender/testMain.cpp"
        ${Source_Files}
        ${SVOLOD_Files}
        ${SVORaycast_All_Files}
        ${GENERATED_SHADER_HEADERS}
    )
    add_executable(CADacombRenderTest ${Test_Source_Files})
    set_property(TARGET CADacombRenderTest PROPERTY CXX_STANDARD 20)
    target_link_libraries(CADacombRenderTest PUBLIC
        CADacomb
        Vulcant::Vulcant
        VulcantToolkit
        Catch2::Catch2
        glm::glm
    )
    target_include_directories(CADacombRenderTest PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Source
        ${CMAKE_CURRENT_SOURCE_DIR}/Source/CADacombRender
        ${SHADER_GEN_DIR}
    )
    set_property(TARGET CADacombRenderTest PROPERTY COMPILE_DEFINITIONS ISTESTPROJECT=1)

    include(CTest)
    include(Catch)
    catch_discover_tests(CADacombRenderTest DISCOVERY_MODE PRE_TEST)
endif()
