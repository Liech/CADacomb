cmake_minimum_required(VERSION 3.20...3.30)

project(CADacomb)

find_package(glm REQUIRED)
find_package(nlohmann_json REQUIRED)

set(Binary_Files
    "Source/Util/Binary/base64.h"
    "Source/Util/Binary/base64.cpp"
    "Source/Util/Binary/BufferRange.h"
    "Source/Util/Binary/BufferRange.cpp"
    "Source/Util/Binary/BufferSpan.h"
    "Source/Util/Binary/BufferSpan.cpp"
    "Source/Util/Binary/DefragmentationBuffer.h"
    "Source/Util/Binary/DefragmentationBuffer.cpp"
)
source_group("Source/Util/Binary" FILES ${Binary_Files})

set(Triangle_Files
    "Source/Triangle/BaseShapeGenerator.h"
    "Source/Triangle/BaseShapeGenerator.cpp"
    "Source/Triangle/Triangulation.h"
    "Source/Triangle/Triangulation.cpp"
    "Source/Triangle/sreiter_stl_reader_LICENSE"
)
source_group("Source/Triangle" FILES ${Triangle_Files})

set(Triangle_HalfEdge_Files
    "Source/Triangle/HalfEdge/HalfEdge.h"
    "Source/Triangle/HalfEdge/HalfEdge.cpp"
    "Source/Triangle/HalfEdge/HalfEdgeHealth.h"
    "Source/Triangle/HalfEdge/HalfEdgeHealth.cpp"
    "Source/Triangle/HalfEdge/mesh2halfedge.h"
    "Source/Triangle/HalfEdge/mesh2halfedge.cpp"
)
source_group("Source/Triangle/HalfEdge" FILES ${Triangle_HalfEdge_Files})

set(Triangle_Util_Files
    "Source/Triangle/Util/STLWriter.h"
    "Source/Triangle/Util/STLWriter.cpp"
    "Source/Triangle/Util/stl_reader.h"
)
source_group("Source/Triangle/Util" FILES ${Triangle_Util_Files})

set(ALL_FILES
    ${Binary_Files}
    ${Triangle_Files}
    ${Triangle_HalfEdge_Files}
    ${Triangle_Util_Files}
)

add_library(CADacomb ${ALL_FILES})
add_library(CADacomb::CADacomb ALIAS CADacomb)
set_property(TARGET CADacomb PROPERTY CXX_STANDARD 20)

target_link_libraries(CADacomb PUBLIC 
    glm::glm
    nlohmann_json::nlohmann_json
)

target_include_directories(CADacomb PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source/Triangle>
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
        "Source/testMain.cpp"
        ${ALL_FILES}
    )
    add_executable(CADacombTest ${Test_Source_Files})
    set_property(TARGET CADacombTest PROPERTY CXX_STANDARD 20)
    target_link_libraries(CADacombTest PUBLIC 
        Catch2::Catch2
        glm::glm
        nlohmann_json::nlohmann_json
    )
    target_include_directories(CADacombTest PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}/Source
        ${CMAKE_CURRENT_SOURCE_DIR}/Source/Triangle
    )
    set_property(TARGET CADacombTest PROPERTY COMPILE_DEFINITIONS ISTESTPROJECT=1)

    include(CTest)
    include(Catch)
    catch_discover_tests(CADacombTest)
endif()
