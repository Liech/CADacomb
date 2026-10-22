cmake_minimum_required(VERSION 3.20...3.30)

find_package(Catch2 REQUIRED)

project(CADacomb)
project(CADacombTest)

find_package(glm REQUIRED)
find_package(nlohmann_json REQUIRED)

set(Source_Files
    "Source/Library/testMain.cpp"
    "Source/Library/HelloWorld.cpp"
    "Source/Library/HelloWorld.h"
)
source_group("Source" FILES ${Source_Files})

set(ALL_FILES
    ${Source_Files}
)

############################################################################
############################################################################
############################################################################

add_executable(CADacombTest ${ALL_FILES})
add_library(CADacomb ${ALL_FILES})
set_property(TARGET CADacombTest PROPERTY CXX_STANDARD 20)
set_property(TARGET CADacomb PROPERTY CXX_STANDARD 20)

target_link_libraries(CADacombTest PUBLIC 
Catch2::Catch2
glm::glm
nlohmann_json::nlohmann_json
)
target_link_libraries(CADacomb PUBLIC 
glm::glm
nlohmann_json::nlohmann_json
)

target_include_directories(CADacomb PUBLIC 
./Source/
)
target_include_directories(CADacombTest PUBLIC 
./Source/
)

set_property(TARGET CADacombTest PROPERTY COMPILE_DEFINITIONS ISTESTPROJECT=1)

include(CTest)
include(Catch)
catch_discover_tests(CADacombTest)
