cmake_minimum_required(VERSION 2.8...3.20)

find_package(Catch2 REQUIRED)

project(Library)
project(Test)

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

add_executable(Test ${ALL_FILES})
add_library(Library ${ALL_FILES})
set_property(TARGET Test PROPERTY CXX_STANDARD 20)
set_property(TARGET Library PROPERTY CXX_STANDARD 20)

target_link_libraries(Test PUBLIC 
Catch2::Catch2
glm::glm
nlohmann_json::nlohmann_json
)
target_link_libraries(Library PUBLIC 
glm::glm
nlohmann_json::nlohmann_json
)

target_include_directories(Library PUBLIC 
./Source/
)
target_include_directories(Test PUBLIC 
./Source/
)

set_property( TARGET Test PROPERTY COMPILE_DEFINITIONS ISTESTPROJECT=1 )


include(CTest)
include(Catch)
catch_discover_tests(Test)