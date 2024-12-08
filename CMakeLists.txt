cmake_minimum_required(VERSION 3.14)
project(RobotArm)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_FIND_DEBUG_MODE ON)

# Add source and include directories
file(GLOB_RECURSE SOURCES src/*.cpp)
include_directories(include)

# Option to toggle between Emscripten and g++
option(USE_EMSCRIPTEN "Build with Emscripten" OFF)

if (USE_EMSCRIPTEN)
    # Emscripten setup
    set(CMAKE_CXX_COMPILER emcc)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s WASM=1 -s USE_SDL=2 -O3")
    set(CMAKE_EXECUTABLE_SUFFIX ".js")
    
    # Output target for Emscripten
    add_executable(robotarm ${SOURCES})
    set_target_properties(robotarm PROPERTIES 
        OUTPUT_NAME "robotarm"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
else()
    # Native build setup
    find_package(SDL2 REQUIRED)

    # Output target for native build
    add_executable(robotarm ${SOURCES})
    target_link_libraries(robotarm SDL2 SDL2main SDL2_ttf)
    set_target_properties(robotarm PROPERTIES 
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
    )
endif()
