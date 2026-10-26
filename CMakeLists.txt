cmake_minimum_required(VERSION 3.20)

project(DroneSimulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML CONFIG REQUIRED COMPONENTS Graphics Window System)

add_executable(DroneSimulator
main.cpp
)

target_include_directories(DroneSimulator PRIVATE .)

if(WIN32)
    target_compile_definitions(DroneSimulator PRIVATE
        WIN32_LEAN_AND_MEAN   # prevents windows.h from pulling in old winsock.h
        NOMINMAX              # prevents windows.h from macro-defining min/max
    )
endif()

target_link_libraries(DroneSimulator PRIVATE
SFML::Graphics
SFML::Window
SFML::System
)

if(WIN32)
    target_link_libraries(DroneSimulator PRIVATE opengl32 glu32 ws2_32)
else()
    find_package(OpenGL REQUIRED)
    target_link_libraries(DroneSimulator PRIVATE OpenGL::GL OpenGL::GLU)
endif()
