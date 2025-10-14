cmake_minimum_required(VERSION 3.15)
project(SimuStage4A)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_SOURCE_DIR}/Libraries/glad/include
    ${CMAKE_SOURCE_DIR}/Libraries/glfw/include
)

file(GLOB SOURCES
    ${CMAKE_SOURCE_DIR}/*.cpp
    ${CMAKE_SOURCE_DIR}/Libraries/glad/src/glad.c
)

add_executable(SimuStage4A ${SOURCES})

target_link_libraries(SimuStage4A
    ${CMAKE_SOURCE_DIR}/Libraries/glfw/lib/libglfw3.a
    opengl32 gdi32 user32 kernel32
)
