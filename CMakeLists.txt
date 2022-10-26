cmake_minimum_required(VERSION 3.22)
project(myPhysicsEngine)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_COMPILER clang++)

set(CMAKE_CXX_FLAGS " -L./Debug")

include_directories(src/ include/)
link_directories(/usr/lib/x86_64-linux-gnu/)

add_library(lib STATIC src/app.cpp src/timing.cpp )
add_library(target STATIC src/demos/ballistic.cpp)

find_package(OpenGL REQUIRED COMPONENTS OpenGL)
include_directories(${OpenGL_INCLUDE_DIR})

add_executable(demo src/main.cpp)
target_link_libraries(demo  target lib OpenGL::GL OpenGL::GLU libglut.so) 