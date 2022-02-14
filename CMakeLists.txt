cmake_minimum_required(VERSION 3.17)
project(camtest)

find_package(OpenMP REQUIRED)

set(CMAKE_C_FLAGS "\${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_C11_FLAGS "\${CMAKE_C11_FLAGS} ${OpenMP_C11_FLAGS}")

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Ofast)
add_executable(camtest main.cpp)

target_link_libraries(camtest -lOpenGL32 -lfreeGLUT)
