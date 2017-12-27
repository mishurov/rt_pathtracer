cmake_minimum_required(VERSION 3.4.1)
project(rt_pathtracer)

# in case a gcc version is too new for CUDA (e.g. Debian 9, CUDA 8, gcc 5)
# cmake -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ ..

find_package(CUDA REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(JPEG REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)


include_directories(
    ${CUDA_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${JPEG_INCLUDE_DIR}
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/rib_lexer_parser
    ${CMAKE_CURRENT_BINARY_DIR}/rib_lexer_parser
)

set(NVIDIA_SOURCES
    nvidia_base/Array.cpp
    nvidia_base/Buffer.cpp
    nvidia_base/Defs.cpp
    nvidia_base/Math.cpp
    nvidia_base/Scene.cpp
    nvidia_base/Sort.cpp
    nvidia_bvh/Platform.cpp
    nvidia_bvh/BVH.cpp
    nvidia_bvh/BVHNode.cpp
    nvidia_bvh/CudaBVH.cpp
    nvidia_bvh/SplitBVHBuilder.cpp
    nvidia_bvh/Util.cpp
)

set(SOURCES
    main.cc
    renderer.cc
    geometry/rib_primitives.cc
    geometry/rib_loader.cc
    geometry/quadrics_tesselator.cc
    materials/texture_loader.cc
    cameras/maya_camera.cc
    cameras/cardboard_camera.cc
    network/server.cc
    network/PracticalSocket.cpp
)

set(CUDA_SOURCES
    integrator/integrator.cu
    geometry/quadrics.cu
    geometry/quadrics_gpl.cu
    nvidia_kernels/intersect_kepler.cu
)

add_subdirectory(rib_lexer_parser)

set(CUDA_SEPARABLE_COMPILATION ON)
set(CUDA_NVCC_FLAGS "-arch=sm_50")

cuda_add_library(integrators STATIC ${CUDA_SOURCES})
add_executable(rt_pathtracer ${NVIDIA_SOURCES} ${SOURCES})

set_target_properties(rt_pathtracer PROPERTIES COMPILE_FLAGS
                      "${CMAKE_CXX_FLAGS} -std=c++11")


target_link_libraries(rt_pathtracer
    rib_driver
    integrators
    ${CUDA_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${GLFW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${JPEG_LIBRARIES}
    -lcuda
)

