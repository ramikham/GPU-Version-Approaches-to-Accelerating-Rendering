cmake_minimum_required(VERSION 3.26)
project(untitled CUDA)

set(CMAKE_CUDA_STANDARD 11)

add_executable(untitled main.cu
        Vec3D.h
        Sphere.h
        Ray.h
        Material.h
        Primitives_List.h
        Primitive.h
        camera.h
        XY_Rectangle.h
        XZ_Rectangle.h
        YZ_Rectangle.h
        Box.h
        Triangle.h
        BVH.h
        AABB.h
        PDF.h
        Rotate_Y.h
)

set_target_properties(untitled PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)

find_package(CUDA  REQUIRED)
include_directories("${CUDA_INCLUDE_DIRS}")

# Add the --extended-lambda flag to CUDA_NVCC_FLAGS
list(APPEND CUDA_NVCC_FLAGS "--extended-lambda" "--use_fast_math" "-O3" "-arch=sm_61" "--extra-device-vectorization")