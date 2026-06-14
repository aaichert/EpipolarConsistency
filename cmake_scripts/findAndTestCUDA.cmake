## Modern CUDA support via CMake language integration
if(POLICY CMP0146)
    cmake_policy(SET CMP0146 NEW)
endif()

## Project() already declared CUDA as a language; CMake handles nvcc compilation.
## For libraries (e.g., cufft), find them explicitly if needed at link time.

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_EXTENSIONS OFF)
set(CMAKE_CUDA_SEPARABLE_COMPILATION OFF)
set(CMAKE_CUDA_ARCHITECTURES native)

set(CUDA_HAVE_GPU TRUE CACHE BOOL "Whether CUDA support is available" FORCE)

## Attempt to find CUDA libraries if CUDAToolkit is available; otherwise rely on system paths.
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
    message(STATUS "CUDAToolkit found: ${CUDAToolkit_VERSION}")
else()
    message(STATUS "CUDAToolkit not found; using system CUDA libraries if available")
endif()
