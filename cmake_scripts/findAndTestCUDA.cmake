# Add CUDA
find_package(CUDA)
if(CUDA_FOUND)
    try_run(RUN_RESULT_VAR COMPILE_RESULT_VAR
        ${CMAKE_BINARY_DIR} 
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake_scripts/has_cuda_gpu.cpp
        CMAKE_FLAGS 
            -DINCLUDE_DIRECTORIES:STRING=${CUDA_TOOLKIT_INCLUDE}
            -DLINK_LIBRARIES:STRING=${CUDA_CUDART_LIBRARY}
        COMPILE_OUTPUT_VARIABLE COMPILE_OUTPUT_VAR
        RUN_OUTPUT_VARIABLE RUN_OUTPUT_VAR)
    message("${RUN_OUTPUT_VAR}") # Display number of GPUs found
    # COMPILE_RESULT_VAR is TRUE when compile succeeds
    # RUN_RESULT_VAR is zero when a GPU is found
	if(NOT COMPILE_RESULT_VAR)
		 message("Compilation of a simple CUDA program failed. Please check if CUDA is installed correctly.")
	endif()
    if(COMPILE_RESULT_VAR AND NOT RUN_RESULT_VAR)
        set(CUDA_HAVE_GPU TRUE CACHE BOOL "Whether CUDA-capable GPU is present")
    else()
        set(CUDA_HAVE_GPU FALSE CACHE BOOL "Whether CUDA-capable GPU is present")
    endif()
	# nvcc flags
	set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};--gpu-architecture sm_50)
	option(CUDA_NVCC_DEBUGGING "Build CUDA code with NVCC debug info (-g;-G)" OFF)
	if(CUDA_NVCC_DEBUGGING)
		set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-g;-G)
	endif()
	option(CUDA_NVCC_VERBOSE "Verbose output of NVCC" OFF)
	if(CUDA_NVCC_VERBOSE)
		set(CUDA_NVCC_FLAGS --ptxas-options=-v;-lineinfo)
	endif(CUDA_NVCC_VERBOSE)
endif(CUDA_FOUND)
