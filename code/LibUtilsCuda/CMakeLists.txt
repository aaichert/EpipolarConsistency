
add_library(LibUtilsCuda STATIC
	CudaTextureArray.hxx
	CudaBindlessTexture.h
	CudaBindlessTexture.cpp
	CudaBindlessTexture.cu
	CudaMemory.h
	CudaMemory.cpp
	CudaAlgorithms.h
	CudaAlgorithms.cpp
	CudaConvolution.cu
)
target_compile_definitions(LibUtilsCuda PRIVATE HAS_CUDA="Cuda Support")
target_link_libraries(LibUtilsCuda PUBLIC CUDA::cudart GetSet)
install(FILES UtilsCuda.hxx CudaTextureArray.hxx CudaMemory.h CudaAlgorithms.h CudaBindlessTexture.h DESTINATION include/LibUtilsCuda)

install(TARGETS LibUtilsCuda DESTINATION lib EXPORT LibUtilsCuda-targets)
install(EXPORT LibUtilsCuda-targets DESTINATION cmake)

# Header only linear algebra functions work on both CPU and GPU (i.e. always install)
install(FILES culaut/culaut.hxx culaut/xgeinv.hxx culaut/xprojectionmatrix.hxx DESTINATION include/LibUtilsCuda/culaut)
set_property(TARGET LibUtilsCuda PROPERTY FOLDER "Packages") 
