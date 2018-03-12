
if (CUDA_HAVE_GPU)
	# cuda_add_executable(TestUtilsCuda
	# 	TestUtilsCuda.cpp
	# 	TestUtilsCuda.cu
	# )
	# target_link_libraries(TestUtilsCuda LibUtilsCuda LibProjectiveGeometry)

	cuda_add_library(LibUtilsCuda STATIC
		CudaTextureArray.hxx
		CudaBindlessTexture.h
		CudaBindlessTexture.cpp
		CudaBindlessTexture.cu
		CudaMemory.h
		CudaMemory.cpp
		CudaAlgorithms.h
		CudaAlgorithms.cpp
		CudaConvolution.cu
		OPTIONS -DHAS_CUDA="Cuda Support"
	)
	install(FILES UtilsCuda.hxx CudaTextureArray.hxx CudaMemory.h CudaAlgorithms.h CudaBindlessTexture.h DESTINATION include/LibUtilsCuda)

	install(TARGETS LibUtilsCuda DESTINATION lib EXPORT LibUtilsCuda-targets)
	install(EXPORT LibUtilsCuda-targets DESTINATION cmake)
endif(CUDA_HAVE_GPU)

# Header only linear algebra functions work on both CPU and GPU (i.e. always install)
install(FILES culaut/culaut.hxx culaut/xgeinv.hxx culaut/xprojectionmatrix.hxx DESTINATION include/LibUtilsCuda/culaut)
set_property(TARGET LibUtilsCuda PROPERTY FOLDER "Packages") 
