
# A library to evaluate a metric of epipolar consistency
cuda_add_library(LibEpipolarConsistency
	# Common methods for ECC direct and with Radon intermediate
	EpipolarConsistency.h
	EpipolarConsistency.cpp
	# Some epipolar geometry functions which are needed in any case.
	EpipolarConsistencyCommon.hxx
	# Extract redundant signals directly from images
	# This version is useful when ECC metric is computed only once
	# or when the images are subject to change during optimization.
	EpipolarConsistencyDirect.h
	EpipolarConsistencyDirect.cpp
	EpipolarConsistencyDirect.cu
	# Compute pairwise fan-beam cc via rectification.
	RectifiedFBCC.h
	# Actual rectification (used only for visualization)
	Rectification.cu
	# Extract redundant samples from Radon intermediate functions of images.
	# This Version requires the Radon transform to be pre-computed. It is able to
	# quickly evaluate the ECC metric if only projection matrices are changed.
	EpipolarConsistencyRadonIntermediate.h
	EpipolarConsistencyRadonIntermediate.cpp
	EpipolarConsistencyRadonIntermediate.cu
	# The Radon intermediate functions is the expensive pre-computation.
	RadonIntermediate.h
	RadonIntermediate.cpp
	RadonIntermediate.cu
	OPTIONS -DHAS_CUDA="Cuda Support"
)
target_link_libraries(LibEpipolarConsistency LibProjectiveGeometry LibUtilsCuda ${CUDA_CUFFT_LIBRARIES} )
install(FILES EpipolarConsistencyCommon.hxx EpipolarConsistency.h EpipolarConsistencyDirect.h RectifiedFBCC.h RadonIntermediate.h EpipolarConsistencyRadonIntermediate.h DESTINATION include/LibEpipolarConsistency)
install(TARGETS LibEpipolarConsistency DESTINATION lib EXPORT LibEpipolarConsistency-targets)
install(EXPORT LibEpipolarConsistency-targets DESTINATION cmake)
set_property(TARGET LibEpipolarConsistency PROPERTY FOLDER "EpipolarConsistency") 

# Some useful code for loading and pre-processing data when GetSetGui is available.
add_library(LibEpipolarConsistencyGui
	# Pre-Processing and Radon intermediate function
	Gui/PreProccess.h
	Gui/PreProccess.cpp
	Gui/ComputeRadonIntermediate.hxx
	# Input Data
	Gui/InputDataRadonIntermediate.h
	Gui/InputDataRadonIntermediate.cpp
	Gui/InputDataDirect.h
	Gui/InputDataDirect.cpp
	# Visualization
	Gui/DisplayGeometry.hxx
	Gui/Visualization.h
	Gui/Visualization.cpp
	# Input Data
	Gui/SingleImageMotion.h
	# Registration
	Gui/Registration.h
	Gui/Registration.cpp
	# ECC optimization over one input image given several reference images
	# (e.g. one by one optimimization for FDCT or tracking of Fluoro sequence)
	#Gui/SingleImageMotion.hxx
	# (Inefficiant but simple) implementation of ECC motion correction in FDCT
	Gui/FDCTMotionCorrection.hxx
)
target_link_libraries(LibEpipolarConsistencyGui LibEpipolarConsistency LibProjectiveGeometry LibUtilsQt GetSet GetSetGui)
install(FILES Gui/PreProccess.h Gui/ComputeRadonIntermediate.hxx Gui/Visualization.h Gui/DisplayGeometry.hxx Gui/SingleImageMotion.hxx Gui/FDCTMotionCorrection.hxx DESTINATION include/LibEpipolarConsistency/Gui)
install(TARGETS LibEpipolarConsistencyGui DESTINATION lib EXPORT LibEpipolarConsistencyGui-targets)
install(EXPORT LibEpipolarConsistencyGui-targets DESTINATION cmake)
set_property(TARGET LibEpipolarConsistencyGui PROPERTY FOLDER "EpipolarConsistency") 
