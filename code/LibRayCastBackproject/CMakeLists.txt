
cuda_add_library(LibRayCastBackproject
	VolumeRendering.h
	VolumeRendering.cpp
	VoxelData.h
	VoxelData.cpp
	VoxelData.cu
	Reconstruction.h
	Reconstruction.cpp
	Reconstruction.cu
	ray_box_intersection.hxx
	vr_ray_geom_setup.cu
	vr_raycast_drr.cu
	vr_raycast_ea_1Dtf.cu
	vr_raycast_iso.cu
	vr_raycast_mip.cu
	vr_raycast_debug.cu
	# 2do only if GetSet is available.
	VolumeRenderingGui.h
	VolumeRenderingGui.cpp
)

# DEBUG ONLY LibUtilsQt
target_link_libraries(LibRayCastBackproject LibProjectiveGeometry LibUtilsCuda LibUtilsQt GetSet Qt6::Core Qt6::Gui Qt6::Widgets)
set_property(TARGET LibRayCastBackproject PROPERTY FOLDER "Packages") 

install(FILES VolumeRendering.h DESTINATION include/LibRayCastBackproject)
install(TARGETS LibRayCastBackproject DESTINATION lib EXPORT LibRayCastBackproject-targets)
install(EXPORT LibRayCastBackproject-targets DESTINATION cmake)

