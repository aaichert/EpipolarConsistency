
cuda_add_executable( VolumeRendering
	volume_renderer.cpp
)
target_link_libraries( VolumeRendering LibProjectiveGeometry LibRayCastBackproject LibUtilsCuda LibUtilsQt GetSet GetSetGui)
qt5_use_modules(VolumeRendering Gui )
install(TARGETS VolumeRendering RUNTIME DESTINATION bin)
set_property(TARGET VolumeRendering PROPERTY FOLDER "Utilities")

# cuda_add_executable( Reconstruction
	# reconstruction.cpp
# )
# target_link_libraries( Reconstruction LibProjectiveGeometry LibRayCastBackproject LibEpipolarConsistencyGui LibUtilsCuda LibUtilsQt GetSet GetSetGui)
# qt5_use_modules(Reconstruction Gui )
# install(TARGETS Reconstruction RUNTIME DESTINATION bin)
# set_property(TARGET Reconstruction PROPERTY FOLDER "Utilities")
