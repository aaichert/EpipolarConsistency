add_executable(Registration
	Registration.cpp
	Registration3D3D.hxx
)
target_link_libraries( Registration LibProjectiveGeometry LibEpipolarConsistency LibEpipolarConsistencyGui LibUtilsQt LibUtilsCuda GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARIES})
qt5_use_modules(Registration Widgets Svg OpenGL PrintSupport)
install(TARGETS Registration DESTINATION bin)
set_property(TARGET Registration PROPERTY FOLDER "EpipolarConsistency") 


cuda_add_executable(RegistrationIntensityBased
	RegistrationIntensityBased.cpp
	warp3d.cu
)
target_link_libraries( RegistrationIntensityBased LibProjectiveGeometry LibRayCastBackproject LibUtilsQt LibUtilsCuda GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARIES})
qt5_use_modules(RegistrationIntensityBased Widgets Svg OpenGL PrintSupport)
install(TARGETS RegistrationIntensityBased DESTINATION bin)
set_property(TARGET RegistrationIntensityBased PROPERTY FOLDER "Utilities") 

