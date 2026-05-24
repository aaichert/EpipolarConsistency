add_executable(Registration
	Registration.cpp
	Registration3D3D.hxx
)
target_link_libraries( Registration LibProjectiveGeometry LibEpipolarConsistency LibEpipolarConsistencyGui LibUtilsQt LibUtilsCuda GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARIES} Qt6::Widgets Qt6::Svg Qt6::OpenGL Qt6::PrintSupport)
install(TARGETS Registration DESTINATION bin)
set_property(TARGET Registration PROPERTY FOLDER "EpipolarConsistency") 


cuda_add_executable(RegistrationIntensityBased
	RegistrationIntensityBased.cpp
	warp3d.cu
)
target_link_libraries( RegistrationIntensityBased LibProjectiveGeometry LibRayCastBackproject LibUtilsQt LibUtilsCuda GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARIES} Qt6::Widgets Qt6::Svg Qt6::OpenGL Qt6::PrintSupport)
install(TARGETS RegistrationIntensityBased DESTINATION bin)
set_property(TARGET RegistrationIntensityBased PROPERTY FOLDER "Utilities") 

