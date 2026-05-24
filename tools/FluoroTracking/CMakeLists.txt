
#remove cuda_

cuda_add_executable(FluoroTracking
	FluoroTracking.cpp
)
target_link_libraries( FluoroTracking LibProjectiveGeometry LibEpipolarConsistency LibEpipolarConsistencyGui LibUtilsQt LibUtilsCuda GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARIES} Qt6::Widgets Qt6::Svg Qt6::OpenGL Qt6::PrintSupport)
install(TARGETS FluoroTracking DESTINATION bin)
set_property(TARGET FluoroTracking PROPERTY FOLDER "EpipolarConsistency") 
