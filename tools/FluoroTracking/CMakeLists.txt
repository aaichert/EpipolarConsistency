
#remove cuda_

add_executable(FluoroTracking
	FluoroTracking.cpp
)
target_link_libraries( FluoroTracking PRIVATE
	LibProjectiveGeometry
	LibEpipolarConsistency
	LibEpipolarConsistencyGui
	LibUtilsQt
	LibUtilsCuda
	GetSet
	GetSetGui
	OpenGL::GL
	${NLOPT_LIBRARIES}
	Qt6::Widgets
	Qt6::Svg
	Qt6::OpenGL
	Qt6::OpenGLWidgets
	Qt6::PrintSupport
)
install(TARGETS FluoroTracking DESTINATION bin)
qt_win32_copy_dlls(FluoroTracking)
set_property(TARGET FluoroTracking PROPERTY FOLDER "EpipolarConsistency") 
