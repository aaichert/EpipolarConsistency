add_executable(ComputeRadonIntermediate
	ComputeRadonIntermediate.cpp
)
target_link_libraries( ComputeRadonIntermediate PRIVATE
	LibProjectiveGeometry
	LibEpipolarConsistency
	LibEpipolarConsistencyGui
	LibUtilsQt
	LibUtilsCuda
	GetSet
	GetSetGui
	Qt6::Widgets
	Qt6::Svg
	Qt6::OpenGL
	Qt6::OpenGLWidgets
	Qt6::PrintSupport
)
install(TARGETS ComputeRadonIntermediate DESTINATION bin)
qt_win32_copy_dlls(ComputeRadonIntermediate)
set_property(TARGET ComputeRadonIntermediate PROPERTY FOLDER "Utilities") 
