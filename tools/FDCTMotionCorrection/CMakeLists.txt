add_executable( FDCTMotionCorrection
	main_gradient_free.cpp
)
target_link_libraries( FDCTMotionCorrection PRIVATE
	LibProjectiveGeometry
	LibEpipolarConsistency
	LibUtilsCuda
	QCP
	GetSet
	GetSetGui
	${OPENGL_LIBRARIES}
	${NLOPT_LIBRARY}
	Qt6::Widgets
	Qt6::OpenGL
	Qt6::OpenGLWidgets
	Qt6::PrintSupport
)
install(TARGETS FDCTMotionCorrection DESTINATION bin)

