
add_library( LibUtilsQt
	Figure.hxx
	Plot.hxx
	EventFilters.h
	EventFilters.cpp
	FigureWindow.h
	FigureWindow.cpp
	QCameraView.h
	QCameraView.cpp
	QGLPlot3D.h
	QGLPlot3D.cpp
# QCP should actually be a library built someplace else ... (http://www.qcustomplot.com/)
	QCustomPlot.h
	QCustomPlot.cpp
)
target_link_libraries(LibUtilsQt Qt6::OpenGL Qt6::PrintSupport LibProjectiveGeometry GetSet GetSetGui ${OPENGL_LIBRARIES})
install(TARGETS LibUtilsQt DESTINATION lib EXPORT LibUtilsQt-targets)
install(EXPORT LibUtilsQt-targets DESTINATION cmake)
set_property(TARGET LibUtilsQt PROPERTY FOLDER "Packages") 