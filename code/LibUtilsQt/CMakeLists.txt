
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
qt5_use_modules(LibUtilsQt OpenGL PrintSupport)
target_link_libraries(LibUtilsQt LibProjectiveGeometry GetSet GetSetGui ${OPENGL_LIBRARIES})
install(TARGETS LibUtilsQt DESTINATION lib EXPORT LibUtilsQt-targets)
install(EXPORT LibUtilsQt-targets DESTINATION cmake)
set_property(TARGET LibUtilsQt PROPERTY FOLDER "Packages") 