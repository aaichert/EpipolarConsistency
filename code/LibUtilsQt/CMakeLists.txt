
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
target_link_libraries(LibUtilsQt
	PUBLIC
		LibProjectiveGeometry
		GetSet
		GetSetGui
		Qt6::Core
		Qt6::Gui
		Qt6::Widgets
		Qt6::PrintSupport
		Qt6::OpenGL
		Qt6::OpenGLWidgets
		OpenGL::GL
)
install(TARGETS LibUtilsQt DESTINATION lib EXPORT LibUtilsQt-targets)
install(EXPORT LibUtilsQt-targets DESTINATION cmake)
set_property(TARGET LibUtilsQt PROPERTY FOLDER "Packages") 