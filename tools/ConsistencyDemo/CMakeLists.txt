
add_executable( ConsistencyDemo
	main.cpp
)
target_link_libraries( ConsistencyDemo PRIVATE
	LibProjectiveGeometry
	LibEpipolarConsistency
	LibEpipolarConsistencyGui
	LibUtilsQt
	GetSet
	GetSetGui
	Qt6::Gui
	Qt6::Svg
	Qt6::OpenGL
	Qt6::OpenGLWidgets
	Qt6::PrintSupport
)
install(TARGETS ConsistencyDemo RUNTIME DESTINATION bin)
set_property(TARGET ConsistencyDemo PROPERTY FOLDER "Utilities") 
