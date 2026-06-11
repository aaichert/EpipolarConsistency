
add_executable(VisualizeECC
	draw_epipolar_lines.hxx
	VisualizeECC.cpp
)
target_link_libraries( VisualizeECC PRIVATE
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
install(TARGETS VisualizeECC DESTINATION bin)
set_property(TARGET VisualizeECC PROPERTY FOLDER "EpipolarConsistency") 

add_executable(VisualizeECC_RadonIntermediate
	draw_epipolar_lines.hxx
	VisualizeECC_RadonIntermediate.cpp
)
target_link_libraries( VisualizeECC_RadonIntermediate PRIVATE
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
install(TARGETS VisualizeECC_RadonIntermediate DESTINATION bin)
set_property(TARGET VisualizeECC_RadonIntermediate PROPERTY FOLDER "EpipolarConsistency") 

add_executable(VisualizeECC_ComputedTomography
	draw_epipolar_lines.hxx
	VisualizeECC_ComputedTomography.cpp
)
target_link_libraries( VisualizeECC_ComputedTomography PRIVATE
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
install(TARGETS VisualizeECC_ComputedTomography DESTINATION bin)
set_property(TARGET VisualizeECC_ComputedTomography PROPERTY FOLDER "EpipolarConsistency") 
