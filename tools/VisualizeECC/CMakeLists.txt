
cuda_add_executable(VisualizeECC
	draw_epipolar_lines.hxx
	VisualizeECC.cpp
)
target_link_libraries( VisualizeECC LibProjectiveGeometry LibEpipolarConsistency LibEpipolarConsistencyGui LibUtilsQt LibUtilsCuda GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARIES})
qt5_use_modules(VisualizeECC Widgets Svg OpenGL PrintSupport)
install(TARGETS VisualizeECC DESTINATION bin)
set_property(TARGET VisualizeECC PROPERTY FOLDER "EpipolarConsistency") 

cuda_add_executable(VisualizeECC_RadonIntermediate
	draw_epipolar_lines.hxx
	VisualizeECC_RadonIntermediate.cpp
)
target_link_libraries( VisualizeECC_RadonIntermediate LibProjectiveGeometry LibEpipolarConsistency LibEpipolarConsistencyGui LibUtilsQt LibUtilsCuda GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARIES})
qt5_use_modules(VisualizeECC_RadonIntermediate Widgets Svg OpenGL PrintSupport)
install(TARGETS VisualizeECC_RadonIntermediate DESTINATION bin)
set_property(TARGET VisualizeECC_RadonIntermediate PROPERTY FOLDER "EpipolarConsistency") 

cuda_add_executable(VisualizeECC_ComputedTomography
	draw_epipolar_lines.hxx
	VisualizeECC_ComputedTomography.cpp
)
target_link_libraries( VisualizeECC_ComputedTomography LibProjectiveGeometry LibEpipolarConsistency LibEpipolarConsistencyGui LibUtilsQt LibUtilsCuda GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARIES})
qt5_use_modules(VisualizeECC_ComputedTomography Widgets Svg OpenGL PrintSupport)
install(TARGETS VisualizeECC_ComputedTomography DESTINATION bin)
set_property(TARGET VisualizeECC_ComputedTomography PROPERTY FOLDER "EpipolarConsistency") 
