
cuda_add_executable(VisualizeECC
	VisualizeECC.cpp
)
target_link_libraries( VisualizeECC LibProjectiveGeometry LibEpipolarConsistency LibEpipolarConsistencyGui LibUtilsQt LibUtilsCuda GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARIES})
qt5_use_modules(VisualizeECC Widgets Svg OpenGL PrintSupport)
install(TARGETS VisualizeECC DESTINATION bin)
set_property(TARGET VisualizeECC PROPERTY FOLDER "EpipolarConsistency") 

cuda_add_executable(VisualizeECC_RadonIntermediate
	VisualizeECC_RadonIntermediate.cpp
)
target_link_libraries( VisualizeECC_RadonIntermediate LibProjectiveGeometry LibEpipolarConsistency LibEpipolarConsistencyGui LibUtilsQt LibUtilsCuda GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARIES})
qt5_use_modules(VisualizeECC_RadonIntermediate Widgets Svg OpenGL PrintSupport)
install(TARGETS VisualizeECC_RadonIntermediate DESTINATION bin)
set_property(TARGET VisualizeECC_RadonIntermediate PROPERTY FOLDER "EpipolarConsistency") 
