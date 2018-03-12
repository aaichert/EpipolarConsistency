
cuda_add_executable(VisualizeECC
	VisualizeECC.cpp
)
target_link_libraries( VisualizeECC LibProjectiveGeometry LibEpipolarConsistency LibEpipolarConsistencyGui LibUtilsQt LibUtilsCuda GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARY})
qt5_use_modules(VisualizeECC Widgets Svg OpenGL PrintSupport)
install(TARGETS VisualizeECC DESTINATION bin)
set_property(TARGET VisualizeECC PROPERTY FOLDER "EpipolarConsistency") 

