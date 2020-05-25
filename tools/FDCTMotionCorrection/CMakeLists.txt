add_executable( FDCTMotionCorrection
	main_gradient_free.cpp
)
target_link_libraries( FDCTMotionCorrection LibProjectiveGeometry LibEpipolarConsistency LibCudaUtils QCP GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARY})
qt5_use_modules(FDCTMotionCorrection Widgets OpenGL PrintSupport)
install(TARGETS FDCTMotionCorrection DESTINATION bin)

