
add_executable( FDCTCalibrationCorrection
	FDCTCalibrationCorrection.cpp
)
target_link_libraries( FDCTCalibrationCorrection LibProjectiveGeometry LibEpipolarConsistency LibCudaUtils QCP GetSet GetSetGui LibPlotView ${NLOPT_LIBRARY} Qt6::Widgets Qt6::OpenGL Qt6::PrintSupport)
install(TARGETS FDCTCalibrationCorrection DESTINATION bin)
