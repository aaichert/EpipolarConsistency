add_library( LibGeometryCalibration
	BeadDetection.h
	BeadDetection.cpp
	GeometryCalibration.h
	GeometryCalibration.cpp
	EstimateProjectionMatrix.h
	EstimateProjectionMatrix.cpp
	PhantomPDS2.h
	PhantomPDS2.cpp
	BeadPhantomGui.hxx
)
target_link_libraries( LibGeometryCalibration LibProjectiveGeometry LibRayCastBackproject LibUtilsQt GetSet GetSetGui)
qt5_use_modules(LibGeometryCalibration Gui Svg PrintSupport)
install(TARGETS LibGeometryCalibration  DESTINATION lib)
set_property(TARGET LibGeometryCalibration PROPERTY FOLDER "Calibration")

add_executable( CalibPDS2
	main_pds2.cpp
)
target_link_libraries( CalibPDS2 LibGeometryCalibration LibRayCastBackproject LibEpipolarConsistencyGui LibRayCastBackproject LibUtilsQt GetSet GetSetGui)
qt5_use_modules(CalibPDS2 Gui Svg PrintSupport)
set_property(TARGET CalibPDS2 PROPERTY FOLDER "Calibration")
