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
target_link_libraries( LibGeometryCalibration
	PUBLIC
		LibProjectiveGeometry
		LibRayCastBackproject
		LibUtilsQt
		GetSet
		GetSetGui
		Qt6::Gui
		Qt6::Svg
		Qt6::PrintSupport
)
install(TARGETS LibGeometryCalibration  DESTINATION lib)
set_property(TARGET LibGeometryCalibration PROPERTY FOLDER "Calibration")

add_executable( CalibPDS2
	main_pds2.cpp
)
target_link_libraries( CalibPDS2
	PRIVATE
		LibGeometryCalibration
		LibRayCastBackproject
		LibEpipolarConsistencyGui
		LibUtilsQt
		GetSet
		GetSetGui
		Qt6::Gui
		Qt6::Svg
		Qt6::PrintSupport
)
set_property(TARGET CalibPDS2 PROPERTY FOLDER "Calibration")
