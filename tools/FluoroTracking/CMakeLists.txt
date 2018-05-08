
add_executable(FluoroTracking
	FluoroTracking.cpp
)
target_link_libraries( FluoroTracking LibProjectiveGeometry LibEpipolarConsistency LibEpipolarConsistencyGui LibUtilsQt LibUtilsCuda GetSet GetSetGui ${OPENGL_LIBRARIES} ${NLOPT_LIBRARY})
qt5_use_modules(FluoroTracking Widgets Svg OpenGL PrintSupport)
install(TARGETS FluoroTracking DESTINATION bin)
qt5_win32_copy_dlls(FluoroTracking)
set_property(TARGET FluoroTracking PROPERTY FOLDER "Tools") 
