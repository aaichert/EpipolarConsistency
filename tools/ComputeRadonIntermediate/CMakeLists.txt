add_executable(ComputeRadonIntermediate
	ComputeRadonIntermediate.cpp
)
target_link_libraries( ComputeRadonIntermediate LibProjectiveGeometry LibEpipolarConsistency LibEpipolarConsistencyGui LibUtilsQt LibUtilsCuda GetSet GetSetGui Qt6::Widgets Qt6::Svg Qt6::OpenGL Qt6::PrintSupport)
install(TARGETS ComputeRadonIntermediate DESTINATION bin)
set_property(TARGET ComputeRadonIntermediate PROPERTY FOLDER "Utilities") 
