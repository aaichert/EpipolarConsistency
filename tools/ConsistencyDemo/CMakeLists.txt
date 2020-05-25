
add_executable( ConsistencyDemo
	main.cpp
)
target_link_libraries( ConsistencyDemo LibProjectiveGeometry LibEpipolarConsistency LibEpipolarConsistencyGui LibUtilsQt GetSet GetSetGui)
qt5_use_modules(ConsistencyDemo Gui Svg OpenGL PrintSupport)
install(TARGETS ConsistencyDemo RUNTIME DESTINATION bin)
set_property(TARGET ConsistencyDemo PROPERTY FOLDER "Utilities") 
