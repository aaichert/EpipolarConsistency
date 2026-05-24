if(MSVC)
set(ICON_RC icon_win.rc icon.ico)
else()
set(ICON_RC "")
endif()

# The actual TrajectoryView executable
add_executable( TrajectoryView
	main_trajectoryview.cpp
	${ICON_RC}
)
target_link_libraries( TrajectoryView LibProjectiveGeometry LibUtilsQt GetSet GetSetGui Qt6::Gui Qt6::Svg Qt6::OpenGL Qt6::PrintSupport)
install(TARGETS TrajectoryView RUNTIME DESTINATION bin)
set_property(TARGET TrajectoryView PROPERTY FOLDER "Utilities") 
