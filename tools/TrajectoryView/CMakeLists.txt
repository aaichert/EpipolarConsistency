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
target_link_libraries( TrajectoryView PRIVATE LibProjectiveGeometry LibUtilsQt GetSet GetSetGui Qt6::Gui Qt6::Svg Qt6::OpenGL Qt6::OpenGLWidgets Qt6::PrintSupport)
qt_win32_copy_dlls(TrajectoryView "Core;Gui;Widgets;Svg;OpenGL;PrintSupport")
install(TARGETS TrajectoryView RUNTIME DESTINATION bin)
set_property(TARGET TrajectoryView PROPERTY FOLDER "Utilities") 
