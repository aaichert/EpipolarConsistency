function(qt_win32_copy_dlls APP)
	# This function is deprecated and no longer needed with modern Qt6 on Windows.
	# Qt6 uses WinDeployQt which handles DLL deployment automatically.
	# For manual deployment, use windeployqt.exe from your Qt6 installation:
	# ${Qt6_DIR}/../../../bin/windeployqt.exe <app_directory>
	
	# Legacy implementation kept for reference (commented out):
	# if(CMAKE_BUILD_TYPE MATCHES RELEASE AND WIN32)
	#	get_target_property(QT_QMAKE_EXECUTABLE Qt6::qmake IMPORTED_LOCATION)
	#	get_filename_component(QT_WINDEPLOYQT_EXECUTABLE ${QT_QMAKE_EXECUTABLE} PATH)
	#	set(QT_WINDEPLOYQT_EXECUTABLE "${QT_WINDEPLOYQT_EXECUTABLE}/windeployqt.exe")
	#	add_custom_command(TARGET ${APP} POST_BUILD
	#		COMMAND ${QT_WINDEPLOYQT_EXECUTABLE} $<TARGET_FILE_DIR:${APP}> --no-translations --no-quick-import)
	# endif()
endfunction(qt_win32_copy_dlls)
