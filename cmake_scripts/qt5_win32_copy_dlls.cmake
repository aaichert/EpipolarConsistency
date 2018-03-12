function(qt5_win32_copy_dlls APP)

	# if(CMAKE_BUILD_TYPE MATCHES RELEASE AND WIN32)


		# get_target_property(QT5_QMAKE_EXECUTABLE Qt5::qmake IMPORTED_LOCATION)
		# get_filename_component(QT5_WINDEPLOYQT_EXECUTABLE ${QT5_QMAKE_EXECUTABLE} PATH)
		# set(QT5_WINDEPLOYQT_EXECUTABLE "${QT5_WINDEPLOYQT_EXECUTABLE}/windeployqt.exe")

		# add_custom_command(TARGET ${APP} POST_BUILD
		# COMMAND ${QT5_WINDEPLOYQT_EXECUTABLE} --qmldir ${CMAKE_SOURCE_DIR} $<TARGET_FILE_DIR:${APP}> --no-translations --no-quick-import --no-webkit2 --no-angle)

	# endif()
	
	# foreach(DLL ${QT5_MODULES})
			# # find the release *.dll file
			# get_target_property(Qt5_${DLL}Location Qt5::${DLL} LOCATION)
			# # find the debug *d.dll file
			# get_target_property(Qt5_${DLL}LocationDebug Qt5::${DLL} IMPORTED_LOCATION_DEBUG)

			# add_custom_command(TARGET ${APP} POST_BUILD
			   # COMMAND ${CMAKE_COMMAND} -E copy_if_different $<$<CONFIG:Debug>:${Qt5_${DLL}LocationDebug}> $<$<NOT:$<CONFIG:Debug>>:${Qt5_${DLL}Location}> $<TARGET_FILE_DIR:${APP}>)
	# endforeach(DLL)

endfunction(qt5_win32_copy_dlls)
