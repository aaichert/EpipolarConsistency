if(MSVC)
set(ICON_RC icon_win.rc icon.ico)
else()
set(ICON_RC "")
endif()

add_executable( nrrdView
	nrrdView.cpp
	${ICON_RC}
)
target_link_libraries( nrrdView LibProjectiveGeometry LibUtilsQt GetSet GetSetGui )
install(TARGETS nrrdView RUNTIME DESTINATION bin)
set_property(TARGET nrrdView PROPERTY FOLDER "Utilities") 
