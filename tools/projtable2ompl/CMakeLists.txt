
add_executable(projtable2ompl
	projtable2ompl.cpp
)
target_link_libraries(projtable2ompl LibProjectiveGeometry)
install(TARGETS projtable2ompl RUNTIME DESTINATION bin)
set_property(TARGET projtable2ompl PROPERTY FOLDER "Utilities") 

add_executable(ompl2projtable
	ompl2projtable.cpp
)
target_link_libraries(ompl2projtable LibProjectiveGeometry)
install(TARGETS ompl2projtable RUNTIME DESTINATION bin)
set_property(TARGET ompl2projtable PROPERTY FOLDER "Utilities") 

add_executable(ompl2rtkgeometry
	ompl2rtkgeometry.cpp
)
target_link_libraries(ompl2rtkgeometry LibProjectiveGeometry)
install(TARGETS ompl2rtkgeometry RUNTIME DESTINATION bin)
set_property(TARGET ompl2rtkgeometry PROPERTY FOLDER "Utilities") 