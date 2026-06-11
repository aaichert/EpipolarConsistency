
# Add DRR tool
add_executable(ComputeDRR
	ComputeDRR.cpp
)
target_link_libraries(ComputeDRR PRIVATE
	GetSet
	GetSetGui
	LibSimple
	LibProjectiveGeometry
	LibUtilsQt
	GLAD
	Qt6::Gui
	Qt6::OpenGL
	Qt6::OpenGLWidgets
)
install(TARGETS ComputeDRR RUNTIME DESTINATION ".")

# Add Registration Tool
add_executable(Registration2D3D
	Registration2D3D.h
	Registration2D3D.cpp
	main.cpp
)
target_link_libraries( Registration2D3D PRIVATE
	GetSet
	GetSetGui
	LibSimple
	LibProjectiveGeometry
	LibUtilsQt
	GLAD
	Qt6::Gui
	Qt6::OpenGL
	Qt6::OpenGLWidgets
)
install(TARGETS Registration2D3D RUNTIME DESTINATION ".")