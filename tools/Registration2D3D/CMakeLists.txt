
# Add DRR tool
add_executable(ComputeDRR
	ComputeDRR.cpp
)
target_link_libraries(ComputeDRR GetSet GetSetGui LibSimple LibProjectiveGeometry LibUtilsQt GLAD)
qt5_use_modules(ComputeDRR Gui OpenGL)
install(TARGETS ComputeDRR RUNTIME DESTINATION ".")

# Add Registration Tool
add_executable(Registration2D3D
	Registration2D3D.h
	Registration2D3D.cpp
	main.cpp
)
target_link_libraries( Registration2D3D GetSet GetSetGui LibSimple LibProjectiveGeometry LibUtilsQt GLAD)
qt5_use_modules(Registration2D3D Gui OpenGL)
install(TARGETS Registration2D3D RUNTIME DESTINATION ".")