
add_library(GLAD
	GL/glad.c
)
set_source_files_properties(GL/glad.c PROPERTIES LANGUAGE C)
target_compile_definitions(GLAD PUBLIC GLAD_GLAPI_EXPORT PRIVATE GLAD_GLAPI_EXPORT_BUILD)

add_library(LibSimple
	BufferUtils.h
	SimpleOptimizer.h
	SimpleOptimizer.cpp
	SimpleMHD.h
	SimpleMHD.cpp
	SimpleGL.h
	SimpleGL.cpp
	SimpleQTGL.h
	SimpleQTGL.cpp
	SimpleVR.h
	SimpleVR.cpp
	SimpleRayCastPass.h
	SimpleRayCastPass.cpp
)
target_link_libraries(LibSimple PRIVATE GLAD Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets)