# A libbrary for advanced 2D and 3D projective geometry
add_library(LibProjectiveGeometry
	# toString and stringTo<...> function for a few Eigen types.
	EigenToStr.hxx
	# Geometry of 2D point and lines, as well as 3D points, Plücker lines and planes.
	ProjectiveGeometry.hxx
	# The 3x4 projection  matrix and its geometric interpretation as pinhole camera.
	ProjectionMatrix.h
	ProjectionMatrix.cpp
	# The SVD implementation is compiled separately to reduce compilation times.
	SingularValueDecomposition.h
	SingularValueDecomposition.cpp
	# Interpretation as the projection matrix as an X-ray source-detector geometry.
	SourceDetectorGeometry.h
	SourceDetectorGeometry.cpp
	# Linear estimation of the projection matrix.
	EstimateProjectionMatrix.h
	EstimateProjectionMatrix.cpp
	# Conversion of OpenGL and 3x4 projection matrices.
	CameraOpenGL.hxx
	# An over-parametrization of 8DOF 2D projective transformations.
	Models/ModelHomography2D.hxx
	Models/ModelHomography3D.hxx
	# Parametization of a similarity transformation.
	Models/ModelSimilarity2D.hxx
	Models/ModelSimilarity3D.hxx
	# Parametrization of 2D and 3D similarity transformations acting on a projection matrix.
	Models/ModelCameraSimilarity2D3D.hxx
	# Parametrization of a projection matrix acoording to the IEC 61217 standard (RTK).
	Models/ModelTrajectoryIEC61217.hxx
	# Obsolete alternative Parametrization of a projection matrix.
	Models/ModelFDCTCalibrationCorrection.hxx
)
install(FILES EigenToStr.hxx ProjectiveGeometry.hxx ProjectionMatrix.h SingularValueDecomposition.h CameraOpenGL.hxx GeometryVisualization.hxx DESTINATION include/LibProjectiveGeometry)
install(FILES Models/ModelFDCTCalibrationCorrection.hxx Models/ModelCameraSimilarity2D3D.hxx Models/ModelSimilarity2D.hxx Models/ModelHomography2D.hxx Models/ModelSimilarity3D.hxx Models/ModelHomography3D.hxx Models/ModelTrajectoryIEC61217.hxx DESTINATION include/LibProjectiveGeometry/Models)
install(TARGETS LibProjectiveGeometry DESTINATION lib EXPORT LibProjectiveGeometry-targets)
install(EXPORT LibProjectiveGeometry-targets DESTINATION cmake)
set_property(TARGET LibProjectiveGeometry PROPERTY FOLDER "Packages") 
