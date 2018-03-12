#ifndef __camera_open_gl
#define __camera_open_gl
// Created by A. Aichert on Tue April 12th 2015
// Conversion between OpenGL ModelView/Projection Matrices and 3x4 ProjectionMatrix (Hartley style)

#include "ProjectionMatrix.h"

namespace Geometry
{
	/// Intrinsic parameters of a pinhole camera as a 3x3 matrix
	inline Eigen::Matrix3d cameraPerspective(double fovy_rad, double width, double height) // Bug?
	{
		double tanfov2=2*std::tan(0.5*fovy_rad);
		return makeCalibrationMatrix(height/tanfov2, height/tanfov2, 0.5*width, 0.5*height);
	}
	
	/// A function similar to gluLookAt
	inline ProjectionMatrix cameraLookAt(const Eigen::Matrix3d& K,
		const Eigen::Vector3d& eye, const Eigen::Vector3d& center,
		Eigen::Vector3d up=Eigen::Vector3d(0,1,0) )
	{
		// Make orthonormal basis
		auto fwd=(center-eye).normalized();
		auto left=up.cross(fwd).normalized();
		up=fwd.cross(left);
		Eigen::Matrix3d R;
		R.block<1,3>(0,0)=left;
		R.block<1,3>(1,0)=up;
		R.block<1,3>(2,0)=-fwd;
		return makeProjectionMatrix(K,R,-R*eye);
	}

	/// Convert from OpenGL ModelView Projection Matrix to ProjectionMatrix (world to image pixels). If flip Y is specified, image origin is in upper left corner.
	inline ProjectionMatrix modelViewProjectionMatrixOpenGL(Eigen::Matrix4d MVP, double image_width, double image_height, bool flipY)
	{
		ProjectionMatrix P;
		// Correct for axis flip
		Eigen::Matrix4d Fix = Eigen::Matrix4d::Identity();
		Fix(0, 0) =1; Fix(0, 1) = 0; Fix(0, 2) = 0;
		Fix(1, 0) = 0; Fix(1, 1) = -1; Fix(1, 2) = 0; // FIXME Fix(1, 1) = -1; is enough
		Fix(2, 0) = 0; Fix(2, 1) = 0; Fix(2, 2) = 1; 
		MVP = Fix*MVP;

		// Remove third row (z-coordinate)
		P.block<2,4>(0,0)=MVP.block<2,4>(0,0);
		P.block<1,4>(2,0)=MVP.block<1,4>(3,0);
		
		// Finally, convert normalized device coordinates to image pixels
		Eigen::Matrix3d H=Eigen::Matrix3d::Identity();
		// Shift and Scale to pixels (assuming the current image has the same size as g_texture)
		H(0,2)=H(0,0)=image_width/2;
		H(1,2)=H(1,1)=image_height/2;
		P=H*P;

		// Flip image Y-axis
		if (flipY)
		{
			Eigen::Matrix3d H=Eigen::Matrix3d::Identity();
			H(1,1)=-1;
			H(1,2)=image_height;
			P=H*P;
		}

		// Return normalized matrix
		normalizeProjectionMatrix(P);
		return P;
	}

	/// Build an OpenGL Projection Matrix out of intrinsic parameters
	inline Eigen::Matrix4d intrinsicParametersToOpenGL(Eigen::Matrix3d K, double left, double right, double bottom, double top, double neap, double farp)
	{
		// Negate last column of K
		K.block<3,1>(0,2)*=-1.0;
		// Add a third row to store depth and turn K into a 4x4 matrix
		Eigen::Matrix4d Pgl=Eigen::Matrix4d::Zero();
		Pgl.block<2,3>(0,0)=K.block<2,3>(0,0);
		Pgl.block<1,3>(3,0)=K.block<1,3>(2,0);
		Pgl(2,2)=neap+farp;
		Pgl(2,3)=neap*farp;
		Pgl(3,2)=-1;
		// Conversion to NDC (glOrtho-style)
		Eigen::Vector3d t((right+left)/(right-left), (top+bottom)/(top-bottom), (farp+neap)/(farp-neap));
		Eigen::Vector4d s(2/(right-left),2/(top-bottom),-2/(farp-neap),1);
		Eigen::Matrix4d NDC=s.asDiagonal();
		NDC.block<3,1>(0,3)=-t;
		return NDC*Pgl;	
	}

	// Decompose Projection Matrix into OpenGL ModelView and Projection Matrices
	inline void projectionMatrixToOpenGL(ProjectionMatrix P, Eigen::Matrix4d& MVgl, Eigen::Matrix4d& Pgl, double left, double right, double bottom, double top, double nearp, double farp, bool left_handed=false)
	{
		// Decompose P
		Eigen::Matrix3d K, R;
		Eigen::Vector3d t;
		normalizeProjectionMatrix(P);
		bool rhs=projectionMatrixDecomposition(P,K,R,t,bottom<top);
		// Assemble rigid transformation
		MVgl=Eigen::Matrix4d::Identity();
		MVgl.block<3,3>(0,0)=R;
		MVgl.block<3,1>(0,3)=t;
		if (left_handed)
			MVgl.block<1,4>(2,0)*=-1;
		// Convert intrinsic parameters to OpenGL
		Pgl=intrinsicParametersToOpenGL(K,left,right,bottom,top,nearp,farp);
	}

} // namespace Geometry

#endif // __camera_open_gl
