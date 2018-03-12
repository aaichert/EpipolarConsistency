#ifndef __geometry_visualization
#define __geometry_visualization
// Created by aaichert on Wed Oct 29th 2014
// Utility functions for drawing lines in two and three-space and planes as well as cameras

#include "SourceDetectorGeometry.h"

namespace Geometry
{
	
	inline Geometry::RP3Plane cameraFrustum(const Geometry::ProjectionMatrix& P, const Eigen::Vector4d& image_rect, double pixelSize, std::vector<Geometry::RP3Point>& Xs)
	{
		// 2do reimplement using proj. to plane

		using namespace Geometry;
		// Decompose P
		Eigen::Matrix3d K, R;
		Eigen::Vector3d t;
		projectionMatrixDecomposition(P, K, R, t);
		// Camera Center
		auto C=getCameraCenter(P);
		// Focal Length in world units (typically millimeters)
		double focalLengthX=K(0,0)*pixelSize;
		// Principal Plane
		Eigen::Vector4d E=P.block<1,4>(2,0);
		// Scales plane normal to unit length
		double e=1./E.block<3,1>(0,0).norm();
		E=E*e;
		// Move plane by focal length (i.e. image plane)
		E[3]-=focalLengthX;	
		// Backproject image corners
		ProjectionMatrixInverse Pinv=pseudoInverse(P);
		// The camera center and 4 corner points of the detector, as well as x- and y-axes
		Xs.clear();
		Xs.push_back(C);
		Xs.push_back(Pinv*RP2Point(image_rect[0],image_rect[1],1));
		Xs.push_back(Pinv*RP2Point(image_rect[0],image_rect[3],1));
		Xs.push_back(Pinv*RP2Point(image_rect[2],image_rect[3],1));
		Xs.push_back(Pinv*RP2Point(image_rect[2],image_rect[1],1));
//		Xs.push_back(C + RP3Point(E[0], E[1], E[2], 0)); // Check same as below.
		Xs.push_back(Pinv*RP2Point(K(0, 2), K(1, 2), 1));
		// axes: (principal point + x/y max)
		Xs.push_back(Pinv*RP2Point(image_rect[2],K(1,2),1));
		Xs.push_back(Pinv*RP2Point(K(0,2),image_rect[3],1));
		// Intersect backprojection rays with image plane
		for (int i=1;i<8;i++)
		{
			// Backprojection Ray
			auto L=join_pluecker(Xs[i],C);
			Xs[i]=meet_pluecker(L,E);
			Xs[i]=Xs[i]/Xs[i][3];
		}
		return E;
	}

	// Four oriented lines enclosing a rectangle.
	inline std::vector<Geometry::RP2Line> box(double left, double bottom, double right, double top)
	{
		std::vector<Geometry::RP2Line> ret(4);
		ret[0]=join(RP2Point(left,bottom,1),RP2Point(right,bottom,1));
		ret[1]=join(RP2Point(right,bottom,1),RP2Point(right,top,1));
		ret[2]=join(RP2Point(right,top,1),RP2Point(left,top,1));
		ret[3]=join(RP2Point(left,top,1),RP2Point(left,bottom,1));
		return ret;
	}

	// Returns entry and exit of a ray intersecting a convex shape made up of oriented lines.
	inline std::pair<RP2Point,RP2Point> intersectLineWithConvex(const Geometry::RP2Line& l, const std::vector<Geometry::RP2Line>& shape)
	{
		// Compute all intersections
		std::pair<RP2Point,RP2Point> ret;
		for (int i=0;i<(int)shape.size();i++)
		{
			bool inside=true;
			auto intersection=meet(l,shape[i]);
			if (std::abs(intersection(2))>1e-12)
				intersection/=intersection(2);
			// Test if they are inside
			for (int j=0;j<(int)shape.size();j++)
				if (j!=i&&intersection.transpose()*shape[j]<0)
				{
					inside=false;
					break;
				}
			// If inside, they are a valid intersection
			if (inside)
				if (ret.first==RP2Point())
					ret.first=intersection;
				else if (ret.second==RP2Point())
					ret.second=intersection;
				else // Intersection with a corner
				{
					// replace if euclidian distance is greater
					double d_them=(ret.first-ret.second).norm();
					double d_us=(ret.first-intersection).norm();
					if (d_us>d_them)
						ret.second=intersection;
				}
		}
		// Make sure order is correct: l is line from first to second (i.e. join(second,first) )
		if (join(ret.second,ret.first).transpose()*l<0)
			std::swap(ret.first,ret.second);
		return ret;
	}

	// Intersects line with a rect. Returns x1 y1 x2 y2 in a four vector. Use this for drawing lines.
	inline Eigen::Vector4d intersectLineWithRect(const Geometry::RP2Line& l, int n_x, int n_y )
	{
		using Geometry::RP2Line;
		using Geometry::RP2Point;
		using Geometry::meet;
		const double eps=1E-6;
		// Find intersections with image boundaries
		std::vector<RP2Line> intersection(4);
		intersection[0]=meet(l,RP2Line(1.f,0.f,0.f));
		intersection[1]=meet(l,RP2Line(-1.f,0.f,(float)n_x-1.f));
		intersection[2]=meet(l,RP2Line(0.f,1.f,0.f));
		intersection[3]=meet(l,RP2Line(0.f,-1.f,(float)n_y-1.f));
		RP2Point from=RP2Point(-1.f,-1.f,-1.f), to=RP2Point(-1.f,-1.f,-1.f);
		// Find intersections which are in bounds
		for (int i=0;i<4;i++)
		{
			intersection[i]/=intersection[i](2);
			if (intersection[i](0)<=n_x+eps&&intersection[i](1)<=n_y+eps&&intersection[i](0)+eps>=0&&intersection[i](1)+eps>=0)
				if (from(0)<0.f)
					from=RP2Point(intersection[i](0),intersection[i](1),0);
				else
				{
					if (to(0)<0.f)
						to=RP2Point(intersection[i](0),intersection[i](1),0);
					else
					{
						// This may happen if a corner coincides with the line.
						// Then, we have to use two intersections, which are far apart to get the line.
						RP2Point to2=RP2Point(intersection[i](0),intersection[i](1),0);
						if ((from-to).norm()<(from-to2).norm())
							to=to2;
					}
				}
		}
		return Eigen::Vector4d(from(0),from(1),to(0),to(1));
	}
	
} // namespace Geometry
 
#endif // __geometry_visualization
