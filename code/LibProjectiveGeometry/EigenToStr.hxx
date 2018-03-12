#ifndef __eigen_to_str
#define __eigen_to_str
// Created by A. Aichert on Thu Oct 9th 2014

#include <GetSet/StringUtil.hxx>

#include "ProjectionMatrix.h"

// Vectors

template <class EigenVector, int N> inline EigenVector stringToEigenVector(const std::string& in)
{
	EigenVector ret=EigenVector::Zero();
	std::string s=in;
	std::replace( s.begin(), s.end(), '[', ' ');
	std::replace( s.begin(), s.end(), ';', ' ');
	std::replace( s.begin(), s.end(), ',', ' ');
	auto raw=stringToVector<double>(s,' ');
	if (raw.size()>=N)
		for (int i=0;i<N;i++)
			ret[i]=(typename EigenVector::Scalar)raw[i];
	return ret;
}

template <> inline std::string toString<>(const Eigen::Vector2i& in) { return toString(in(0))+" "+toString(in(1));}
template <> inline std::string toString<>(const Eigen::Vector3i& in) { return toString(in(0))+" "+toString(in(1))+" "+toString(in(2));}
template <> inline std::string toString<>(const Eigen::Vector4i& in) { return toString(in(0))+" "+toString(in(1))+" "+toString(in(2))+" "+toString(in(3));}

template <> inline Eigen::Vector2i stringTo<>(const std::string& in) { return stringToEigenVector<Eigen::Vector2i,2>(in); }
template <> inline Eigen::Vector3i stringTo<>(const std::string& in) { return stringToEigenVector<Eigen::Vector3i,3>(in); }
template <> inline Eigen::Vector4i stringTo<>(const std::string& in) { return stringToEigenVector<Eigen::Vector4i,4>(in); }

template <> inline std::string toString<>(const Eigen::Vector2d& in) { return toString(in(0))+" "+toString(in(1));}
template <> inline std::string toString<>(const Eigen::Vector3d& in) { return toString(in(0))+" "+toString(in(1))+" "+toString(in(2));}
template <> inline std::string toString<>(const Eigen::Vector4d& in) { return toString(in(0))+" "+toString(in(1))+" "+toString(in(2))+" "+toString(in(3));}

template <> inline Eigen::Vector2d stringTo<>(const std::string& in) { return stringToEigenVector<Eigen::Vector2d,2>(in); }
template <> inline Eigen::Vector3d stringTo<>(const std::string& in) { return stringToEigenVector<Eigen::Vector3d,3>(in); }
template <> inline Eigen::Vector4d stringTo<>(const std::string& in) { return stringToEigenVector<Eigen::Vector4d,4>(in); }


// Vectors of vectors

template <> inline std::string toString<>(const std::vector<Eigen::Vector2d>& in) {return vectorToString(in,";"); }
template <> inline std::string toString<>(const std::vector<Eigen::Vector3d>& in) {return vectorToString(in,";"); }
template <> inline std::string toString<>(const std::vector<Eigen::Vector4d>& in) {return vectorToString(in,";"); }

template <> inline std::vector<Eigen::Vector2d> stringTo<>(const std::string& in) { return stringToVector<Eigen::Vector2d>(in,';'); }
template <> inline std::vector<Eigen::Vector3d> stringTo<>(const std::string& in) { return stringToVector<Eigen::Vector3d>(in,';'); }
template <> inline std::vector<Eigen::Vector4d> stringTo<>(const std::string& in) { return stringToVector<Eigen::Vector4d>(in,';'); }

// Matrices

template <> inline std::string toString<>(const Eigen::Matrix3d& in) { return std::string("[") +
	toString(in(0,0))+" "+toString(in(0,1))+" "+toString(in(0,2))+"; "+
	toString(in(1,0))+" "+toString(in(1,1))+" "+toString(in(1,2))+"; "+
	toString(in(2,0))+" "+toString(in(2,1))+" "+toString(in(2,2))+"] ";
}

template <> inline std::string toString<>(const Eigen::Matrix4d& in) { return std::string("[") +
	toString(in(0,0))+" "+toString(in(0,1))+" "+toString(in(0,2))+" "+toString(in(0,3))+"; "+
	toString(in(1,0))+" "+toString(in(1,1))+" "+toString(in(1,2))+" "+toString(in(1,3))+"; "+
	toString(in(2,0))+" "+toString(in(2,1))+" "+toString(in(2,2))+" "+toString(in(2,3))+"; "+
	toString(in(3,0))+" "+toString(in(3,1))+" "+toString(in(3,2))+" "+toString(in(3,3))+"] ";
}

template <> inline std::string toString<>(const Geometry::ProjectionMatrix& in) {
	std::ostringstream strstr;
	strstr << std::setprecision(12) << "["
		<< in(0,0) << " " << in(0,1) << " " << in(0,2) << " " << in(0,3) << "; "
		<< in(1,0) << " " << in(1,1) << " " << in(1,2) << " " << in(1,3) << "; "
		<< in(2,0) << " " << in(2,1) << " " << in(2,2) << " " << in(2,3) << "] ";
	return strstr.str();
}

template <> inline Eigen::Matrix3d stringTo<>(const std::string& in)
{
	Eigen::Matrix3d ret=Eigen::Matrix3d::Identity();
	std::string s=in;
	std::replace( s.begin(), s.end(), '[', ' ');
	std::replace( s.begin(), s.end(), ';', ' ');
	std::replace( s.begin(), s.end(), ',', ' ');
	auto raw=stringToVector<double>(s,' ');
	if (raw.size()>=3*3)
		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
				ret(i,j)=raw[i*3+j];
	return ret;
}

template <> inline Eigen::Matrix4d stringTo<>(const std::string& in)
{
	Eigen::Matrix4d ret=Eigen::Matrix4d::Identity();
	std::string s=in;
	std::replace( s.begin(), s.end(), '[', ' ');
	std::replace( s.begin(), s.end(), ';', ' ');
	std::replace( s.begin(), s.end(), ',', ' ');
	auto raw=stringToVector<double>(s,' ');
	if (raw.size()==4*4)
		for (int i=0;i<4;i++)
			for (int j=0;j<4;j++)
				ret(i,j)=raw[i*4+j];
	return ret;
}

template <> inline Geometry::ProjectionMatrix stringTo<>(const std::string& in)
{
	Geometry::ProjectionMatrix ret=Geometry::ProjectionMatrix::Zero();
	ret(0,0)=ret(1,1)=ret(2,2)=1;
	std::string s=in;
	std::replace( s.begin(), s.end(), '\n', ' ');
	std::replace( s.begin(), s.end(), '\t', ' ');
	std::replace( s.begin(), s.end(), '[', ' ');
	std::replace( s.begin(), s.end(), ';', ' ');
	std::replace( s.begin(), s.end(), ',', ' ');
	auto raw=stringToVector<double>(s,' ');
	if (raw.size()==3*4)
		for (int i=0;i<3;i++)
			for (int j=0;j<4;j++)
				ret(i,j)=raw[i*4+j];
	return ret;
}

#endif // __eigen_to_str
