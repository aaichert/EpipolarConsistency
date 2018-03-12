// Created by A. Aichert in Aug 2016
#ifndef __NRRD_METAIO_HXX
#define __NRRD_METAIO_HXX

// Simple header-only implementation of the ITK/VTK MetaIO file format with local data file.
// Only implements: 
//    ObjectType = Image
//    BinaryData = True
//    CompressedData = False
//    ElementDataFile = LOCAL
// Only supports
//    ElementType =
//       MET_FLOAT
//       MET_DOUBLE
//       MET_CHAR MET_UCHAR
//       MET_SHORT MET_USHORT
//       MET_INT MET_UINT
// Correctly handles the following tags:
//    NDims
//    BinaryDataByteOrderMSB
//    ElementSpacing
//    DimSize


#include "nrrd_image.hxx"

namespace NRRD
{

	inline std::string typeToMET(std::string type)
	{
		std::transform(type.begin(), type.end(), type.begin(), tolower);
		if (false) ;
		else if (type==typeName<float>())	return "MET_FLOAT";
		else if (type==typeName<double>())	return "MET_DOUBLE";
		else if (type==typeName<char>())	return "MET_CHAR";
		else if (type==typeName<short>())	return "MET_USHORT";
		else if (type==typeName<int>())		return "MET_INT";
		else if (type==typeName<unsigned char>())	return "MET_UCHAR";
		else if (type==typeName<unsigned short>())	return "MET_SHORT";
		else if (type==typeName<unsigned int>())	return "MET_UINT";
		else return "";
	}

	inline std::string typeFromMET(std::string type)
	{
		std::transform(type.begin(), type.end(), type.begin(), tolower);
		if (false) ;
		else if (type=="MET_FLOAT")		return typeName<float>();
		else if (type=="MET_DOUBLE")	return typeName<double>();
		else if (type=="MET_CHAR")		return typeName<char>();
		else if (type=="MET_SHORT")		return typeName<short>();
		else if (type=="MET_INT")		return typeName<int>();
		else if (type=="MET_UCHAR")		return typeName<unsigned char>();
		else if (type=="MET_UINT")		return typeName<unsigned short>();
		else if (type=="MET_USHORT")	return typeName<unsigned int>();
		else return "";
	}

	/// Save raw data in MHA file.
	template <typename T>
	bool saveMHA(const std::string& file, const NRRD::ImageView<T>& img )
	{
		std::ofstream mha(file.c_str(),std::ios::binary);
		if (!mha||!mha.good())
			return false;
		// Write MetaIO header
		mha
			<< "ObjectType = Image\n"
			<< "NDims = " << img.dimension() << "\n"
			<< "BinaryData = True\n"
			<< "BinaryDataByteOrderMSB = " << (is_cpu_BIG_endian()?"True":"False") << "\n"
			<< "CompressedData = False\n"
			<< "ElementSpacing = " << img.spacing(0) << " " << img.spacing(1) << " " << img.spacing(2) << "\n"
			<< "ElementType = " << typeToMET(typeName<T>())
			<< "ElementDataFile = LOCAL\n";
		// Write raw data chunk
		mha.write((char*)((T*)img),sizeof(T)*img.length());
		return true;
	}

} // namespace NRRD


#endif // __NRRD_HXX
