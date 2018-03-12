// Created by A. Aichert Fri Mar 1st 2013
#ifndef __itk_utils_hxx
#define __itk_utils_hxx

// itk uses some deprecated xutility functions for which warnings are printed for each an every template specunter VC++
#pragma warning(push)
#pragma warning(disable: 4996)
#pragma warning(disable: 4267)

#include <itkImage.h>
#include <itkMetaDataDictionary.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>

#include <itkImageSeriesReader.h>
#include <itkImageSeriesWriter.h>
#include <itkNumericSeriesFileNames.h>
#include <itkGDCMImageIO.h>
#include <itkGDCMSeriesFileNames.h>


#pragma warning(pop)

#include "GetSet/StringUtil.hxx"

namespace ITKUtils
{

	/// Load an ITK image
	template <typename TImage>
	typename TImage::Pointer loadImage(const std::string& path, itk::MetaDataDictionary* dictionary=0x0)
	{
		typedef itk::ImageFileReader<TImage> ReaderType;
		ReaderType::Pointer reader = ReaderType::New();
		reader->SetFileName(path);
		try {
			reader->Update();
			if (dictionary) *dictionary=reader->GetMetaDataDictionary();
		}
		catch (...) {
			return 0x0;
		}
		return reader->GetOutput();
	}
	
	/// Save an ITK image. Type can be "dicom", "mhd", "nrrd". If type is not given, the extension of filename is used.
	template <typename TImagePtr>
	bool saveImage(const typename TImagePtr image, const std::string& path, itk::MetaDataDictionary* dictionary=0x0)
	{
		typedef itk::ImageFileWriter<TImagePtr::ObjectType> TWriter;
		TWriter::Pointer writer = TWriter::New();
		writer->SetInput(image);
		writer->SetFileName(path);

		if (dictionary)
			writer->SetMetaDataDictionary(*dictionary);

		try {
			writer->Update();
		}
		catch( itk::ExceptionObject & excp )
		{
			std::cerr << "Exception thrown while writing the series " << std::endl;
			std::cerr << excp << std::endl;
			return false;
		}
		return true;
	}
	
	/// Extract raw data from an itk image (no copy, up to four dimensions)
	template <typename TImagePtr>
	void getRawImage(TImagePtr image, typename TImagePtr::ObjectType::PixelType** data, int *w=0x0, int *h=0x0, int *d=0x0, int *k=0x0)
	{
		if (!image)
		{
			*data=0x0;
			*w=0;
			return;
		}
		auto size=image->GetLargestPossibleRegion().GetSize();
		int n=size.GetSizeDimension();
		if (w) *w=size[0];
		if (h) *h=n>1?size[1]:1;
		if (d) *d=n>2?size[2]:1;
		if (k) *k=n>3?size[3]:1;
		if (data)
			*data=image->GetBufferPointer();
	}

	/// Create an n-D itk image and copy image data
	template <typename TPixelType, int N>
	typename itk::Image<TPixelType, N>::Pointer createImage(int *v, TPixelType* data=0x0)
	{
		typedef itk::Image<TPixelType, N> ImageType;
		ImageType::Pointer img=ImageType::New();
		ImageType::IndexType start;
		start.Fill(0);
		ImageType::SizeType size;
		int n=1;
		for (int i=0;i<N;i++)
		{
			size[i]=v[i];
			n*=v[i];
		}
		img->SetRegions(ImageType::RegionType(start,size));
		img->Allocate();
		if (data)
		{
			TPixelType* ptr=img->GetBufferPointer();
			for (int i=0;i<n;i++) ptr[i]=data[i];
		}
		return img;
	}

	/// Create a 2D itk image w x h and copy image data
	template <typename TPixelType>
	typename itk::Image<TPixelType, 2>::Pointer createImage(int w, int h, TPixelType* data=0x0) {
		int v[]={w,h};
		return createImage<TPixelType,2>(v,data);
	}

	/// Create a 3D itk image w x h x d and copy image data
	template <typename TPixelType>
	typename itk::Image<TPixelType, 3>::Pointer createImage(int w, int h, int d, TPixelType* data=0x0) {
		int v[]={w,h,d};
		return createImage<TPixelType,3>(v,data);
	}

	/// Create a 4D itk image w x h x d x k and copy image data
	template <typename TPixelType>
	typename itk::Image<TPixelType, 4>::Pointer createImage(int w, int h, int d, int k, TPixelType* data=0x0) {
		int v[]={w,h,d,k};
		return createImage<TPixelType,4>(v,data);
	}

} // namespace itkutils

#endif // __itk_utils_hxx
