#ifndef __NRRD_QT_HXX
#define __NRRD_QT_HXX

#include <NRRD/nrrd_image.hxx>

#include <QImage>
#include <QPixmap>
#include <QPainter>

namespace NRRD {
	
	/// Convert three channel 2D NRRD::Image to QImage
	template <typename T>
	inline void nrrdFromQImage(const QImage& input_image, NRRD::Image<T>& output_image, double bias=0, double scale=1.0/255.0)
	{
		if (input_image.isGrayscale())
		{
			// Convert to 8bit grayscale image and copy data
			QImage qtimg=input_image.convertToFormat(QImage::Format_Grayscale8);
			// Get image size
			int w=qtimg.width();
			int h=qtimg.height();
			// Allocate NRRD image
			output_image.set(w,h);
			// Copy intensity pixel by pixel
//			#pragma omp parallel for
// 			for (int y=0;y<h;y++)
// 				for (int x=0;x<w;x++)
// 					output_image.pixel(x,y,0)=(T)(input_image.pixelColor(x,y).red()*scale+bias);
		}
		else
		{
			// Get image size
			int w=input_image.width();
			int h=input_image.height();
			// Allocate NRRD image
			output_image.set(3,w,h);
			output_image.setElementKindColor();
			// Copy RGB color pixel by pixel
			#pragma omp parallel for
			for (int y=0;y<h;y++)
			{
				T *line=(T*)output_image+y*w*3;
// 				for (int x=0;x<w;x++)
// 				{
// 					QColor pixel=input_image.pixelColor(x,y);
// 					*(line++)=(T)(pixel.red()*scale+bias);
// 					*(line++)=(T)(pixel.green()*scale+bias);
// 					*(line++)=(T)(pixel.blue()*scale+bias);
// 				}
			}
		}
	}

	/// Define how to normalize NRRD images
	template <typename T>
	inline double nrrdComputeBiasAndScale(const NRRD::ImageView<T>& img, double& bias, double& scale)
	{
		if (!img) return -1;
		T max=img[0];
		int l=img.length();
		#pragma omp parallel for
		for (int i=0;i<l;i++)
			if (max<img[i]) max=img[i];
		bias=0;
		return scale=1.0/(double)max;
	}

	/// Convert single channel 2D NRRD::Image to QImage
	template <typename T>
	inline void nrrdToQImage(const NRRD::ImageView<T>& img, QImage& qtimg, double bias=0, double scale=0, bool is_signed=false)
	{
		// Delete existing image
		qtimg=QImage();
		// Convert to unsigned char
		int n_xy;
		int l=img.length();
		bool color=img.isElementKindColor();
		//nAllocate destination image
		if (!color)
		{
			// Make sure we have enough input data
			if (img.dimension()<2) return;
			n_xy=img.size(0)*img.size(1);
			qtimg=QImage(img.size(0),img.size(1),QImage::Format::Format_RGB32);
		}
		else
		{
			n_xy=img.size(1)*img.size(2);
			qtimg=QImage(img.size(1),img.size(2),QImage::Format::Format_RGB32);
		}
		// Normalize image intensities (unless scale is given)
		if (scale==0)
			nrrdComputeBiasAndScale(img, bias, scale);
		
		if (typeName<T>()!="unsigned char")
			scale*=255;

		// Access pixel data directly
		unsigned char * uc_data=qtimg.bits();
		if (!color || img.size(0)==1)
		{
			#pragma omp parallel for
			for (int i=0;i<n_xy;i++)
			{
				double c=(img[i]+bias)*scale;
				if (is_signed) c=128+c;
				if (c<0)c=0;
				if (c>255)c=255;
				uc_data[i*4+0]=uc_data[i*4+1]=uc_data[i*4+2]=uc_data[i*4+3]=(unsigned char)c;
			}
		}
		else
		{
			const int c=img.size(0);
			#pragma omp parallel for
			for (int i=0;i<n_xy;i++)
			{
				double r=(img[i*c+0]+bias)*scale;
				if (is_signed) r=128+r;
				if (r<0)r=0;
				if (r>255)r=255;
				uc_data[i*4+2]=r;
				
				if (c>1) // rg
				{
					double g=(img[i*c+1]+bias)*scale;
					if (is_signed) g=128+g;
					if (g<0)g=0;
					if (g>255)g=255;
					uc_data[i*4+1]=g;
				}
				else
				{
					uc_data[i*4+0]=r;
					uc_data[i*4+1]=r;
				}
				if (c>2) // rgb
				{
					double b=(img[i*c+2]+bias)*scale;
					if (is_signed) b=128+b;
					if (b<0)b=0;
					if (b>255)b=255;
					uc_data[i*4+0]=b;
				}
				if (c>3) // rgba
				{
					float a=(img[i*c+3]+bias)*scale;
					if (is_signed) a=128+a;
					if (a<0)a=0;
					if (a>255)a=255;
					uc_data[i*4+3]=a;
				}
			}
		}
	}

} // namespace NRRD

#endif // __NRRD_QT_HXX
