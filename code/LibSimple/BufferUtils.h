#ifndef MHD_BUFFER_UTILS
#define MHD_BUFFER_UTILS

#include <limits>

namespace MHD
{

	/// Computes minimum and maximum in data and returns them through min_out and max_out
	template <typename T>
	void computeMinMax(T* v, int n, T &min_out, T &max_out)
	{
		if (n<=0) return;
		T shared_min=v[0];
		T shared_max=v[0];
		// #pragma omp parallel 
		{
			T min=shared_min;
			T max=shared_max;
			// #pragma omp for nowait
			for(int i=0; i<n; i++)
			{
				if (v[i]<min) min=v[i];
				if (v[i]>max) max=v[i];
			}
			// #pragma omp critical 
			{
				if (min<shared_min) shared_min=min;
				if (max>shared_max) shared_max=max;
			}
		}
		min_out=shared_min;
		max_out=shared_max;
	}

	/// Computes mean and deviation
	template <typename T>
	void computeMeanDev(T* v, int n, double& mean, double& deviation)
	{
		double sum=0;
		double sqsum=0;
		if (n<=0) return;
		for(int i=0; i<n; i++)
		{
			sum+=v[i];
			sqsum+=(double)v[i]*v[i];
		}
		mean=sum/n;
		deviation=std::sqrt(sqsum/n-mean*mean);
	}

	/// Allocates a buffer of type T2 and copies normalized data from T1. Please delete buffer returned.
	template<typename T1, typename T2>
	T2* convertBuffer(T1* src, int n, bool requantize, T2 windowMin=0, T2 windowMax=1)
	{
		if (!requantize)
		{
			T2* dst=new T2[n];
			// #pragma omp parallel for
			for (int i=0;i<n;i++)
			{
				dst[i]=(T2)src[i];
			}
			return dst;
		}
		else
		{
			T1 min,max;
			computeMinMax(src,n,min,max);
			double normBias=-1.0*(double)min;
			double normScale=1.0/((double)max-(double)min);
			if (windowMin!=0||windowMax!=1)
			{
				double windowRange=normScale*((double)windowMax-(double)windowMin);
				return convertBuffer<T1,T2>(src,n,normBias-windowMin/windowRange,windowRange);
			}
			else
				return convertBuffer<T1,T2>(src,n,normBias,normScale); // everything in interval [0,1]
		}
	}

	/// Allocates a buffer of type T and copies a cropped region of input. Please delete buffer returned.
	template<typename T>
	T* cropBuffer(T* input, int inx, int iny, int inz, int minx, int maxx, int miny, int maxy, int minz, int maxz, int c=1)
	{
		int nx=maxx-minx;
		int ny=maxy-miny;
		int nz=maxz-minz;
		T* output=new T[nx*ny*nz*c];
		for (int i=0;i<nx*ny*nz*c;i++)
			output[i]=0;
		for (int z=0;z<nz;z++) if (z<inx)
			for (int y=0;y<ny;y++) if (y<iny)
				for (int x=0;x<nx;x++) if (x<inx)
					for (int i=0;i<c;i++)
					{
						int iout=i+c*x+c*nx*y+c*nx*ny*z;
						int iin=i+c*(x+minx)+c*inx*(y+miny)+c*inx*iny*(z+minz);
						output[iout]=input[iin];
					}
		return output;
	}

	/// Copies element-wise data from src to dst applying bias and scale.
	template<typename T1, typename T2>
	void convertBuffer(T1* src, T2* dst, int n, double bias, double scale)
	{
		if ((bias==0 && scale==1) || scale==0)
			for (int i=0;i<n;i++)
				dst[i]=(T2)src[i];
		else
			for (int i=0;i<n;i++)
				dst[i]=(T2)(((double)src[i]+bias)*scale);
	}

	// Bias and scale
	template<typename T>
	void biasScale(T* dst, int n, double bias, double scale)
	{
		convertBuffer(dst,dst,n,bias,scale);
	}

	/// Allocates a buffer of type T2 and copies element-wise data from T1 applying bias and scale. Please delete buffer returned.
	template<typename T1, typename T2>
	T2* convertBuffer(T1* src, int n, double bias, double scale)
	{
		T2* dst=new T2[n];
		convertBuffer(src,dst,n,bias,scale);
		return dst;	
	}

	// Type general conversion by string...
	inline void convertBuffer(char * src, char * dst, int n, const std::string& type_src, const std::string& type_dst, double bias, double scale)
	{
		if (type_src==type_dst)
			return;

		if (type_src=="MET_CHAR")
		{
			if (type_dst=="MET_CHAR")   convertBuffer<char,char>(src,(char*)dst,n,bias,scale);		
			else if (type_dst=="MET_UCHAR")  convertBuffer<char,unsigned char>(src,(unsigned char*)dst,n,bias,scale);
			else if (type_dst=="MET_SHORT")  convertBuffer<char,short>(src,(short*)dst,n,bias,scale);
			else if (type_dst=="MET_USHORT") convertBuffer<char,unsigned short>(src,(unsigned short*)dst,n,bias,scale);
			else if (type_dst=="MET_INT")    convertBuffer<char,int>(src,(int*)dst,n,bias,scale);
			else if (type_dst=="MET_UINT")   convertBuffer<char,unsigned int>(src,(unsigned int*)dst,n,bias,scale);
			else if (type_dst=="MET_FLOAT")  convertBuffer<char,float>(src,(float*)dst,n,bias,scale);
			else if (type_dst=="MET_DOUBLE") convertBuffer<char,double>(src,(double*)dst,n,bias,scale);
		}
		else if (type_src=="MET_UCHAR")
		{
			if (type_dst=="MET_CHAR")   convertBuffer<unsigned char,char>((unsigned char*)src,(char*)dst,n,bias,scale);		
			else if (type_dst=="MET_UCHAR")  convertBuffer<unsigned char,unsigned char>((unsigned char*)src,(unsigned char*)dst,n,bias,scale);
			else if (type_dst=="MET_SHORT")  convertBuffer<unsigned char,short>((unsigned char*)src,(short*)dst,n,bias,scale);
			else if (type_dst=="MET_USHORT") convertBuffer<unsigned char,unsigned short>((unsigned char*)src,(unsigned short*)dst,n,bias,scale);
			else if (type_dst=="MET_INT")    convertBuffer<unsigned char,int>((unsigned char*)src,(int*)dst,n,bias,scale);
			else if (type_dst=="MET_UINT")   convertBuffer<unsigned char,unsigned int>((unsigned char*)src,(unsigned int*)dst,n,bias,scale);
			else if (type_dst=="MET_FLOAT")  convertBuffer<unsigned char,float>((unsigned char*)src,(float*)dst,n,bias,scale);
			else if (type_dst=="MET_DOUBLE") convertBuffer<unsigned char,double>((unsigned char*)src,(double*)dst,n,bias,scale);
		}
		else if (type_src=="MET_SHORT")
		{
			if (type_dst=="MET_CHAR")   convertBuffer<short,char>((short*)src,(char*)dst,n,bias,scale);		
			else if (type_dst=="MET_UCHAR")  convertBuffer<short,unsigned char>((short*)src,(unsigned char*)dst,n,bias,scale);
			else if (type_dst=="MET_USHORT") convertBuffer<short,unsigned short>((short*)src,(unsigned short*)dst,n,bias,scale);
			else if (type_dst=="MET_INT")    convertBuffer<short,int>((short*)src,(int*)dst,n,bias,scale);
			else if (type_dst=="MET_UINT")   convertBuffer<short,unsigned int>((short*)src,(unsigned int*)dst,n,bias,scale);
			else if (type_dst=="MET_FLOAT")  convertBuffer<short,float>((short*)src,(float*)dst,n,bias,scale);
			else if (type_dst=="MET_DOUBLE") convertBuffer<short,double>((short*)src,(double*)dst,n,bias,scale);
		}
		else if (type_src=="MET_USHORT")
		{
			if (type_dst=="MET_CHAR")   convertBuffer<unsigned short,char>((unsigned short*)src,(char*)dst,n,bias,scale);		
			else if (type_dst=="MET_UCHAR")  convertBuffer<unsigned short,unsigned char>((unsigned short*)src,(unsigned char*)dst,n,bias,scale);
			else if (type_dst=="MET_SHORT")  convertBuffer<unsigned short,short>((unsigned short*)src,(short*)dst,n,bias,scale);
			else if (type_dst=="MET_INT")    convertBuffer<unsigned short,int>((unsigned short*)src,(int*)dst,n,bias,scale);
			else if (type_dst=="MET_UINT")   convertBuffer<unsigned short,unsigned int>((unsigned short*)src,(unsigned int*)dst,n,bias,scale);
			else if (type_dst=="MET_FLOAT")  convertBuffer<unsigned short,float>((unsigned short*)src,(float*)dst,n,bias,scale);
			else if (type_dst=="MET_DOUBLE") convertBuffer<unsigned short,double>((unsigned short*)src,(double*)dst,n,bias,scale);
		}
		else if (type_src=="MET_INT")
		{
			if (type_dst=="MET_CHAR")   convertBuffer<int,char>((int*)src,(char*)dst,n,bias,scale);		
			else if (type_dst=="MET_UCHAR")  convertBuffer<int,unsigned char>((int*)src,(unsigned char*)dst,n,bias,scale);
			else if (type_dst=="MET_SHORT")  convertBuffer<int,short>((int*)src,(short*)dst,n,bias,scale);
			else if (type_dst=="MET_USHORT") convertBuffer<int,unsigned short>((int*)src,(unsigned short*)dst,n,bias,scale);
			else if (type_dst=="MET_UINT")   convertBuffer<int,unsigned int>((int*)src,(unsigned int*)dst,n,bias,scale);
			else if (type_dst=="MET_FLOAT")  convertBuffer<int,float>((int*)src,(float*)dst,n,bias,scale);
			else if (type_dst=="MET_DOUBLE") convertBuffer<int,double>((int*)src,(double*)dst,n,bias,scale);
		}
		else if (type_src=="MET_UINT")
		{
			if (type_dst=="MET_CHAR")   convertBuffer<unsigned int,char>((unsigned int*)src,(char*)dst,n,bias,scale);		
			else if (type_dst=="MET_UCHAR")  convertBuffer<unsigned int,unsigned char>((unsigned int*)src,(unsigned char*)dst,n,bias,scale);
			else if (type_dst=="MET_SHORT")  convertBuffer<unsigned int,short>((unsigned int*)src,(short*)dst,n,bias,scale);
			else if (type_dst=="MET_USHORT") convertBuffer<unsigned int,unsigned short>((unsigned int*)src,(unsigned short*)dst,n,bias,scale);
			else if (type_dst=="MET_INT")    convertBuffer<unsigned int,int>((unsigned int*)src,(int*)dst,n,bias,scale);
			else if (type_dst=="MET_FLOAT")  convertBuffer<unsigned int,float>((unsigned int*)src,(float*)dst,n,bias,scale);
			else if (type_dst=="MET_DOUBLE") convertBuffer<unsigned int,double>((unsigned int*)src,(double*)dst,n,bias,scale);
		}
		else if (type_src=="MET_FLOAT")
		{
			if (type_dst=="MET_CHAR")   convertBuffer<float,char>((float*)src,(char*)dst,n,bias,scale);		
			else if (type_dst=="MET_UCHAR")  convertBuffer<float,unsigned char>((float*)src,(unsigned char*)dst,n,bias,scale);
			else if (type_dst=="MET_SHORT")  convertBuffer<float,short>((float*)src,(short*)dst,n,bias,scale);
			else if (type_dst=="MET_USHORT") convertBuffer<float,unsigned short>((float*)src,(unsigned short*)dst,n,bias,scale);
			else if (type_dst=="MET_INT")    convertBuffer<float,int>((float*)src,(int*)dst,n,bias,scale);
			else if (type_dst=="MET_UINT")   convertBuffer<float,unsigned int>((float*)src,(unsigned int*)dst,n,bias,scale);
			else if (type_dst=="MET_DOUBLE") convertBuffer<float,double>((float*)src,(double*)dst,n,bias,scale);
		}
		else if (type_src=="MET_DOUBLE")
		{
			if (type_dst=="MET_CHAR")   convertBuffer<double,char>((double*)src,(char*)dst,n,bias,scale);		
			else if (type_dst=="MET_UCHAR")  convertBuffer<double,unsigned char>((double*)src,(unsigned char*)dst,n,bias,scale);
			else if (type_dst=="MET_SHORT")  convertBuffer<double,short>((double*)src,(short*)dst,n,bias,scale);
			else if (type_dst=="MET_USHORT") convertBuffer<double,unsigned short>((double*)src,(unsigned short*)dst,n,bias,scale);
			else if (type_dst=="MET_INT")    convertBuffer<double,int>((double*)src,(int*)dst,n,bias,scale);
			else if (type_dst=="MET_UINT")   convertBuffer<double,unsigned int>((double*)src,(unsigned int*)dst,n,bias,scale);
			else if (type_dst=="MET_FLOAT")  convertBuffer<double,float>((double*)src,(float*)dst,n,bias,scale);
		}
	}

} // namespace MHD

#endif
