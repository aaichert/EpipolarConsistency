#ifndef SIMPLE_HMD
#define SIMPLE_HMD

/** @brief	Simple type-generic Meta Image File loading and saving
 *  @author	Andre Aichert
 */

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <map>

#include "BufferUtils.h"

namespace MHD
{
	/// Get a Meta Image type string by type of data
	template <typename MHD_TYPE> static std::string getTypeStr() {return "MET_UNKNOWN_TYPE";}
	template <typename MHD_TYPE> static std::string getTypeStr(MHD_TYPE* unused) {return getTypeStr<MHD_TYPE>();}
	template <> inline std::string getTypeStr<char>()           {return "MET_CHAR";}
	template <> inline std::string getTypeStr<unsigned char>()  {return "MET_UCHAR";}
	template <> inline std::string getTypeStr<short>()          {return "MET_SHORT";}
	template <> inline std::string getTypeStr<unsigned short>() {return "MET_USHORT";}
	template <> inline std::string getTypeStr<int>()            {return "MET_INT";}
	template <> inline std::string getTypeStr<unsigned int>()   {return "MET_UINT";}
	template <> inline std::string getTypeStr<float>()          {return "MET_FLOAT";}
	template <> inline std::string getTypeStr<double>()         {return "MET_DOUBLE";}


	/// Get byte size of a type
	static int getTypeSize(const std::string& type)
	{
		if (type==getTypeStr<char>())			return sizeof(char);
		if (type==getTypeStr<unsigned char>())	return sizeof(unsigned char);
		if (type==getTypeStr<short>())			return sizeof(short);
		if (type==getTypeStr<unsigned short>())	return sizeof(unsigned short);
		if (type==getTypeStr<int>())			return sizeof(int);
		if (type==getTypeStr<unsigned int>())	return sizeof(unsigned int);
		if (type==getTypeStr<float>())			return sizeof(float);
		if (type==getTypeStr<double>())			return sizeof(double);
		return -1;
	}
	
	/// Get a vector of all supported MHD types
	static const std::vector<std::string>& getSupportedTypes()
	{
		static std::vector<std::string> types;
		if (types.empty())
		{
			types.push_back(getTypeStr<char>());
			types.push_back(getTypeStr<unsigned char>());
			types.push_back(getTypeStr<short>());
			types.push_back(getTypeStr<unsigned short>());
			types.push_back(getTypeStr<int>());
			types.push_back(getTypeStr<unsigned int>());
			types.push_back(getTypeStr<float>());
			types.push_back(getTypeStr<double>());
		}
		return types;
	}

	/// Interface for type general data access
	class DataInterface
	{
	protected:
		/// Data dimensions in voxels
		int 	m_dims[3];
		/// Number of data channels (usually 1)
		int 	m_channels;
		/// Element spacing between consecutive voxels
		double	m_spacing[3];
		/// Does the endian of this machine match that of the data (usually true)
		bool	m_localEndian;
		/// Number of elements of MHD_TYPE allocated in m_data (=m_dims[0]*m_dims[1]*m_dims[2]*m_channels)
		int		m_nDataElements;

	public:
		DataInterface();
		virtual ~DataInterface();

		/// Create an instance of MHD::Data for a specific type. Returns null for unknown type string.
		static DataInterface * createForType(const std::string& type);

		/// Get dimensions
		inline int dim(int i) const {return m_dims[i];}
		// Get number of channels
		inline int channels() const {return m_channels;}
		/// Get dimensions
		inline double spacing(int i) const {return m_spacing[i];}
		/// Get physical size in [mm]
		inline double physicalSize(int i) const {return m_dims[i]*m_spacing[i];}

		/// Abstract allocament (implemented by Data<MHD_TYPE>)
		virtual void alloc() = 0;
		/// Abstract deallocament (implemented by Data<MHD_TYPE>)
		virtual void destroy() = 0;

		/// Set dimensions
		void setDimensions(int x, int y, int z, int c=1)
		{
			destroy();
			m_dims[0]=x;
			m_dims[1]=y;
			m_dims[2]=z;
			m_channels=c;
		}

		/// Set element spacing [mm]
		void setSpacing(double spx, double spy, double spz)
		{
			m_spacing[0]=spx;
			m_spacing[1]=spy;
			m_spacing[2]=spz;
		}

		/// Set the image contents directly. Ownership of data is transferred to this instance. See also copyFrom()
		virtual void setData(char* data) = 0;
		/// Copy raw data
		virtual void copyFrom(char * data) = 0;
		/// Access to raw data
		virtual char * raw() const = 0;
		/// Abstract data access (implemented by Data<MHD_TYPE>)
		virtual double get_direct(int x, int y, int z=0, int c=0) const = 0;
		/// Abstract data write access (implemented by Data<MHD_TYPE>)
		virtual void set_direct(double v, int x, int y, int z=0, int c=0) = 0;

		/// Abstract function to access MHD_TYPE as string
		virtual std::string getType() const = 0;
		/// Abstract function to access size of MHD_TYPE in bytes
		virtual int getTypeSize() const = 0;
		/// Access number of elements (=width*height*depth*channels)
		int getNumberOfElements() const;
		/// Function to access total size of raw data buffer in bytes
		int getBufferSize() const;
		
		/// Apply per-voxel mapping
		virtual void transform(double(*f)(double) ) = 0;

		/// Swap bytes
		virtual void convertEndian() = 0;

		/// Get with bounds checking in voxel space
		inline double get_safe(int x, int y, int z=0,int c=0) const
		{
			if ( x<0 || y<0 || z<0 || x>=m_dims[0] || y>=m_dims[1]
			|| z>=m_dims[2] || c<0 || c>=m_channels ) return 0;
			else return get_direct(x,y,z,c);
		}

		/// Set with bounds checking in voxel space,
		// Returns false for attemps to access data out of bounds
		inline bool set_safe(double v, int x, int y, int z=0, int c=0)
		{
			if ( x<0 || y<0 || z<0 || x>=m_dims[0] || y>=m_dims[1]
			|| z>=m_dims[2] || c<0 || c>=m_channels ) return false;
			else { set_direct(v,x,y,z,c); return true; }
		}

		/// Sampling with tri-linear interpolation in voxel space
		double sample_voxel(double x, double y, double z=0, int c=0) const;

		/// Sampling with tri-linear interpolation in millimeter space
		inline double sample_mm(double x, double y, double z=0, int c=0) const
		{
			return sample_voxel(x/m_spacing[0],y/m_spacing[1],z/m_spacing[2],c);
		}

		/// Analyze data, look for min max
		virtual void computeBounds(double& min, double& max) const = 0;

		/// Analyze data, look for min max mean and deviation
		virtual void computeBounds(double& min, double& max, double& mean, double& deviation) const = 0;

		/// Crop image. x,y,z will be mapped to origin, w,h,d are new dimensions.
		virtual void crop(int x, int y, int z, int w, int h, int d) = 0;

		// allow MHD::Image to access protected members
		friend class Image;
	};

	/// swap bytes
	inline void swapBytes(short& v)
	{
		char *d=(char*)(&v);
		std::swap(d[0],d[1]);
	}
	inline void swapBytes(int& v)
	{
		short *d=(short*)(&v);
		std::swap(d[0],d[1]);
		swapBytes(d[0]);
		swapBytes(d[1]);
	}
	inline void swapBytes(double& v)
	{
		int *d=(int*)(&v);
		std::swap(d[0],d[1]);
		swapBytes(d[0]);
		swapBytes(d[1]);
	}

	/// Type Specifications for all Meta Image Types (templated)
	template<typename MHD_TYPE> class Data : public DataInterface
	{
	protected:
		/// Pointer to raw data or null
		MHD_TYPE* m_data;

	public:
		Data();
		Data(const Data& other);
		virtual ~Data();

		/// Direct access to m_data through parenthesis, no bound checking
		inline MHD_TYPE& operator()(int x, int y, int z=0, int c=0)
		{
			return m_data[(z*m_dims[0]*m_dims[1]+y*m_dims[0]+x)*m_channels+c];
		}

		/// Direct access to m_data through parenthesis, no bound checking (const overload)
		inline const MHD_TYPE& operator()(int x, int y, int z=0, int c=0) const
		{
			return m_data[(z*m_dims[0]*m_dims[1]+y*m_dims[0]+x)*m_channels+c];
		}

		/// Allocate raw buffer (delete old one)
		virtual void alloc()
		{
			destroy();
			m_nDataElements=m_dims[0]*m_dims[1]*m_dims[2]*m_channels;
			m_data=new MHD_TYPE[m_nDataElements];
		}

		/// destroy raw buffer
		virtual void destroy()
		{
			m_nDataElements=0;
			if (m_data)
				delete m_data;
			m_data=0x0;
		}
		
		/// Set the image contents directly. Ownership of data is transferred to this instance. See also copyFrom()
		virtual void setData(char* data)
		{
			destroy();
			m_nDataElements=m_dims[0]*m_dims[1]*m_dims[2]*m_channels;
			m_data=(MHD_TYPE*)data;			
		}

		/// Copy raw data from a buffer of the same size
		virtual void copyFrom(char * data)
		{
			MHD_TYPE* d=(MHD_TYPE*)data;
			if (data)
				for (int i=m_nDataElements-1;i>=0;i--)
					m_data[i]=d[i];
		}

		/// Raw pointer
		virtual char * raw() const
		{
			return (char*)m_data;
		}

		/// Type specification for MHD_TYPE type string
		virtual std::string getType() const
		{
			return getTypeStr(m_data);
		}

		/// Type specification for MHD_TYPE type size in bytes
		virtual int getTypeSize() const
		{
			return sizeof(MHD_TYPE);
		}

		/// Transform (apply f() for each value in data)
		virtual void transform(double(*f)(double) )
		{
			for (int i=0;i<m_nDataElements;i++)
				m_data[i]=(MHD_TYPE)f((double)m_data[i]);
		}

		// Swap Bytes
		virtual void convertEndian()
		{
			switch(sizeof(MHD_TYPE))
			{
			default:
			case 1:
				std::cout << "Nothing to be done.\n";
			break;
			case 2:
				for (int i=0;i<m_nDataElements;i++)
					swapBytes(*(short*)(m_data+i));
				break;
			case 4:
				for (int i=0;i<m_nDataElements;i++)
					swapBytes(*(int*)(m_data+i));
				break;
			case 8:
				for (int i=0;i<m_nDataElements;i++)
					swapBytes(*(double*)(m_data+i));
				break;
			}
		}

		/// Read access without knowing the type. Possible loss of data, parenthesis operator preferred.
		virtual double get_direct(int x, int y, int z, int c) const
		{
			return (double)operator()(x,y,z);
		}

		//// Write access without knowing the type. Possible loss of data, parenthesis operator preferred.
		virtual void set_direct(double v, int x, int y, int z, int c)
		{
			operator()(x,y,z,c)=(MHD_TYPE)v;
		}

		//// Analyze data, look for min max
		virtual void computeBounds(double& min, double& max) const
		{
			min=m_data[0];max=m_data[0];
			for (int i=m_nDataElements-1;i>=0;i--)
			{
				if (m_data[i]<min) min=m_data[i];
				if (m_data[i]>max) max=m_data[i];
			}
		}

		//// Analyze data, look for min max mean deviation
		virtual void computeBounds(double& min, double& max, double& mean, double& deviation) const
		{
			double sum=0;
			double sqsum=0;
			min=m_data[0];max=m_data[0];
			for (int i=m_nDataElements-1;i>=0;i--)
			{
				if (m_data[i]<min) min=m_data[i];
				if (m_data[i]>max) max=m_data[i];
				sum+=m_data[i];
				sqsum+=m_data[i]*m_data[i];
			}
			mean=sum/=m_nDataElements;
			sqsum/=m_nDataElements;
			deviation=std::sqrt(sqsum-mean*mean);
		}

		/// Crop image. x,y,z will be mapped to origin, w,h,d are new dimensions.
		virtual void crop(int x, int y, int z, int w, int h, int d)
		{
			MHD_TYPE * n=cropBuffer(m_data,m_dims[0],m_dims[1],m_dims[2],x,x+w,y,y+h,z,z+d,m_channels);
			setDimensions(w,h,d,m_channels);
			setData((char*)n);
		}

	};

	/// Container of data and MHD file parser
	class Image {
	protected:
		/// Type general data
		DataInterface *m_data;
		/// Mapping of tags to values contained in mhd header file
		mutable std::map<std::string, std::string> m_mhd_tags;

	public:
		Image();
		Image(const Image& other);
		~Image();

		/// Parse an MHD header file and attempt to load raw data
		virtual bool loadMHD(const std::string& path);
		/// Try to load a windows BMP file. 8,24 and 32 bit Images only.
		virtual bool loadBMP(const std::string& path);

		/// Check if a certain mhd tag is present
		bool hasTag(const std::string& tag) const;
		/// Get a certain mhd tag. Returns false if not present
		bool getTag(const std::string& tag, std::string& value) const;
		/// Set a certain mhd tag other than the ones of updateTags() and ElementDataFile.
		void setTag(const std::string& tag, const std::string& value);
		/// Update values of NDims, DimSize, ElementSpacing, ElementByteOrderMSB, ElementType
		void updateTags() const;

		/// Write data to disk in MHD format, ".raw" and ".mhd" will be added to path
		virtual bool saveMHD(const std::string& path) const;
		/// Write a windows BMP file. 1,3 and 4 channel 2D unsigned char images only.
		virtual bool saveBMP(const std::string& path) const;

		/// Create an image of given type and dimensions (and optionally data)
		virtual bool create(std::string type_str, int x, int y, int z=1, int c=1, double spx=1, double spy=1, double spz=1, char* data=0x0);
		/// Destroy image and release data.
		virtual void destroy();

		// Access to data
		inline void setDataInterface(MHD::DataInterface* data)
		{
			destroy();
			m_data=data;
		}
		// Access to data
		inline DataInterface* getData() {return m_data;}
		/// Access to data
		inline const DataInterface* getData() const {return m_data;}
		/// Convert data type
		bool convert(const std::string& new_type, double bias=0, double scale=0)
		{
			DataInterface *old_data=m_data;
			m_data=0x0;
			create(new_type,old_data->dim(0),old_data->dim(1),old_data->dim(2),old_data->channels(),
					old_data->spacing(0),old_data->spacing(1),old_data->spacing(2));
			if (m_data==0x0)
			{
				m_data=old_data;
				return false;
			}
			MHD::convertBuffer(old_data->raw(),m_data->raw(),old_data->getNumberOfElements(),old_data->getType(),new_type,bias,scale);
			return true;
		}



	};

} // namespace MHD

#endif // SIMPLE_HMD
