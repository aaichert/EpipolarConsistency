#include "SimpleMHD.h"

#include <fstream>
#include <iostream>

namespace  MHD
{

	template <typename T>
	std::string toString(T in)
	{
		std::ostringstream strstr;
		strstr << in;
		return strstr.str();
	}

	template <typename T>
	T stringTo(const std::string& in)
	{
		T value;
		std::istringstream strstr(in);
		strstr >> value;
		return value;
	}

	inline bool is_cpu_BIG_endian()
	{
	   unsigned int u=255;
	   unsigned char *c=(unsigned char*)&u;
	   return 0!=(*(c+3));
	}

	std::string splitPathAndName(std::string& path)
	{
		std::string::size_type loc=path.find_last_of ("/\\");
		std::string name;
		if (loc!=std::string::npos)
		{
			name=path.substr(loc+1,std::string::npos);
			path=path.substr(0,loc+1);
		}
		else
		{
			name=path;
			path="";
		}
		return name;
	}

	DataInterface::DataInterface()
		: m_channels(0)
		, m_localEndian(1)
	{
		m_dims[0]=m_dims[1]=m_dims[2]=0;
		m_spacing[0]=m_spacing[1]=m_spacing[2]=1;
	}
	DataInterface::~DataInterface() {}

	DataInterface * DataInterface::createForType(const std::string& type)
	{
		if (type==getTypeStr<char>())			return new Data<char>;
		if (type==getTypeStr<unsigned char>())	return new Data<unsigned char>;
		if (type==getTypeStr<short>())			return new Data<short>;
		if (type==getTypeStr<unsigned short>())	return new Data<unsigned short>;
		if (type==getTypeStr<int>())			return new Data<int>;
		if (type==getTypeStr<unsigned int>())	return new Data<unsigned int>;
		if (type==getTypeStr<float>())			return new Data<float>;
		if (type==getTypeStr<double>())			return new Data<double>;
		return 0x0;
	}

	int DataInterface::getNumberOfElements() const
	{
		return m_nDataElements;
	}

	int DataInterface::getBufferSize() const
	{
		return getTypeSize()*m_nDataElements;
	}

	double DataInterface::sample_voxel(double x, double y, double z, int c) const
	{
		int ix=(int)std::floor(x);
		int iy=(int)std::floor(y);
		int iz=(int)std::floor(z);
		x-=ix;
		y-=iy;
		z-=iz;
		// no interpolation needed
		if (x==0&&y==0&&z==0)
			return get_safe(ix,iy,iz);
		// xy-bilinear only
		if (z==0)
			return	(1.0-y)*((1.0-x)*get_safe(ix,iy,iz,c) +x*get_safe(ix+1,iy,iz,c))
					+y*((1.0-x)*get_safe(ix,iy+1,iz,c) +x*get_safe(ix+1,iy+1,iz,c));
		// trilinear
		return	(1.0-z)*(((1-y)*((1.0-x)*get_safe(ix,iy,iz,c) +x*get_safe(ix+1,iy,iz,c)))
				+(y*((1.0-x)*get_safe(ix,iy+1,iz,c) +x*get_safe(ix+1,iy+1,iz,c))))
				+z*((1-y)*((1.0-x)*get_safe(ix,iy,iz,c) +x*get_safe(ix+1,iy,iz,c))
				+(y*((1.0-x)*get_safe(ix,iy+1,iz,c) +x*get_safe(ix+1,iy+1,iz,c))));
	}


	template <typename MHD_TYPE> Data<MHD_TYPE>::Data() : DataInterface() , m_data(0x0) {}

	template <typename MHD_TYPE> Data<MHD_TYPE>::Data(const Data& other)
	{
		alloc(other.m_dims[0],other.m_dims[1],other.m_dims[2], other.m_channels);
		m_spacing[0]=other.m_spacing[0];
		m_spacing[1]=other.m_spacing[1];
		m_spacing[2]=other.m_spacing[2];
		m_data->copyFrom(other.raw());
	}

	template <typename MHD_TYPE> Data<MHD_TYPE>::~Data()
	{
		if (m_data)
			delete [] m_data;
		m_data=0x0;
	}

	Image::Image() : m_data(0x0) {}
	Image::Image(const Image& other)
	{
		if (!create(other.m_data->getType(),
			other.m_data->m_dims[0], other.m_data->m_dims[1], other.m_data->m_dims[2], other.m_data->m_channels,
			other.m_data->m_spacing[0], other.m_data->m_spacing[1], other.m_data->m_spacing[2], other.m_data->raw() ))
			/*error*/;
		else
			m_mhd_tags=other.m_mhd_tags;
	}
	Image::~Image()
	{
	//	if (m_data)
	//		delete m_data;
	//	m_data=0x0;
	}


	bool Image::loadMHD(const std::string& path)
	{
		bool err=false;
		// clean up first
		if (m_data)
		{
			delete m_data;
			m_data=0x0;
		}
		m_mhd_tags.clear();
		// load MHD header file
		std::ifstream file(path.c_str());
		if (!file.is_open() && file.good())
			return false;
		// read whole file
		std::string all;
		std::getline(file,all,'\0');
		// store MHD header file line by line to m_mhd_tags
		int l=0, ndim=0;
		std::istringstream strstr(all);
		while (!strstr.eof())
		{
			// the tag is one word in the beginning of the line
			std::string line;
			std::getline(strstr,line);
			std::stringstream lss(line);
			std::string tag,value;
			lss >> tag;
			lss >> value;
			// then we expect a '='
			if (value=="=")
			{
				// then comes the value
				std::getline(lss,value,'\0');
				// ltrim
				std::string::size_type n=line.find_first_not_of(' ');
				if (n!=std::string::npos)
					value=value.substr(n+1,value.length()-n-1);
				m_mhd_tags[tag]=value;
			}
			// else parse error, should print a warning...
			l++;
		}
		// at this point we should have the whole MHD header file loaded to m_mhd_tags

		// try to create an instance for the right type
		m_data=DataInterface::createForType(m_mhd_tags["ElementType"]);
		if (m_data!=0x0)
		{
			{
				std::istringstream strstr(m_mhd_tags["DimSize"]);
				strstr >> m_data->m_dims[0] >> m_data->m_dims[1] >> m_data->m_dims[2];
				if (stringTo<int>(m_mhd_tags["NDims"])<3)
					m_data->m_dims[2]=1;
			}
			{
				std::istringstream strstr(m_mhd_tags["ElementSpacing"]);
				strstr >> m_data->m_spacing[0] >> m_data->m_spacing[1] >> m_data->m_spacing[2];
				if (stringTo<int>(m_mhd_tags["NDims"])<3)
					m_data->m_spacing[2]=1;
				if (m_data->m_spacing[0]*m_data->m_spacing[1]*m_data->m_spacing[2]<0)
					err=true;
			}
			/// FIXME m_channels ElementDataCHennles or whatever its called
			if (hasTag("ElementByteOrderMSB"))
			{
				bool isDataMSB=m_mhd_tags["ElementByteOrderMSB"]=="True"
							|| m_mhd_tags["ElementByteOrderMSB"]=="true"
							|| m_mhd_tags["ElementByteOrderMSB"]=="TRUE";
				if (is_cpu_BIG_endian()!=isDataMSB)
					m_data->m_localEndian=false;
				else
					m_data->m_localEndian=true;
			}
			else
					m_data->m_localEndian=true;
		}
		else
			err=true;

		if (!m_data)
			err=true;
		else
		{
			if (m_data->m_channels<1)
				m_data->m_channels=1;
			m_data->m_nDataElements=m_data->m_dims[0]*m_data->m_dims[1]*m_data->m_dims[2]*m_data->m_channels;
			if (m_data->m_nDataElements<1)
				err=true;
		}

		if (!err && hasTag("ElementDataFile"))
		{
			std::string rawpath=path;
			splitPathAndName(rawpath);
			rawpath+=m_mhd_tags["ElementDataFile"];
			// 2do Maybe print stats here 
			std::ifstream rawfile(rawpath.c_str(), std::ios::in|std::ios::binary|std::ios::ate);
			if (rawfile.is_open())
			{
				int headerSize=0;
				if (hasTag("HeaderSize"))
				{
					headerSize=stringTo<int>(m_mhd_tags["HeaderSize"]);
					if (headerSize<0) headerSize=0;
				}
				// check file size
				int size=(int)rawfile.tellg();
				rawfile.seekg(headerSize,std::ios::beg);
				m_data->alloc();
				int expectedSize=m_data->getBufferSize();
				if (size!=expectedSize+headerSize)
					err=1;
				else
					rawfile.read(m_data->raw(),m_data->getBufferSize());
			}
			else
				err=true;
		}

		// if we failed to load data, we just clean up this instance
		if (err)
		{
			if (m_data)
				delete m_data;
			m_data=0x0;
			m_mhd_tags.clear();
		}

		if (m_data && !m_data->m_localEndian)
		{
			// convert endian
			std::cout << "Converting endianess. \n";
			m_data->convertEndian();
		}


		return !err;
	}

	/// Combined BMP file header and BMP info header. To work around forced memory alignment, use all unsigned short.
	union BMPHeader{
		struct BMPInfo {
			unsigned char	B;
			unsigned char	M;
			unsigned short	unused;			unsigned short	unused_hi;
			unsigned short  creator;		unsigned short  creator_hi;
			unsigned short	bmp_offset;		unsigned short	bmp_offset_hi;	
			unsigned short	header_sz;		unsigned short	header_sz_hi;
			unsigned short	width;			unsigned short	width_hi;
			unsigned short	height;			unsigned short	height_hi;
			unsigned short	nplanes;
			unsigned short	bitspp;
			unsigned short	compress_type;	unsigned short	compress_type_hi;
			unsigned short	bmp_bytesz;		unsigned short	bmp_bytesz_hi;
			unsigned short	hres;			unsigned short	hres_hi;
			unsigned short	vres;			unsigned short	vres_hi;
			unsigned short	ncolors;		unsigned short	ncolors_hi;
			unsigned short	nimpcolors;		unsigned short	nimpcolors_hi;
		} info;
		unsigned short raw[27];
		// 2do convert endian
		// for (int i=1;i<13;i+=2) endian(raw[i]);
		// for (int i=7;i<27;i+=2) endian(raw[i]);
	};

	void swapRG(unsigned char * data, int n, int c)
	{
		if (c>2)
			for (int i=0;i<n;i+=c)
				std::swap(data[i],data[i+2]);
	}

	bool Image::loadBMP(const std::string& path)
	{
		if (is_cpu_BIG_endian())
		{
			std::cerr << "Loading BMP Files is not supported on BIG endian systems.\n";
			return false;
		}

		BMPHeader header;
		int sizeh=sizeof(header);

		std::ifstream file(path, std::ios::in | std::ios::binary | std::ios::ate);
		if (file.is_open())
		{
			int filesize=(int)file.tellg();
			file.seekg(0, std::ios::beg);
			file.read((char*)&header, sizeof(BMPHeader));
			int ipad=(header.info.width*header.info.bitspp/8)%4;
			if (ipad!=0) ipad=4-ipad;
			int expectedFileSize=header.info.bmp_offset+(header.info.height*(header.info.width+ipad))*header.info.bitspp/8;
			if (	(header.info.bitspp!=8 && header.info.bitspp!=24 && header.info.bitspp!=32) // unsigned char 256 gray, rgb or rgba
			//	||  header.info.compress_type!=0 // no compression
				||  filesize<expectedFileSize // enough data
				||	header.info.B!='B' || header.info.M!='M' ) // windows dib
			{
				std::cerr << "Unsupported BMP image or file access error.\n";
				return false;
			}

			if (header.info.ncolors)
				std::cerr << "BMP color table ignored.\n";

			destroy();
			m_data=new Data<unsigned char>();
			m_data->setDimensions(header.info.width,header.info.height,1,header.info.bitspp/8);
			m_data->setSpacing(1,1,1);
			m_data->alloc();
			// load lines reversing (bmps are stored bottom-up)
			file.seekg(header.info.bmp_offset, std::ios::beg);
			for (int i=0;i<header.info.height;i++)
			{
				file.read(m_data->raw()+i*(header.info.width*header.info.bitspp/8+ipad),header.info.width*header.info.bitspp/8);
				if (ipad) file.seekg(ipad, std::ios::cur);
			}
			swapRG((unsigned char*)m_data->raw(),m_data->getNumberOfElements(),m_data->channels());
			file.close();
		}
		else
		{
			std::cerr << "Failed to load BMP. File not found.\n";
			return false;
		}
		return true;
	}

	bool Image::saveBMP(const std::string& path) const
	{
		if (!m_data)
		{
			std::cerr << "Failed to save BMP. No Image data.\n";
			return false;
		}

		if (m_data->m_channels!=1 && m_data->m_channels!=3 && m_data->m_channels!=4)
		{
			std::cerr << "Failed to save BMP. Only intensity, rgb and rgba images supported.\n";
			return false;
		}

		if (m_data->getType()!=MHD::getTypeStr<unsigned char>())
		{
			std::cerr << "Please convert image to unsigned char before saving a BMP image.\n";
			return false;			
		}

		if (is_cpu_BIG_endian())
		{
			std::cerr << "Saving BMP Files is not supported on BIG endian systems.\n";
			return false;
		}

		// prepare header
		BMPHeader header;
		for (int i=0;i<27;i++)
		header.raw[i]=0;
		header.info.B='B';
		header.info.M='M';
		header.info.bmp_offset=54;
		header.info.header_sz=40;
		header.info.width=m_data->dim(0);
		header.info.height=m_data->dim(1);
		header.info.nplanes=1;
		header.info.bitspp=m_data->channels()*8;
		int ipad=(header.info.width*header.info.bitspp/8)%4;
		if (ipad!=0) ipad=4-ipad;
		int bmp_bytesz=(header.info.width*header.info.bitspp/8+ipad)*header.info.height;
		header.info.bmp_bytesz=*((unsigned short*)&bmp_bytesz);
		header.info.bmp_bytesz_hi=*(((unsigned short*)&bmp_bytesz)+1);
		header.info.hres=11811;
		header.info.vres=11811;
		if (m_data->channels()==1)
		{
			header.info.bmp_offset+=4*256;
			header.info.ncolors=256;
		}
		// write file
		int zero=0;
		std::ofstream file(path, std::ios::binary);
		if (file.is_open())
		{
			// swap in a const function is a little dirty, although it happens twice.
			swapRG((unsigned char*)m_data->raw(),m_data->getNumberOfElements(),m_data->channels());
			file.write((char*)&header,sizeof(BMPHeader));
			if (m_data->channels()==1)
			{
				unsigned char lut[256*4];
				for (int i=0;i<256;i++)
					for (int c=0; c<4;c++)
						lut[i*4+c]=i;
				file.write((char*)lut,4*256);
			}
			for (int i=0;i<header.info.height;i++)
			{
				file.write(m_data->raw()+i*(header.info.width*header.info.bitspp/8+ipad),header.info.width*header.info.bitspp/8);
				if (ipad) file.write((char*)&zero,ipad);
			}
			swapRG((unsigned char*)m_data->raw(),m_data->getNumberOfElements(),m_data->channels());
			file.close();

		}
		return true;
	}

	void Image::updateTags() const
	{
		if (m_data->m_dims[2]<=1)
			m_mhd_tags["NDims"]="2";
		else
			m_mhd_tags["NDims"]="3";
		m_mhd_tags["DimSize"] = toString(m_data->m_dims[0]) + " "
				+ toString(m_data->m_dims[1]) + " " + toString(m_data->m_dims[2]);
		m_mhd_tags["ElementSpacing"] = toString(m_data->m_spacing[0]) + " "
				+ toString(m_data->m_spacing[1]) + " " + toString(m_data->m_spacing[2]);
		if (m_data->m_channels!=1)
			m_mhd_tags["ElementNumberOfChannels"] = toString(m_data->m_channels);
		if ((is_cpu_BIG_endian() && m_data->m_localEndian)
		|| (!is_cpu_BIG_endian() && !m_data->m_localEndian))
			m_mhd_tags["ElementByteOrderMSB"]="True";
		else
			m_mhd_tags["ElementByteOrderMSB"]="False";
		m_mhd_tags["ElementType"]=m_data->getType();
		// make sure we do not have the ElementDataFile tag in the m_mhd_tags, since that can only be set when saved
		std::map<std::string, std::string>::iterator it=m_mhd_tags.find("ElementDataFile");
		if (it!=m_mhd_tags.end())
			m_mhd_tags.erase(it);
	}

	bool Image::hasTag(const std::string& tag) const
	{
		std::map<std::string, std::string>::const_iterator it=m_mhd_tags.find(tag);
		return it!=m_mhd_tags.end();
	}

	bool Image::saveMHD(const std::string& path) const
	{
		updateTags();

		std::string folder=path;
		// split path and filename
		std::string name=splitPathAndName(folder);

		// create two files (mhd and raw) with the same name
		std::string mhdname(name+".mhd");
		std::string rawname(name+".raw");

		// open these files for access
		std::ofstream mhdfile((folder+mhdname).c_str());
		std::ofstream rawfile((folder+rawname).c_str(),std::ios_base::out | std::ios_base::binary);
		if (!mhdfile.is_open() || !rawfile.is_open())
		{
			std::cerr << "Failed to save MHD. File access error.\n";
			return false;
		}

		// write the volume data file
		int filesize=m_data->getBufferSize();
		rawfile.write(m_data->raw(), filesize );
		rawfile.close();

		// write MHD header file
		mhdfile << "ElementDataFile = " << (name+".raw") << std::endl;
		for (std::map<std::string, std::string>::const_iterator it=m_mhd_tags.begin();it!=m_mhd_tags.end();++it)
			mhdfile << it->first << " = " << it->second << std::endl;
		mhdfile.close();
		return true;
	}

	bool Image::create(std::string type_str, int x, int y, int z, int c, double spx, double spy, double spz, char * data)
	{
		m_data=DataInterface::createForType(type_str);
		if (m_data==0x0) return false;
		m_data->setDimensions(x,y,z,c);
		m_data->setSpacing(spx,spy,spz);
		m_data->alloc();
		m_data->copyFrom(data);
		return true;
	}

	void Image::destroy()
	{
		if (m_data)
			delete m_data;
		m_data=0x0;
		m_mhd_tags.clear();
	}

} // namespace MHD
