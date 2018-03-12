// Created by A. Aichert in Aug 2013

#ifndef __NRRD_IMAGE_STACK
#define __NRRD_IMAGE_STACK

#include "nrrd_image.hxx"

namespace NRRD
{
	/// Access 2D Slices out of a 3D volume. Simple image container with loading functoinality.
	template <typename T>
	class ImageStack : public ImageView<T>
	{
	protected:
		/// Image size. Length determines the number of dimensions.
		/// See also ::size(...) and ::dimension()
		std::vector<int> dim;
		/// Voxel spacing. Defines scaling to mm
		/// See also ::size(...) and ::dimension()
		std::vector<double> element_spacing;
		/// File on hard drive
		std::fstream file;
		/// Byte offset to raw data in file
		int raw_offset;

		ImageStack& operator=(const ImageStack&);
	public:
		/// Instantiate an image stack. Test with operator!() for failure.
		ImageStack(const std::string& path="")
		{
			// Get the header
			if (path.empty()) return;
			raw_offset=parseHeader(path,nrrd_header,&meta_info);
			if (raw_offset<=0) {
				std::cerr << "NRRD::ImageStack<T>("<<__LINE__<<"): File access error.\n" << path << std::endl;
				return;
			}

			// Make sure we have the right type
			if (nrrd_header["type"]!=typeName<T>())
			{
				std::cerr << "NRRD::ImageStack<" << typeName<T>()<< ">" << "Type mismatch " << nrrd_header["type"] << std::endl << path << std::endl;
				nrrd_header.clear();
				meta_info.clear();
				return;
			}

			// Open file for access to image data
			file.open(path.c_str()
				, std::ios::in
				| std::ios::out
				| std::ios::app
				| std::ios::binary);
			if (!file) {
				std::cerr << "NRRD::ImageStack<T>("<<__LINE__<<"): File access error (2).\n" << path << std::endl;
				return;
			}

			// Make sure the data is uncompressed
			if (nrrd_header["encoding"]!="raw")
			{
				std::cerr << "NRRD::ImageStack<T>("<<__LINE__<<"): File encoding not supported\n";
				file.close();
				nrrd_header.clear();
				meta_info.clear();
			}

			// Make sure the endian matches
			if (nrrd_header["endian"]=="") nrrd_header["endian"]="little";
			bool endian_file=nrrd_header["endian"]!="little";
			bool endian_machine=is_cpu_BIG_endian();
			if (endian_machine != endian_file)
			{
				std::cerr << "NRRD::ImageStack<T> (warning): Loading a "
					<< (endian_file?"Big":"little") << " endian image stack on a "
					<< (endian_machine?"Big":"little") << " endain machine is currently "
					"not supported. Try NRRD::load<T> and NRRD::save<T> for access to "
					"the whole data chunk at once and conversion to machine endian.\n";
				file.close();
				nrrd_header.clear();
				meta_info.clear();
				return;			
			}

			// Make sure the file type matches 
			if (nrrd_header["type"]!=typeName<T>())
			{
				std::cerr << "NRRD::ImageStack<T>("<<__LINE__<<") with T="<<typeName<T>() << ": Cannot load type "<< nrrd_header["type"]<<"\n";
				file.close();
				nrrd_header.clear();
				meta_info.clear();
				return;
			}

			// Get number of dimensions and size
			int n=stringTo<int>(nrrd_header["dimension"]);
			dim=stringToVector<int>(nrrd_header["sizes"]);
			if (n<=0 || n>10 || (int)dim.size()!=n)
			{
				std::cerr << "NDDR::load<T>(...): Bad dimension/size.\n";
				file.close();
				dim.clear();
				nrrd_header.clear();
				meta_info.clear();
				return;
			}

			// Make sure the data is complete
			file.seekg(0,std::ios::end);
			int file_size=(int)file.tellg();
			int expected_size=length()*sizeof(T)+raw_offset;
			if (expected_size>file_size)
			{
				std::cerr << "NRRD::ImageStack<T>("<<__LINE__<<"): Filesize does not match image dimensions.\n";
				file.close();
				nrrd_header.clear();
				meta_info.clear();
				return;
			}
		}

		/// Load a single slice from disk and return it in out
		bool readImage(int i, NRRD::Image<T>& out) {
			// Check if i is in bounds
			if (i<0 || dim.empty() || i>dim.back())
				return false;
			// Re-size output image only if needed
			bool realloc=(out.dimension()+1!=dim.size());
			if(!realloc)
				for (int j=0; j<(int)dim.size()-1; j++)
					if (dim[j]!=out.size(j)) realloc=true;
			if (realloc)
				out.set(&(dim[0]),(int)dim.size()-1);
			if (!out) return false;
			// Read slice
			file.seekp(raw_offset+out.length()*i*sizeof(T), std::ios::beg);
			file.read((char*)((T*)out),sizeof(T)*out.length());
			return file.good();
		}
		
		/// Write image data from img to the i-th slice of the file.
        bool writeImage(int i, const NRRD::ImageView<T>& img) { // FIXME
        //	// Check if i is in bounds
        //	if (i<0 || i>dim.back())
        //		return false;
        //	// Check if image bounds match
        //	bool sizeok=false;
        //    for (int j=0, j<(int)dim.size()-1;j++)
        //        if (dim[j]!=out.size(j)) sizeok=true;
        //	if (!sizeok) return false;
        //	// Write slice
        //	file.seekg(raw_offset+img.length()*i, std::ios::beg);
        //	file.write((char*)((T*)img),sizeof(T)*img.length());
        //	return file.good();
		}
		
		/// Check if the file is open for r/w
		bool good() const { return file.is_open() && file.good() && dimension()>2; }

		/// Total number of pixels/voxels
		int length() const {
			if (dim.empty()) return 0;
			for (int len=1, i=0;;i++)
				if (i==(int)dim.size()) return len;
				else len*=size(i);
		}

		/// Returns the number of dimensions of the Image.
		int dimension() const { return (int)dim.size(); }

		/// Return image size in i-th dimension. See also int NRRD::Image<T>::dimension();
		int size(int i) const { return dim[i]; }

		/// Like a vector of images.
		int size() const {return dim.empty()?0:dim.back();}

		/// Return image size in i-th dimension. See also int NRRD::Image<T>::dimension();
		double spacing(int i) const { return element_spacing[i]; }
		
		/// Dictionary of meta-information. value by tag read-only.
		std::map<std::string,std::string> meta_info;

		/// Dictionary of nrrd-fields read-only
		std::map<std::string,std::string> nrrd_header;
		
	};

} // namespace NRRD

#endif // __NRRD_IMAGE_STACK
