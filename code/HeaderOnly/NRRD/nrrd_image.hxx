// Created by A. Aichert in Aug 2013

#ifndef __NRRD_IMAGE_HXX
#define __NRRD_IMAGE_HXX

#include "nrrd_image_view.hxx"

namespace NRRD
{
	/// Simple image container with loading functoinality.
	template <typename T>
	class Image : public ImageView<T>
	{
	public:
        using ImageView<T>::nrrd_header;
        using ImageView<T>::meta_info;
	private:
        using ImageView<T>::data;
        using ImageView<T>::dim;
        using ImageView<T>::element_spacing;
	private:
        /// Not copy constructible
		Image(const Image&);
		/// Not copyable
		Image& operator=(const Image&);
	public:
		/// An invalid image
        Image() : ImageView<T>() {}

		/// An image from a .nrrd-file
        Image(const std::string& nrrd_file) : ImageView<T>(), alloc(0) { load(nrrd_file); }
		
		/// A 3D image of specific size. Optionally wraps an existing data pointer.
        Image(int w, int h, int d=1, T* dt=0x0) : ImageView<T>(), alloc(0) { set(w,h,d, dt); }

		/// A n-D image of specific size. Optionally wraps an existing data pointer.
        Image(int size[], int n, T* dt=0x0) : ImageView<T>(), alloc(0) { set(size,n,dt); }

		/// Clone an existing image.
		template <typename T2>
		Image<T>& clone(const NRRD::ImageView<T2>& other)
		{
			int n=other.dimension();
			std::vector<int> dims(n);
			for (int i=0; i<n; i++) dims[i]=other.size(i);
			if (n>0) {
				set(&dims[0],n,0x0);
				this->copyDataFrom((T2*)other);
			}
			else set(0,0x0);
			nrrd_header=other.nrrd_header;
			meta_info=other.meta_info;
			return *this;
		}
		
		/// Create an image from file
		bool load(const std::string& nrrd_file)
		{
			if (alloc&&data!=0x0)
				delete [] data;
			data=0x0;
			dim.clear();
			if (!NRRD::load<T>(nrrd_file,&data,&dim,&meta_info,&nrrd_header))
				return false;
			auto it=nrrd_header.find("spacings");
			if (it!=nrrd_header.end())
				element_spacing=stringToVector<double>(it->second,' ');
			else if ((it=nrrd_header.find("space directions"))!=nrrd_header.end())
			{
				std::replace(it->second.begin(),it->second.end(),'(',' ');
				std::replace(it->second.begin(),it->second.end(),')',' ');
				std::replace(it->second.begin(),it->second.end(),',',' ');
				auto vec=stringToVector<double>(it->second,' ');
                element_spacing.resize(dim.size());
				if (!vec.empty())
					element_spacing[0]=vec[0];
				if (vec.size()>=4)
					element_spacing[1]=vec[4];
				if (vec.size()>=8)
					element_spacing[2]=vec[8];
				nrrd_header.erase(it);
				nrrd_header["spacings"]=vectorToString<double>(element_spacing," ");
			}
			while (element_spacing.size()<dim.size()) element_spacing.push_back(1.0);
			return true;
		}
		
		/// Allocate or reference a 2D or 3D raw image.
		virtual void set(int w, int h, int d=1, T* dt=0x0)
		{
			int size[]={w,h,d};
			int n=d<2?2:3;
			set(size,n,dt);
		}

		/// Allocate or reference n-D image.
		virtual void set(const int size[], int n, T* dt=0x0)
		{
			// Store old state (in case we can re-use the data buffer)
			T*  old_alloc_data=passOwnership();
			int old_alloc_length=old_alloc_data?ImageView<T>::length():0;
			// Set new state
			ImageView<T>::set(size,n,dt);
			// If no data is given and data size has not changed
			if (!dt && old_alloc_length==ImageView<T>::length())
				// Re-use old data buffer
				takeOwnership(old_alloc_data);
			else
			{
				// We no longer need the old data buffer
				if (old_alloc_data)
					delete [] old_alloc_data;
				// If no data is given allocate new data buffer
				if (!dt)
					takeOwnership(new T[ImageView<T>::length()]);
			}
		}

		/// Prevents this class from calling delete [] on the data pointer.
		/// You HAVE TO delete the pionter returned yourself AFTER this object goes out of scope.
		/// Returns null if the image no longer owns its data.
		T* passOwnership()
		{
			if (!alloc) return 0x0;
			alloc=false;
			return data;
		}

		/// Set the data pointer to new_data and deletes that pointer on destruction.
		/// Caller MAY NOT call delete on new_data EVER.
		void takeOwnership(T* new_data)
		{
			if (alloc&&data!=0x0&&data!=new_data)
				delete [] data;
			data=new_data;
			alloc=true;
		}

		/// Destructor. Deletes data pointer if the data has been allocated during construction.
		/// See also bool NRRD::Image<T>::passOwnership();
		~Image()
		{
			if (alloc&&data!=0x0)
				delete [] data;
		}

	private:
		/// True if destructor will call delete [].
		/// See also ::passOwnership()
		bool alloc;
	};

} // namespace NRRD

#endif // __NRRD_IMAGE_HXX
