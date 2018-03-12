// Created by A. Aichert on Wed Sept 3rd 2014

#ifndef __NRRD_IMAGE_VIEW_HXX
#define __NRRD_IMAGE_VIEW_HXX

#include "nrrd.hxx"

namespace NRRD
{

	/// NRRD::ImageView is a view of existing memory in the form of a NRRD image with meta_info and tri-linear sampling.
	/// No memory is allocated, hence an ImageView is always dependent on external data.
	template <typename T>
	class ImageView
	{
	protected:
		/// Image size. Length determines the number of dimensions.
		/// See also ::size(...) and ::dimension()
		std::vector<int> dim;
		/// Voxel spacing. Defines scaling to mm
		/// See also ::size(...) and ::dimension()
		std::vector<double> element_spacing;
		/// Raw data pointer
		/// See also implicit cast operator and ::raw()
		T* data;

	public:
		/// An Invalid ImageView
		ImageView() : data(0x0) {}

		/// A  3D (or 2D) image view.
		ImageView(int w, int h, int d, T* dt) { set(w,h,d,dt); }

		/// An n-D image view of specific size.
		ImageView(int size[], int n, T* dt) { set(size,n,dt); }

		/// A 2D slice of a 3D volume.
		ImageView(const ImageView<T>& volume, int zindex)
		{
			set(volume.size(0), volume.size(1), 1, (T*)volume + zindex*volume.size(0)*volume.size(1));
			spacing(0)=volume.spacing(0);
			spacing(1)=volume.spacing(1);
		}

		/// Re-interpret fast dimension. Copies meta info but ignores nrrd_header.
		template <class complex_type>
		ImageView<complex_type> cast_interpret_fast() {
			ImageView<complex_type> ret;
			if (dim[0]*sizeof(T)!=sizeof(complex_type))
				return ret;
			ret.set(dim.data()+1,(int)dim.size()-1,(complex_type*)data);
			for (int i=1;i<(int)element_spacing.size();i++)
				ret.spacing(i-1)=element_spacing[i];
			ret.meta_info=meta_info;
			return ret;
		}

		/// Cast from a vector to sequence of its elements. Adds a fast dimension if sizeof(T)>sizeof(basic_type). Copies meta info but ignores nrrd_header. See also: reinterpret_fast.
		template <typename basic_type>
		ImageView<basic_type> cast_to_fast() {
			if (sizeof(T)%sizeof(basic_type)!=0)
				return ImageView<basic_type>(); /// Cannot represent T as sequence of basic_type
			// Add a new fast dimension if required
			std::vector<int> new_dim;
			std::vector<double> new_es;
			if (sizeof(T)>sizeof(basic_type)) {
				new_dim.push_back(sizeof(T)/sizeof(basic_type));
				new_es.push_back(1); // unknown...
			}
			// Then comes the old info
			for (auto it=dim.begin();it!=dim.end();++it) new_dim.push_back(*it);
			for (auto it=element_spacing.begin();it!=element_spacing.end();++it) new_es.push_back(*it);
			NRRD::ImageView<basic_type> ret(new_dim.data(),(int)new_dim.size(),(basic_type*)data);
			ret.meta_info=meta_info;
			for (int i=0;i<(int)new_es.size();i++)
				ret.spacing(i)=new_es[i];
			return ret;
		}

		/// Reference a 2D or 3D raw image.
		virtual void set(int w, int h, int d=1, T* dt=0x0)
		{
			int size[]={w,h,d};
			int n=d<2?2:3;
			set(size,n,dt);
		}

		/// Reference an n-D raw image.
		virtual void set(const int size[], int n, T* dt)
		{
			if (n>0) dim.resize(n);
			else dim.clear();
			for (int i=0;i<n;i++)
				dim[i]=size[i];
			element_spacing.clear();
			while (element_spacing.size()<dim.size()) element_spacing.push_back(1.0);
			data=(length()<1)?0x0:dt;
		}

		/// Unary not, to check for null pointers
		bool operator!() const { return data==0x0; }

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
		int size(int i) const { return i<dim.size()?dim[i]:1; }

		/// Return image size in i-th dimension. See also int NRRD::Image<T>::dimension();
		const double& spacing(int i) const { return element_spacing[i]; }

		/// Return image size in i-th dimension. See also int NRRD::Image<T>::dimension();
		double& spacing(int i) { return element_spacing[i]; }

		/// Implicit cast to data pointer
		operator T*() const { return data; }

		/// Implicit cast to void data pointer
		operator void*() const { return (void*)data; }

		/// Direct access to data array
		inline T& operator[](int i) { return data[i]; }

		/// Direct access to data array (const overload)
		inline const T& operator[](int i) const { return data[i]; }

		/// Direct access to image pixel/voxel, No bound-checking, writable.
		T& pixel(int x, int y) { return data[x+y**dim.data()]; }

		/// Direct access to image pixel/voxel. No bound-checking, no interpolation. (const overload)
		const T& operator()(int x, int y) const { return data[x+y**dim.data()]; }

		/// Direct access to image pixel/voxel, No bound-checking, writable.
		T& pixel(int x, int y, int z) {
			const auto _fast_dim=dim.data();
			return data[x+y*_fast_dim[0]+z*_fast_dim[1]*_fast_dim[0]];
		}

		/// Direct access to image pixel/voxel. No bound-checking, no interpolation. (const overload)
		const T& operator()(int x, int y, int z) const {
			const auto _fast_dim=dim.data();
			return data[x+y*_fast_dim[0]+z*_fast_dim[1]*_fast_dim[0]];
		}

		/// Helper function to clamp to value range
		static inline void clamp_edge(int &i, double& f, int n)
		{
			if (i<0) {i=0;f=0;}
			if (i>n-2) {
				f=1.0;
				i=n-2;
			}
		}

		/// Helper to copy data around. T2 can be a NRRD::ImageView<...> or raw pointer
		template <typename T2>
		void copyDataFrom(const T2* other)
		{
			int n=length();
			#pragma omp parallel for
			for (int i=0;i<n;i++)
				data[i]=(T)other[i];
		}

		/// Helper to copy data around. T2 can be a NRRD::ImageView<...> or raw pointer
		template <typename T2>
		void copyDataTo(const T2* other)
		{
			int n=other->length();
			#pragma omp parallel for
			for (int i=0;i<n;i++)
				other[i]=(T)data[i];
		}
		
		/// Access a voxel using tri-linear interpolation
		double operator()(double x, double y, double z=0) const
		{
			int    ixm=(int)x, iym=(int)y, izm=(int)z;
			double fx=x-ixm,   fy=y-iym,   fz=z-izm;
			clamp_edge(ixm,fx,dim[0]);
			clamp_edge(iym,fy,dim[1]);
			if (izm!=0&&fz!=0)
				clamp_edge(izm,fz,dim[2]);
			const ImageView<T>& I(*this);
			if (fx==0&&fy==0&&fz==0) // no interpolation needed
				return I(ixm,iym,izm);
			if (fx==0&&fy==0) // z-linear only
				return	(1.0-fz)*I(ixm,iym,izm)+fz*I(ixm,iym,izm+1);
			if (fz==0) // xy-bilinear only (in case of 2D-data)
				return	(1.0-fy)*((1.0-fx)*I(ixm,iym,izm) +fx*I(ixm+1,iym,izm))
						+fy*((1.0-fx)*I(ixm,iym+1,izm) +fx*I(ixm+1,iym+1,izm));
			return // trilinear
					(1.0-fz)*(((1-fy)*((1.0-fx)*I(ixm,iym,izm) +fx*I(ixm+1,iym,izm)))
					+(fy*((1.0-fx)*I(ixm,iym+1,izm) +fx*I(ixm+1,iym+1,izm))))
					+fz*((1-fy)*((1.0-fx)*I(ixm,iym,izm+1) +fx*I(ixm+1,iym,izm+1))
					+(fy*((1.0-fx)*I(ixm,iym+1,izm+1) +fx*I(ixm+1,iym+1,izm+1))));
		}

		/// Save an image as a .nrrd file.
		bool save(const std::string& nrrd_file) const
		{
			nrrd_header["spacings"]=vectorToString<double>(element_spacing," ");
			return !!(*this) && NRRD::save<T>(nrrd_file,data,(int)dim.size(),&(dim[0]),meta_info,nrrd_header);
		}

		/// Access kinds of dimensions
		std::vector<std::string> getElementKinds() const
		{
			auto it=nrrd_header.find("kinds");
			if (it==nrrd_header.end()) return std::vector<std::string>();
			std::vector<std::string> kinds=stringToVector<std::string>(it->second);
			while (kinds.size()<dim.size()) kinds.push_back("domain");
			return kinds;
		}

		/// Set kinds of dimensions, such as
		/// "domain" (or "space", "time"), "list" (or "vector", "point", "normal" etc.),
		/// or for special dimensions "complex" (2) "3-color" (3) "4-color" (4) "quaternion" (4) etc.
		void setElementKind(int i, const std::string& kind="domain") {
			auto kinds=getElementKinds();
			if (kinds.empty())
				for (int i=0;i<dimension();i++)
					kinds.push_back("domain");
			kinds[i]=kind;
			nrrd_header["kinds"]=vectorToString(kinds);
		}

		/// Use a dimension of size 3 or 4 as RGB or RGBA color.
		void setElementKindColor(int i=0) {
			if (dim[i]==1) setElementKind(i,"scalar"); // intensity
			else if (dim[i]==3) setElementKind(i,"RGB-color");
			else if (dim[i]==4) setElementKind(i,"RGBA-color");
			else std::cerr << "NRRD::ImageView: " << "Color dimensions must have size 3 or 4." << std::endl;
		}

		/// True is i-th dimension is of size 3 or 4 and/or has kind RGB or RGBA color.
		bool isElementKindColor(int i=0) const {
			auto kinds=getElementKinds();
			if (kinds.empty()) return (dim[i]==1||dim[i]==3||dim[i]==4);
			else return	kinds[i]=="scalar" || kinds[i]=="RGB-color" || kinds[i]=="RGBA-color"
						|| kinds[i]=="3-color" || kinds[i]=="4-color"
						|| ((dim[i]>4||dim[i]==2) && kinds[i]=="vector") ;
        }

		/// Dictionary of meta-information. value by tag.
		std::map<std::string,std::string> meta_info;

		/// Dictionary of nrrd-fields
		mutable std::map<std::string,std::string> nrrd_header;
	};

} // namespace NRRD

#endif // __NRRD_IMAGE_VIEW_HXX
