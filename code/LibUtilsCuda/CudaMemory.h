#ifndef _cuda_momory_hxx
#define _cuda_momory_hxx
// Created ba A. Aichert on Mon Aug 17th 2015

#include "UtilsCuda.hxx"

// enable this to keep a list of size and type of active allocations from this class.
#define _cuda_momory_hxx_debug_info

#ifdef _cuda_momory_hxx_debug_info
	#include <map>
	#include <GetSet/StringType.hxx>
	extern void setMemoryAllocation(std::string type, void * location, int length, bool active);
	namespace UtilsCuda {
		std::map<void*,std::pair<int,std::string> > getLog();
	} // namespace UtilsCuda
#endif // _cuda_momory_hxx_debug_info

/// Classes to keep track of CUDA memory allocations
namespace UtilsCuda
{
	/// A view to Device memory at position ptr_d and length n*sizeof(T)
	template <typename T>
	class MemoryView
	{
	protected:
			int n;
			T * ptr_d;

	public:
		MemoryView(T * _ptr_d=0x0, int _n=0)
			: n(_n)
			, ptr_d(_ptr_d)
		{}

		/// Copy from Host to Device
		MemoryView<T>& download(const T* data_host)
		{
			// Copy data
			cudaMemcpy(ptr_d,data_host,n*sizeof(T),cudaMemcpyHostToDevice);
			cudaCheckState
			return *this;
		}

		/// Copy from Device to Host
		void readback(T* out_host)
		{
			if (n<=0) return;
			cudaMemcpy(out_host,ptr_d,n*sizeof(T),cudaMemcpyDeviceToHost);
			cudaCheckState
		}

		/// Access a sub-range of this array
		MemoryView<T> subMemory(int start, int length)
		{
			return MemoryView<T>(ptr_d+start,length);
		}

		/// Mimic pointer behavior: Automatic cast to pointer
		operator T*() {return ptr_d;}

		/// Mimic pointer behavior: Automatic cast to const pointer
		operator const T*() const {return ptr_d;}

		/// Mimic pointer behavior: Test for validity
		bool operator!() const {return n<=0||ptr_d<=0;}

		/// What is the size of this block?
		int size() const { return n; }
	};

	/// Class to keep track of Device memory allocations
	template <typename T>
	class MemoryBlock : public MemoryView<T>
	{
	public:
        using MemoryView<T>::n;
        using MemoryView<T>::ptr_d;

		MemoryBlock(int _n=0, const T* data_host=0x0)  : MemoryView<T>()
		{
			allocate(_n);
			download(data_host,n);
		}
		
		~MemoryBlock()
		{
			dealloc();
		}
		
		/// Allocate memory (if needed) and copy from Host to Device. Returns true if (de-)allocation occured.
		bool allocate(int _n)
		{
			// (Re-)allocate if needed
			if (_n!=n && _n>0)
			{
				dealloc();
				n=_n;
				cudaMalloc((void**)&ptr_d, sizeof(T)*n);
				#ifdef _cuda_momory_hxx_debug_info
				setMemoryAllocation(typeName<T>(),(void*)ptr_d,n,true);
				#endif // _cuda_memory_hxx_debug_info
				cudaCheckState
				return true;
			}
			// Ensure validity of input
			if (n<0)
			{
				dealloc();
				return true;
			}
			return false;
		}

		/// Deallocate previously allocated memory
		void dealloc()
		{
			#ifdef _cuda_momory_hxx_debug_info
			setMemoryAllocation(typeName<T>(),(void*)ptr_d,n,false);
			#endif // _cuda_memory_hxx_debug_info
			if (ptr_d)
				cudaFree(ptr_d);
			ptr_d=0x0;
			n=0;
		}

		/// Allocate memory (if needed) and copy from Host to Device
		void download(const T* data_host, int _n=0)
		{
			allocate(_n);
			if (n>0 && data_host)
				cudaMemcpy(MemoryView<T>::ptr_d,data_host,n*sizeof(T),cudaMemcpyHostToDevice);
			cudaCheckState
		}

	};

} // namespace UtilsCuda

#endif // _cuda_momory_hxx
