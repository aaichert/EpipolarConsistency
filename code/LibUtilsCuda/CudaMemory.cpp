#include "CudaMemory.h"

#ifdef _cuda_momory_hxx_debug_info
	std::map<void*,std::pair<int,std::string> > _cuda_mem_alloc;
	void setMemoryAllocation(std::string type, void * location, int length, bool active)
	{
		if (active)
			_cuda_mem_alloc[location]=std::pair<int,std::string>(length,type);
		else
		{
			auto it=_cuda_mem_alloc.find(location);
			if (it!=_cuda_mem_alloc.end())
				_cuda_mem_alloc.erase(it);
		}
	}

	namespace UtilsCuda
	{
		std::map<void*,std::pair<int,std::string> > getLog()
		{
			return _cuda_mem_alloc;
		}

	} // namespace UtilsCuda
#endif 
