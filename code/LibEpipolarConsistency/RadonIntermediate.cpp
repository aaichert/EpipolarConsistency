
#include "RadonIntermediate.h"

#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>

const double Pi=3.1415926535897931;

/// CUDA implementation of radon transformation based on explicit formulation of the line equation and evaluation (more flexible)
extern void computeDerivLineIntegrals(cudaTextureObject_t in, int n_x, int n_y, int n_alpha, int n_t, int filter, int post_process, float *out_d);

namespace EpipolarConsistency
{

	RadonIntermediate::RadonIntermediate(const NRRD::ImageView<float>& projectionData, int size_alpha, int size_t, Filter filter, PostProcess post_process)
		: m_bin_size_angle(0)
		, m_bin_size_distance(0)
		, m_filter(filter)
		, n_x(0)
		, n_y(0)
		, n_alpha(size_alpha)
		, n_t(size_t)
		, m_tex(0x0)
	{
		m_raw_gpu=new UtilsCuda::MemoryBlock<float>(projectionData.size(0)*projectionData.size(1), projectionData);
		// Temporary texture
		UtilsCuda::BindlessTexture2D<float> tmp_tex(projectionData.size(0),projectionData.size(1),*m_raw_gpu,true);
		compute(tmp_tex,size_alpha,size_t, filter, post_process);
	}

	RadonIntermediate::RadonIntermediate(const UtilsCuda::BindlessTexture2D<float>& projectionData, int size_alpha, int size_t, Filter filter, PostProcess post_process)
		: m_bin_size_angle(0)
		, m_bin_size_distance(0)
		, m_filter(filter)
		, n_x(0)
		, n_y(0)
		, n_alpha(size_alpha)
		, n_t(size_t)
		, m_tex(0x0)
	{
		m_raw_gpu=new UtilsCuda::MemoryBlock<float>();
		compute(projectionData,size_alpha,size_t, filter, post_process);
	}

	RadonIntermediate::RadonIntermediate(const std::string path)
		: m_bin_size_angle(0)
		, m_bin_size_distance(0)
		, n_x(0)
		, n_y(0)
		, n_alpha(0)
		, n_t(0)
		, m_tex(0x0)
		, m_raw_gpu(0x0)
    {
		m_raw_cpu.load(path);
		if (!m_raw_cpu)
		{
			std::cerr << "Failed to load " << path << std::endl;
			return;
		}
		readPropertiesFromMeta(m_raw_cpu.meta_info);
		n_alpha = m_raw_cpu.size(0);
		n_t = m_raw_cpu.size(1);
		m_raw_gpu=new UtilsCuda::MemoryBlock<float>(m_raw_cpu.length(),m_raw_cpu);
	}

	RadonIntermediate::RadonIntermediate(const NRRD::ImageView<float>& radon_intermediate_image)
		: m_bin_size_angle(0)
		, m_bin_size_distance(0)
		, n_x(0)
		, n_y(0)
		, n_alpha(0)
		, n_t(0)
		, m_tex(0x0)
	{
		m_raw_cpu.clone(radon_intermediate_image);
		replaceRadonIntermediateData(m_raw_cpu);
	}
	
	void RadonIntermediate::readPropertiesFromMeta(std::map<std::string, std::string> dict)
	{
		m_bin_size_angle = stringTo<double>(dict["Bin Size/Angle"]);
		m_bin_size_distance = stringTo<double>(dict["Bin Size/Distance"]);
		n_x = stringTo<int>(dict["Original Image/Width"]);
		n_y = stringTo<int>(dict["Original Image/Height"]);
		if (m_raw_cpu.meta_info["Filter"]=="Ramp")
			m_filter=Ramp;
		else if (m_raw_cpu.meta_info["Filter"]=="Derivative")
			m_filter=Derivative;
		else
			m_filter=None;
	}

	void RadonIntermediate::writePropertiesToMeta(std::map<std::string, std::string> &dict) const
	{
		dict["Bin Size/Angle"]=toString(m_bin_size_angle);
		dict["Bin Size/Distance"]=toString(m_bin_size_distance);
		dict["Original Image/Width"]=toString(n_x);
		dict["Original Image/Height"]=toString(n_y);
		dict["Filter"]=m_filter==Derivative?"Derivative":(m_filter==Ramp?"Ramp":"None");
	}

	void RadonIntermediate::replaceRadonIntermediateData(const NRRD::ImageView<float>& radon_intermediate_image)
	{
		// Copy and download new data
		m_raw_cpu.clone(radon_intermediate_image);
		if (!m_raw_gpu) 
			m_raw_gpu = new UtilsCuda::MemoryBlock<float>(radon_intermediate_image.length(), radon_intermediate_image);
		else {
			m_raw_gpu->download(radon_intermediate_image);
		}
		// Set variables compatible to new image dimensions
		n_alpha = radon_intermediate_image.size(0);
		n_t = radon_intermediate_image.size(1);
		double diagonal = std::sqrt((double)n_y*n_y + n_x*n_x);
		m_bin_size_distance = diagonal / n_t;
		readPropertiesFromMeta(radon_intermediate_image.meta_info);
		// Invalidate old texture data
		delete m_tex;
		m_tex = 0x0;
	}

	RadonIntermediate::Filter RadonIntermediate::getFilter() const
	{
		return m_filter;
	}

	bool RadonIntermediate::isDerivative() const
	{
		return m_filter==Derivative;
	}
	
	RadonIntermediate::~RadonIntermediate()
	{
		delete m_raw_gpu;
		delete m_tex;
	}

	void RadonIntermediate::clearRawData() {
		getTexture();
		if (m_raw_gpu) delete m_raw_gpu;
		m_raw_gpu=0x0;
		m_raw_cpu.set(0x0,0);
	}

	void RadonIntermediate::readback(bool gpu_memory_only)
	{
		// Copy texture if required.
		if (!m_raw_gpu) m_raw_gpu = new UtilsCuda::MemoryBlock<float>();
		if (m_raw_gpu->size()<=0)
			m_tex->readback(*m_raw_gpu);
		// Optionally, do nothing else.
		if (gpu_memory_only) {
			m_raw_cpu.set(0x0,0);
			return;
		}
		// read back memory to RAM and set meta info.
		m_raw_cpu.set(n_alpha,n_t);
		writePropertiesToMeta(m_raw_cpu.meta_info);
		m_raw_gpu->readback(m_raw_cpu);
	}

	int RadonIntermediate::getRadonBinNumber(int dim) const {
		if (dim) return n_t;
		else     return n_alpha;
	}

	int RadonIntermediate::getOriginalImageSize(int dim) const {
		if (dim) return n_y;
		else     return n_x;
	}

	double RadonIntermediate::getRadonBinSize(int dim) const {
		if (dim) return m_bin_size_distance;
		else     return m_bin_size_angle;
	}

	NRRD::ImageView<float>& RadonIntermediate::data() {
		return m_raw_cpu;
	}

	const NRRD::ImageView<float>& RadonIntermediate::data() const {
		return m_raw_cpu;
	}

	UtilsCuda::BindlessTexture2D<float>* RadonIntermediate::getTexture()
	{
		if (m_tex) return m_tex;
		if (!(*m_raw_gpu)) return 0x0;
		m_tex=new UtilsCuda::BindlessTexture2D<float>(n_alpha,n_t,*m_raw_gpu,true,true,true);
		delete m_raw_gpu;
		m_raw_gpu=0x0;
		return m_tex;
	}

	void RadonIntermediate::compute(const UtilsCuda::BindlessTexture2D<float>& projectionData, int n_alpha, int n_t,  Filter filter, PostProcess post_proces)
	{
		if (m_tex) delete m_tex;
		m_tex=0x0;
		n_x=projectionData.size[0];
		n_y=projectionData.size[1];
		double diagonal=std::sqrt((double)n_y*n_y+n_x*n_x);
		m_bin_size_distance=diagonal/n_t;
		m_bin_size_angle=Pi/n_alpha;
		m_filter=filter;
		m_raw_gpu->allocate(n_t*n_alpha);
		// Run Radon kernel and filter
		computeDerivLineIntegrals(projectionData,n_x,n_y,n_alpha,n_t, m_filter, (int)post_proces , *m_raw_gpu);
	}

} // namespace EpipolarConsistency
