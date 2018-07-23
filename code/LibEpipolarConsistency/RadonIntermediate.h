// Created by A. Aichert on Thu Nov 21st 2013
#ifndef __radon_derivative
#define __radon_derivative

#include <NRRD/nrrd_image.hxx>
#include "LibProjectiveGeometry/ProjectiveGeometry.hxx"

// Predeclaration of some CUDA stuff
namespace UtilsCuda {
	template <typename T> class BindlessTexture2D;
	template <typename T> class MemoryBlock;
} // namespace UtilsCuda

namespace EpipolarConsistency
{
	using Geometry::Pi;
	/// Compute derivative in t-direction of Radon transform of x-ray projection data.
	class RadonIntermediate {
	public:
		/// Ctor computes radon derivative right away.
		RadonIntermediate(const NRRD::ImageView<float>& projectionData, int size_alpha, int size_t, bool computeDerivative=true);

		/// Ctor computes radon derivative right away.
		RadonIntermediate(const UtilsCuda::BindlessTexture2D<float>& projectionData, int size_alpha, int size_t, bool computeDerivative=true);

		/// Ctor loads a previously saved dtr.
		RadonIntermediate(const std::string path);
		
		/// Ctor uses existing CPU memory.
		RadonIntermediate(const NRRD::ImageView<float>& radon_intermediate_image);

		/// Dtor
		~RadonIntermediate();

		/// Get relevat parameters from meta dictionary.
		void readPropertiesFromMeta(std::map<std::string, std::string> dict);

		/// Store relevat parameters in meta dictionary.
		void writePropertiesToMeta(std::map<std::string, std::string> &dict) const;

		// Update Radon intermediate data with CPU memory. See also: readback() and data() member functions.
		void replaceRadonIntermediateData(const NRRD::ImageView<float>& radon_intermediate_image);

		/// If false, this is a standard Radon transform. If true, a derivtaive in direction of line distance has been computed.
		bool isDerivative() const;

		/// Readback Radon Intermediate data to CPU. If gpu_memory_only is set, the texture will be transferred to global GPU memory but not read back to RAM. See also: clearRawData().
		void readback(bool gpu_memory_only=false);

		/// Make sure a texture exists and then delete temporary memory.
		void clearRawData();

		/// Access to the size of the Radon transform. 0:distance 1:angle
		int getRadonBinNumber(int dim) const;

		/// Access to size of original image 0:width 1:height
		int getOriginalImageSize(int dim) const;

		/// Access to spacing of DTR. 0:angle 1:distance
		double getRadonBinSize(int dim=1) const;

		/// Copy to texture and get rid of plain array.
		UtilsCuda::BindlessTexture2D<float>* getTexture();

		/// Access to raw data on CPU (may return invalid image). See also: readback(...)
		NRRD::ImageView<float>& data();

		/// Access to raw data on CPU (may return invalid image). See also: readback(...)
		const NRRD::ImageView<float>& data() const;

		/// Sample given a line in the orginal image, relative to image center. Line will be overwritten with sample location. Must call readback(...) before use.
		template <typename Line>
		inline float sample(Line& line)
		{
			// Compute range of t-value in Radon transform this dtr corresponds to
			float range_t=(float)m_bin_size_distance*getRadonBinNumber(1);
			// Length of normal
			float length=sqrtf(line[0]*line[0]+line[1]*line[1]);
			// Angle between line and x-axis (scaled from 0 to +2).
			line[0]=atan2f(line[1],line[0])/(float)Pi;
			if (line[0]<0) line[0]+=2;
			// Distance to the origin (scaled to DTR bins)
			line[1]=-(line[2]/length)/range_t+0.5f;
			// Sample DTR and account for symmetry, up to sign.
			if (line[0]>1)
			{
				line[0]=line[0]-1.f;
				line[1]=1.f-line[1];
				// Otherwise, sample and invert sign in case of derivative.
				if (m_is_derivative)
					return -tex2D(line[0],line[1]);
				else
					return +tex2D(line[0],line[1]);
			}
			else // Otherwise, just sample.
				return tex2D(line[0],line[1]);
		}

		/// Sample Radon intermediate function in texture coordinates. Must call readback(...) before use.
		inline float tex2D(float s, float  t) { return (float)m_raw_cpu((n_alpha-1)*s,(n_t-1)*t); }

	protected:
		NRRD::Image<float>					m_raw_cpu;  //< Discrete representation of angular derivative of radon space. Optional data on CPU.
		UtilsCuda::MemoryBlock<float>		*m_raw_gpu; //< Discrete representation of angular derivative of radon space. Optional data on GPU.
		UtilsCuda::BindlessTexture2D<float>	*m_tex;     //< Same as m_raw_*, except as a texture. 
		UtilsCuda::MemoryBlock<float>		*m_random_numbers; //< Random numbers for ray offsets.
		
		double	m_bin_size_angle;			//< y-axis (angle) discretization.
		double	m_bin_size_distance;		//< x-axis (distance) discretization.
		bool    m_is_derivative;			//< if true, we have an odd function (i.e. derivative along t-direction).
		
		int		n_x;						//< Size of original image in X.
		int		n_y;						//< Size of original image in Y.
		int		n_t;						//< Number of bins for distance to the origin.
		int		n_alpha;					//< Number of bind for angle to the image X-axis.
		
		/// Runs Cuda kernel for Radon transform computation.
		void compute(const UtilsCuda::BindlessTexture2D<float>& projectionData, int size_alpha, int size_t, bool computeDerivative);

	};
} // namespace EpipolarConsistency

#endif // __radon_derivative
