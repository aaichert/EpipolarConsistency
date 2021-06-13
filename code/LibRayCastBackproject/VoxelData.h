#ifndef __voxel_data_h
#define __voxel_data_h

#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>

#include <NRRD/nrrd_image_view.hxx>

struct float2;

// Predeclaration of some CUDA BindlessTexture3D
namespace UtilsCuda {
	template <typename T> class BindlessTexture3D;
} // namespace UtilsCuda

namespace VolumeRendering {
	class EmptySpaceSkippingGeometry;

	class VoxelData {
		Geometry::RP3Homography				model_vx2au;		//< Mapping voxles to world units. Should be an affinity.
		int									volume_size_vx[3];	//< Number of voxels
		float								voxel_spacing[3];	//< Voxel spacing of input volume.
		UtilsCuda::BindlessTexture3D<float> *voxel_data;		//< Volume data set given as 3D array using bilinear interpolation

		// Optimized Ray-Casting via ESS
		EmptySpaceSkippingGeometry			*ess_geometry;		//< Allows for the exclusion of empty scace from ray casting operations
		float								value_min;			//< Minimum intensity in volume data
		float								value_max;			//< Maximum intensity in volume data

	public:
		/// Download a 3D image to GPU for visualization
		VoxelData(const NRRD::ImageView<float>& _voxel_data_cpu, bool use_ess=true);
		VoxelData(float *gpu_data, Eigen::Vector3i size, Eigen::Vector3f spacing=Eigen::Vector3f::Ones());
		~VoxelData();

		/// Define how voxels map to world units. (Default applies voxel scaling and centers volume)
		VoxelData& setModelTransform(const Geometry::RP3Homography& _model_vx2au);

		/// Get mapping from voxels to world units
		const Geometry::RP3Homography& getModelTransform() const;

		/// Centers the volume in the world.
		VoxelData& centerVolume();

		/// Use empty space skipping
		VoxelData& emptySpaceSkipping(int bin_factor=16);

		// Number of voxels in dimension = 1,2,3
		int size(int dim) const {return volume_size_vx[dim];}

		// Size of a voxel in dimension = 1,2,3
		float spacing(int dim) const {return voxel_spacing[dim];}
		
		/// Minimum intensity in volume data
		float getValueMin()     const { return value_min; }
		/// Maximum intensity in volume data
		float getValueMax()     const { return value_max; }

		/// Access proxy geometry
		EmptySpaceSkippingGeometry* ess() { return ess_geometry;}

		/// Access CUDA texture
		const UtilsCuda::BindlessTexture3D<float>& getTexture() const { return *voxel_data; }

	};

	/// Empty space skipping by computing a downsized version of the volume with min/max information of bins.
	class EmptySpaceSkippingGeometry {
	protected:
		float								value_min;			//< Minimum intensity in volume data
		float								value_max;			//< Maximum intensity in volume data

	public:
		EmptySpaceSkippingGeometry(float _value_min=0, float _value_max=1)
			: value_min(_value_min)
			, value_max(_value_max)
		{}
		virtual ~EmptySpaceSkippingGeometry(){}
				
		/// Minimum intensity in volume data
		float getValueMin()     const { return value_min; }
		/// Maximum intensity in volume data
		float getValueMax()     const { return value_max; }

	};

	/// Simple ESS using a downsampled volume
	class SimpleBinningESS : public EmptySpaceSkippingGeometry {
		int					bin_size;	//< How many voxels are skipped at once during ESS
		UtilsCuda::BindlessTexture3D<float2>* voxel_data_binned_minmax;	//< minimum/maximum value in ball with radius bin_size
	public: 
		SimpleBinningESS(UtilsCuda::BindlessTexture3D<float>& volume, int bin_size);
		UtilsCuda::BindlessTexture3D<float2>& getBinnedMinMax() const { return *voxel_data_binned_minmax; }
		int   getBinSize() const { return bin_size;  }
		~SimpleBinningESS();
	};


} // namespace VolumeRendering

#endif // __volume_rendering_h
