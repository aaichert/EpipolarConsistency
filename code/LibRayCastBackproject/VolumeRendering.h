#ifndef __volume_rendering_h
#define __volume_rendering_h

// Types needed in both CUDA code and on CPU
#include "vr_cuda.hxx"

// Representation of  volume
#include "VoxelData.h"

#include <LibProjectiveGeometry/ProjectionMatrix.h>

#include <limits>
#include <vector>

// Predeclaration of some CUDA stuff
namespace UtilsCuda {
	template <typename T> class BindlessTexture3D;
	template <typename T> class MemoryBlock;
} // namespace UtilsCuda

namespace VolumeRendering {
	
	/// Defines visualization method (i.e. "Iso Surface", "Emission Absorption", "Digitally Reconstructed Radiograph")
	struct RaycastPass {
		virtual float getMinValue() {return std::numeric_limits<float>::quiet_NaN();}
		virtual float getMaxValue() {return std::numeric_limits<float>::quiet_NaN();}
		virtual void render(
			int n_u, int n_v, int n_c,								//< image size and number of channels
			float * pixel_data_d,									//< image data (RGBA)
			float * model_C_Pinv_d,									//< volume model transform (voxels to world)
			const UtilsCuda::BindlessTexture3D<float>& voxel_data,	//< volume data
			float * ray_entry_d,									//< ray entry
			float * ray_exit_d,										//< ray exit
			float * noise_d,										//< noise
			float samples_per_voxel									//< samples per voxel
			) =0;
	};

	/// Render 3D voxel data to 2D images
	class Raycaster {
	public:
		
		/// Prepare raycaster
		Raycaster(VoxelData& _volume);
		~Raycaster();

		/// Access to volume data
		VoxelData& getVolume();

		/// Uses Min/Max binning and iso-hit for a simple proxy geometry pass
		// void useEmptySpaceSkipping(bool _use_ess); // 2do replace with different ess classes, including cube_ess

		/// Define samples per mm
		Raycaster& setSamplesPerVoxel(double _samples_per_voxel);

		/// Set a vector of clip planes
		Raycaster& setClipPlanes(const std::vector<Geometry::RP3Plane>& clip_planes);

		/// Render a 2D projection of the volume (CUDA). P maps world units to pixels.
		bool render(NRRD::ImageView<float> image, const Geometry::ProjectionMatrix& P);

		/// Access to Raycast Pass and its parameters. Also used to choose visalization method
		template <class RaycastPassType>
		RaycastPassType& raycastPass() {
			RaycastPassType* pass=dynamic_cast<RaycastPassType*>(raycast_pass);
			if (!pass) {
				if (raycast_pass) delete raycast_pass;
				raycast_pass=pass=new RaycastPassType();
			}
			return *pass;
		}

	protected:
		VoxelData							*volume;			//< Voxel data of object, including size, spacing and space skipping info.

		float								samples_per_voxel;	//< Sampling distance defined in voxel space (for anisotropic volumes)
		UtilsCuda::MemoryBlock<float>		*pixel_data;		//< RGBA output image on GPU.
		UtilsCuda::MemoryBlock<float>		*random_numbers;	//< An image filled with noise.
		UtilsCuda::MemoryBlock<float>		*ray_geometry;		//< xyz-d / xyz-d ray entry and exit in voxels, depth in mm if defined.
		UtilsCuda::MemoryBlock<float>		*model_C_Pinv;		//< Model transform, camera center, projection matrix inverse 
		UtilsCuda::MemoryBlock<float>		*clip_planes;		//< Additional clip planes
		RaycastPass							*raycast_pass;		//< Actual work done here. See also setVisualizationMethod(...).
	public:
	};

	struct Debug :  public RaycastPass
	{
		virtual void render(int n_u, int n_v, int n_c, float * pixel_data_d,
							float * model_C_Pinv_d, const UtilsCuda::BindlessTexture3D<float>& voxel_data,
							float * ray_entry_d, float * ray_exit_d,
							float * noise_d, float samples_per_voxel);
	};

	struct MaximumIntensityProjection :  public RaycastPass
	{
		virtual void render(int n_u, int n_v, int n_c, float * pixel_data_d,
							float * model_C_Pinv_d, const UtilsCuda::BindlessTexture3D<float>& voxel_data,
							float * ray_entry_d, float * ray_exit_d,
							float * noise_d, float samples_per_voxel);
	};

	class IsoSurface : public RaycastPass
	{
		float iso_value;

	public:
		virtual void render(int n_u, int n_v, int n_c, float * pixel_data_d,
							float * model_C_Pinv_d, const UtilsCuda::BindlessTexture3D<float>& voxel_data,
							float * ray_entry_d, float * ray_exit_d,
							float * noise_d, float samples_per_voxel);
		virtual float getMinValue() {return iso_value;}
		IsoSurface& setIsoValue(float _iso_value)
		{
			iso_value=_iso_value;
			return *this;
		}

	};

	class DigitallyReconstructedRadiograph : public RaycastPass
	{
		bool                 divide_by_ray_length; // needed for some reconstruction techniques.
	public:
		DigitallyReconstructedRadiograph() : divide_by_ray_length(0) {}
		virtual void render(int n_u, int n_v, int n_c, float *pixel_data_d,
							float * model_C_Pinv_d, const UtilsCuda::BindlessTexture3D<float>& voxel_data,
							float * ray_entry_d, float * ray_exit_d,
							float * noise_d, float samples_per_voxel);
		
		DigitallyReconstructedRadiograph& setRayLangthWeighted(bool weighted=true) { divide_by_ray_length=weighted; return *this; }

	};

	// 2do struct EmissionAbsorption :  public RaycastPass {};




} // namespace VolumeRendering

#endif // __volume_rendering_h
