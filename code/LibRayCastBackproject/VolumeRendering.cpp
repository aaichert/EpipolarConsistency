#include <VolumeRendering.h>

#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>

#include <LibUtilsCuda/culaut/culaut.hxx>

#include <LibUtilsQt/Figure.hxx>
using UtilsQt::Figure;

// GPU ray geometry set up (bounding box case)
extern void rayGeometryAABB(
	int n_u, int n_v,
	AxisAlignedBox<float> box,
	float * Pinv_d, float * C_d,
	float * out_ray_entry_d, float * out_ray_exit_d,
	int n_clip_planes=0, float * clip_planes_d=0x0);

// GPU ray geometry empty space skipping (iso-hit binned min/max boxes)
extern void rayGeometryESS(
	int n_u, int n_v,
	float * out_ray_entry_d, float * out_ray_exit_d,
	int bin_size, cudaTextureObject_t binned_minmax,
	float value_min, float value_max
	);

// GPU ray casting. Maximum intensity Projection
extern void raycast_debug(
	int n_u, int n_v, int n_c,		//< image size
	float * pixel_data_d,			//< image data (RGBA)
	float * model_C_Pinv_d,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float * ray_entry_d,			//< ray entry
	float * ray_exit_d,				//< ray exit
	float samples_per_voxel			//< samples per voxel
	);

// GPU ray casting. Digitally Reconstructed Radiograph.
extern void raycast_drr(
	int n_u, int n_v, int n_c,		//< image size
	float * pixel_data_d,			//< image data (RGBA)
	float * model_C_Pinv_d,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float * ray_entry_d,			//< ray entry
	float * ray_exit_d,				//< ray exit
	float * noise_d,				//< noise for ray offsets 
	float samples_per_voxel			//< samples per voxel
	);

// GPU ray casting. Iso-surface.
extern void raycast_iso(
	int n_u, int n_v, int n_c,		//< image size
	float * pixel_data_d,			//< image data (RGBA)
	float * model_C_Pinv_d,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float * ray_entry_d,			//< ray entry
	float * ray_exit_d,				//< ray exit
	float * noise_d,				//< noise for ray offsets 
	float samples_per_voxel,		//< samples per voxel
	float iso_val					//< iso value
	);

// GPU ray casting. Maximum intensity Projection
extern void raycast_mip(
	int n_u, int n_v, int n_c,		//< image size
	float * pixel_data_d,			//< image data (RGBA)
	float * model_C_Pinv_d,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float * ray_entry_d,			//< ray entry
	float * ray_exit_d,				//< ray exit
	float * noise_d,				//< noise for ray offsets 
	float samples_per_voxel			//< samples per voxel
	);

// GPU ray casting. Emission-absorption with 1D transfer function
void raycast_ea_1Dtf(
	int n_u, int n_v, int n_c,		//< image size
	float * pixel_data_d,			//< image data (RGBA)
	float * model_C_Pinv_d,			//< volume model transform (voxels to world)
	cudaTextureObject_t voxel_data,	//< volume data
	float * ray_entry_d,			//< ray entry
	float * ray_exit_d,				//< ray exit
	float * noise_d,				//< noise for ray offsets 
	float samples_per_voxel,		//< samples per voxel
	cudaTextureObject_t tf			//< transfer function
	);

// Just for debugging.
//#include <Utils/TimerWin32.hxx>

namespace VolumeRendering {

	Raycaster::Raycaster(VoxelData& _volume)
		: volume(&_volume)
		, pixel_data(new UtilsCuda::MemoryBlock<float>())
		, random_numbers(new UtilsCuda::MemoryBlock<float>())
		, model_C_Pinv(new UtilsCuda::MemoryBlock<float>())
		, ray_geometry(new UtilsCuda::MemoryBlock<float>())
		, clip_planes(0x0)
		, raycast_pass(0x0)
	{}

	Raycaster::~Raycaster()
	{
		delete pixel_data;
		delete random_numbers;
		delete model_C_Pinv;
		delete ray_geometry;
		if (clip_planes) delete clip_planes;
		if (raycast_pass) delete raycast_pass;
	}

	VoxelData& Raycaster::getVolume()
	{
		return *volume;
	}

	Raycaster& Raycaster::setSamplesPerVoxel(double _samples_per_voxel)
	{
		samples_per_voxel=(float)_samples_per_voxel;
		return *this;
	}

	Raycaster& Raycaster::setClipPlanes(const std::vector<Geometry::RP3Plane>& _clip_planes)
	{
		if (_clip_planes.empty())
		{
			if (clip_planes) delete clip_planes;
			clip_planes=0x0;
			return *this;
		}
		if (!clip_planes) clip_planes=new UtilsCuda::MemoryBlock<float>();
		std::vector<Eigen::Vector4f> planes_vx(_clip_planes.size());
		for (int i=0;i<(int)_clip_planes.size();i++)
			planes_vx[i]=_clip_planes[i].cast<float>();
		clip_planes->allocate(4*(int)_clip_planes.size());
		clip_planes->download((float*)(&planes_vx[0]));
		return *this;
	}

	bool Raycaster::render(NRRD::ImageView<float> image, const Geometry::ProjectionMatrix& P)
	{
		// Test whether we have a multi-channel image
		bool is_element_kind_color=image.dimension()==3;
		if (is_element_kind_color && image.size(2)>4) image.setElementKind(0,"vector");
		if (is_element_kind_color && image.size(2)<4) image.setElementKindColor();

		// Allocate output image
		int n_c=is_element_kind_color?image.size(0):1;             // number of channels
		int n_u=is_element_kind_color?image.size(1):image.size(0); // image width
		int n_v=is_element_kind_color?image.size(2):image.size(1); // image height
		pixel_data->allocate(n_u*n_v*n_c);
		// Allocate memory for ray geometry (two four-channel images)
		ray_geometry->allocate(n_u*n_v*4*2);

		// Random ray offsets
		if (random_numbers->allocate(n_u*n_v))
		{
			int l=n_u*n_v;
			std::vector<float> noise(l);
			for (int i=0;i<l;i++)
				noise[i]=(float)rand()/RAND_MAX;
			random_numbers->download(&noise[0]);
		}

		// Bounding box of volume
		// (usually, voxel coordinate system starts in zero voxel... could probably drop first tree elements)
		AxisAlignedBox<float> box;
		box.aabb_min[0]=0.f;
		box.aabb_min[1]=0.f;
		box.aabb_min[2]=0.f;
		box.aabb_max[0]=(float)volume->size(0);
		box.aabb_max[1]=(float)volume->size(1);
		box.aabb_max[2]=(float)volume->size(2);

		// Geometry: projection matrix (voxels to pixels), its inverse and camera center.
		Geometry::ProjectionMatrix			P_vx=P*volume->getModelTransform(); //< Model transform maps world units to voxels
		Geometry::ProjectionMatrixInverse	Pinv_vx=Geometry::pseudoInverse(P_vx);
		Geometry::RP3Point					C_vx=Geometry::getCameraCenter(P_vx);
		// Make sure that parallel geometries are handled correctly (i.e. C is truly infinite)
		if (C_vx[3]<1e-12) C_vx[3]=0;
		// Prepare memory block on host to copy to device
		float model_C_Pinv_h[4*4+4*3+4];
		culaut::xvcpy<float,double,4*3>(model_C_Pinv_h+4*4,Pinv_vx.data());
		culaut::xvcpy<float,double,4>  (model_C_Pinv_h+4*4+4*3,C_vx.data());
		culaut::xvcpy<float,double,4*4>(model_C_Pinv_h+0,volume->getModelTransform().data());
		// Allocate space for model and projection matices, as well as camera center and copy to device
		model_C_Pinv->allocate(4*4+4*3+4);
		model_C_Pinv->download(model_C_Pinv_h);
	
		// Prepare geometry
		rayGeometryAABB(
			n_u,n_v,                             //< image size
			box,                               	 //< bounding box size in voxels
			(float*)*model_C_Pinv+4*4,           //< pseudoinverse of projection matrix in voxels (stored in this array after the 4x4 model matrix)
			(float*)*model_C_Pinv+4*4+4*3,       //< camera center in voxel coordinate system (stored after the above)
			(float*)*ray_geometry,               //< output: ray entry point (intersection with bounding box closer to camera center)
			(float*)*ray_geometry+n_u*n_v*4,     //< output: ray exit point (intersection with bounding box behind entry point)
			clip_planes?clip_planes->size()/4:0, //< optional: number of clip planes (to cut off additional partx of bounding box)
			clip_planes?(float*)*clip_planes:0x0 //< optional: clip planes (plane equations in voxel coordinate system)
			);

		// By default resort to debugging raycast pass which shows start/end/direction and mip
		if (raycast_pass==0x0)  raycast_pass=new Debug();

		// Optimize ray geometry
		bool min_isnan=std::isnan(raycast_pass->getMinValue());
		bool max_isnan=std::isnan(raycast_pass->getMaxValue());

		SimpleBinningESS* ess=dynamic_cast<SimpleBinningESS*>(volume->ess());
		if (ess && !(min_isnan&&max_isnan))
		{
			float min=min_isnan?volume->getValueMin():raycast_pass->getMinValue();
			float max=max_isnan?volume->getValueMax():raycast_pass->getMaxValue();
			rayGeometryESS(
				n_u,n_v,
				(float*)*ray_geometry,
				(float*)*ray_geometry+n_u*n_v*4,
				ess->getBinSize(),
				ess->getBinnedMinMax(),
				min,max
				);
		}

		//////////////////////////////////////////////////////////////////////////////////
		// // This block of debugging code is useful for verifying correct geometry setup.
		//NRRD::Image<float> debug;
		//debug.set(4,n_u,n_v*2);
		//debug.setElementKindColor();
		//ray_geometry->readback(debug);
		//NRRD::ImageView<float> ray_entry(4,n_u,n_v,(float*)debug);
		//NRRD::ImageView<float> ray_exit (4,n_u,n_v,(float*)debug+n_u*n_v*4);
		//ray_entry.setElementKindColor();
		//ray_exit .setElementKindColor();
		//using UtilsQt::Figure;
		//Figure("Ray Entry",ray_entry);
		//Figure("Ray Exit" ,ray_exit );
		// ////////////////////////////////////////////////////////////////////////////////

		if (samples_per_voxel<0.1f) samples_per_voxel=0.1f;
		if (samples_per_voxel>2.f) samples_per_voxel=2.f;

		raycast_pass->render(
			n_u,n_v,n_c,					//< image size and number of channels
			(float*)*pixel_data,			//< image data
			(float*)*model_C_Pinv,			//< volume model transform (voxels to world)
			volume->getTexture(),			//< volume data
			(float*)*ray_geometry,			//< ray entry
			(float*)*ray_geometry+n_u*n_v*4,//< ray exit
			(float*)*random_numbers,		//< Some uniform noise the size of the image
			samples_per_voxel				//< samples per voxel;
			);

		pixel_data->readback(image);

		return true;
	}

	void Debug::render(int n_u, int n_v, int n_c, float * pixel_data_d,
							float * model_C_Pinv_d, const UtilsCuda::BindlessTexture3D<float>& voxel_data,
							float * ray_entry_d, float * ray_exit_d,
							float * noise_d, float samples_per_voxel)
	{
		raycast_debug(n_u, n_v, n_c, pixel_data_d,
				model_C_Pinv_d, voxel_data,
				ray_entry_d, ray_exit_d, samples_per_voxel
		);
	}

	void MaximumIntensityProjection::render(int n_u, int n_v, int n_c, float * pixel_data_d,
			float * model_C_Pinv_d, const UtilsCuda::BindlessTexture3D<float>& voxel_data,
			float * ray_entry_d, float * ray_exit_d,
			float * noise_d, float samples_per_voxel)
	{
		raycast_mip(n_u, n_v, n_c, pixel_data_d,
				model_C_Pinv_d, voxel_data,
				ray_entry_d, ray_exit_d,
				noise_d, samples_per_voxel
		);
	}

	void IsoSurface::render(int n_u, int n_v, int n_c, float * pixel_data_d,
			float * model_C_Pinv_d, const UtilsCuda::BindlessTexture3D<float>& voxel_data,
			float * ray_entry_d, float * ray_exit_d,
			float * noise_d, float samples_per_voxel)
	{
		raycast_iso(n_u, n_v, n_c, pixel_data_d,
					model_C_Pinv_d, voxel_data,
					ray_entry_d, ray_exit_d,
					noise_d, samples_per_voxel,
					iso_value
			);
	}

	void DigitallyReconstructedRadiograph::render(int n_u, int n_v, int n_c, float * pixel_data_d,
			float * model_C_Pinv_d, const UtilsCuda::BindlessTexture3D<float>& voxel_data,
			float * ray_entry_d, float * ray_exit_d,
			float * noise_d, float samples_per_voxel)
	{
		raycast_drr(n_u, n_v, n_c, pixel_data_d,
					model_C_Pinv_d, voxel_data,
					ray_entry_d, ray_exit_d,
					noise_d, samples_per_voxel
			);
	}


} // namespace VolumeRendering
