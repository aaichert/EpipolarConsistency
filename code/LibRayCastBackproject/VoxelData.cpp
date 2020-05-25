#include <VoxelData.h>

#include <LibUtilsCuda/CudaMemory.h>
#include <LibUtilsCuda/CudaBindlessTexture.h>

#include <vector_types.h>

#include "vr_cuda.hxx"

#include <NRRD/nrrd_image.hxx>

#include <Utils/TimerWin32.hxx>

// Analyze and optimize volume data by binning
extern void minmaxBinning(
	int n_x, int n_y, int n_z,
	cudaTextureObject_t volume_data,
	int bin_size,
	UtilsCuda::BindlessTexture3D<float2>** out_texture,
	float & value_min, float& value_max);

namespace VolumeRendering {

	VoxelData::VoxelData(const NRRD::ImageView<float>& _voxel_data_cpu, bool use_ess)
		: voxel_data(0x0)
	{
		Utils::TimerWin32 time;
		voxel_data=new UtilsCuda::BindlessTexture3D<float>(
			_voxel_data_cpu.size(0),
			_voxel_data_cpu.size(1),
			_voxel_data_cpu.size(2),
			(float*)_voxel_data_cpu);
		volume_size_vx[0]=_voxel_data_cpu.size(0);
		volume_size_vx[1]=_voxel_data_cpu.size(1);
		volume_size_vx[2]=_voxel_data_cpu.size(2);
		voxel_spacing[0]=(float)_voxel_data_cpu.spacing(0);
		voxel_spacing[1]=(float)_voxel_data_cpu.spacing(1);
		voxel_spacing[2]=(float)_voxel_data_cpu.spacing(2);
			
		/// Analyze input image
		std::cout << "Download Time:   " << time.getElapsedTime() << std::endl;
		if (use_ess) {
			ess_geometry=new SimpleBinningESS(*voxel_data,8);
			value_min=ess_geometry->getValueMin();
			value_max=ess_geometry->getValueMax();
		}
		else
		{
			int l=_voxel_data_cpu.length();
			value_min=value_max=_voxel_data_cpu[0];
			#pragma omp parallel for 
			for (int i=0;i<l;i++)
			{
				auto& v=_voxel_data_cpu[i];
				if (value_min>v)value_min=v;
				if (value_max<v)value_max=v;
			}
			ess_geometry=new EmptySpaceSkippingGeometry();
		}

		std::cout << "Analysis Time:   " << time.getElapsedTime() << std::endl;
		std::cout << "Data Range:      " << getValueMin() << " to " << getValueMax() << std::endl;

		centerVolume();
	}

	VoxelData::VoxelData(float *gpu_data, Eigen::Vector3i size, Eigen::Vector3f spacing)
		: voxel_data(0x0)
	{
		Utils::TimerWin32 time;
		voxel_data=new UtilsCuda::BindlessTexture3D<float>(
			size[0], size[1], size[2], gpu_data,true);
		volume_size_vx[0]=size[0]; 
		volume_size_vx[1]=size[1];
		volume_size_vx[2]=size[2];
		voxel_spacing [0]=spacing[0];
		voxel_spacing [1]=spacing[1];
		voxel_spacing [2]=spacing[2];
			
		/// Analyze input image
		std::cout << "Download Time:   " << time.getElapsedTime() << std::endl;
		value_min=0;
		value_max=1;
		ess_geometry=new EmptySpaceSkippingGeometry();
		centerVolume();
	}


	VoxelData::~VoxelData()
	{
		if (voxel_data)   delete voxel_data;
		if (ess_geometry) delete ess_geometry;
	}

	VoxelData& VoxelData::setModelTransform(const Geometry::RP3Homography& _model_vx2au)
	{
		model_vx2au=_model_vx2au;
		return *this;
	}

	const Geometry::RP3Homography& VoxelData::getModelTransform() const
	{
		return model_vx2au;
	}
			
	VoxelData& VoxelData::centerVolume()
	{
		model_vx2au=Eigen::Vector4d(voxel_spacing[0],voxel_spacing[1],voxel_spacing[2],1.0).asDiagonal();
		model_vx2au.block<3,1>(0,3)=Eigen::Vector3d(
			-0.5*volume_size_vx[0]*voxel_spacing[0],
			-0.5*volume_size_vx[1]*voxel_spacing[1],
			-0.5*volume_size_vx[2]*voxel_spacing[2]);
		return *this;
	}
	
	VoxelData& VoxelData::emptySpaceSkipping(int bin_factor) {
		if (ess_geometry) delete ess_geometry;
		if (bin_factor<=1)
			ess_geometry=new EmptySpaceSkippingGeometry();
		else
			ess_geometry=new SimpleBinningESS(*voxel_data,bin_factor);
		return *this;
	}
	
	SimpleBinningESS::SimpleBinningESS(UtilsCuda::BindlessTexture3D<float>& volume, int _bin_size)
	{
		minmaxBinning(volume.size[0],volume.size[1],volume.size[2],volume,bin_size, &voxel_data_binned_minmax, value_min, value_max);
	}
	
	SimpleBinningESS::~SimpleBinningESS() { delete voxel_data_binned_minmax; }

} // namespace VolumeRendering
