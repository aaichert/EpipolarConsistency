
#include "GeometryCalibration.h"

#include <LibRayCastBackproject/VoxelData.h>
#include <LibRayCastBackproject/VolumeRendering.h>

#include <Eigen/Dense>

/// f(x)=1-x^2+x^4 is zero at +/-1, has zero derivative at +/-1 and a maxiumum at f(0)=1; Values outside [-1,1] are clamped to zero. 
inline double weighting(double x)
{
	if (x<-1.0||x>1.0) return 0;
	double xx=x*x;
	return 1.0-2*xx+xx*xx;
}

namespace Calibration {
	template <class VectorOfVectors>
	double computeRadiusThreshold(const VectorOfVectors& beads)
	{
		if (beads.empty()) return 0;
		double sum=0;
		for (auto it=beads.begin();it!=beads.end();++it)
			sum+=it->tail(1)[0];
		double mean=sum/(double)beads.size();
		int n_large=0,n_small=0;
		double sum_large=0, sum_small=0;
		for (auto it=beads.begin();it!=beads.end();++it)
			if (it->tail(1)[0]>mean)  { n_large++; sum_large+=it->tail(1)[0]; }
			else                      { n_small++; sum_small+=it->tail(1)[0]; }
		double mean_large=sum_large/(double)n_large;
		double mean_small=sum_small/(double)n_small;
		return 0.5*(mean_large+mean_small);
	}

	double BeadPhantomDetection::computeRadiusThresholdMM(const std::vector<Eigen::Vector4d>& phantom_beads)
	{
		return computeRadiusThreshold(phantom_beads);
	}

	double BeadPhantomDetection::computeRadiusThresholdPx(const std::vector<Eigen::Vector3d>& detected_beads)
	{
		return computeRadiusThreshold(detected_beads);
	}

	void BeadPhantomDetection::gui_declare_section (const GetSetGui::Section& section) {
		GetSet<std::vector<Eigen::Vector4d>>("Location and Radius", section, beads).setDescription("3D Location of beads (X,Y,Z) and their radius (R) in mm. Format: \"X0 Y0 Z0 R0; X1 Y1 Z1 R1; ...\"");
	}

	void BeadPhantomDetection::gui_retreive_section(const GetSetGui::Section& section) {
		beads=GetSet<std::vector<Eigen::Vector4d>>("Location and Radius", section);
	}
	
	BeadPhantomSimulator::BeadPhantomSimulator() {}

	BeadPhantomSimulator::BeadPhantomSimulator(const BeadPhantomSimulator& other)
	{
		voxelization=other.voxelization;
		drr.voxel_data=0x0;
		drr.raycaster=0x0;
	}

	BeadPhantomSimulator::~BeadPhantomSimulator() {
		if (drr.raycaster ) delete drr.raycaster ;
		if (drr.voxel_data) delete drr.voxel_data;
		drr.raycaster =0x0;
		drr.voxel_data=0x0;
	}

	Geometry::RP3Homography	BeadPhantomSimulator::getModel() const {
		return Geometry::Scale(voxelization.voxel_spacing)*Geometry::Translation(-0.5*voxelization.voxel_number.cast<double>());
	}

	void BeadPhantomSimulator::voxelize(NRRD::Image<float>& phantom, const std::vector<Eigen::Vector4d>& beads, GetSetGui::ProgressInterface &app) const
	{
		// Initilalize 3D voxel data
		const auto& dim = voxelization.voxel_number;
		const auto& spacing = voxelization.voxel_spacing ;
		phantom.set(dim[0], dim[1], dim[2]);
		phantom.spacing(0) = spacing[0];
		phantom.spacing(1) = spacing[1];
		phantom.spacing(2) = spacing[2];
		const auto& model = getModel();
		// Set phantom to zero
		int l = phantom.length();
		#pragma omp parallel for
		for (int i = 0; i<l; i++) phantom[i] = 0;

		// Start progress
		bool cancel_clicked = false;
		app.progressStart(__FUNCTION__, "Computing Voxelized version of the bead phantom...", (int)beads.size(), &cancel_clicked);

		// Then loop over blocks with beads and draw them
		for (int i = 0; i<(int)beads.size(); i++)
		{
			if (cancel_clicked) break;
			app.progressUpdate(i);
			// Get Bead radius and transfrm bead to voxel coordinates.
			auto B(beads[i]);
			double r = B[3];
			double rc = std::ceil(r);
			B[3] = 1;
			// Figure out voxel range affected by bead (assumes model is affine)
			Eigen::Vector3i min = (model.inverse()*(B - Geometry::RP3Point(rc+1, rc+1, rc+1, 0))).head(3).cast<int>();
			Eigen::Vector3i max = (model.inverse()*(B + Geometry::RP3Point(rc+1, rc+1, rc+1, 0))).head(3).cast<int>();
			min = min.cwiseMax(Eigen::Vector3i::Zero());
			// max = max.cwiseMin(dim);
			if (max[0]>=dim[0]) max[0]=dim[0]-1;
			if (max[1]>=dim[1]) max[1]=dim[1]-1;
			if (max[2]>=dim[2]) max[2]=dim[2]-1;

			// And draw bead
			for (int z = min[2]; z<max[2]; z++)
				for (int y = min[1]; y<max[1]; y++)
					for (int x = min[0]; x<max[0]; x++)
					{
						Geometry::RP3Point X = model*Geometry::RP3Point(x+0.5, y+0.5, z+0.5, 1);
						Geometry::dehomogenize(X);
						double d = (X.head(3) - B.head(3)).norm();
						float opacity=0.0f;
						if      (d<r-0.25) opacity=1.0f;
						else if (d<r+0.5) opacity=(float)weighting((r-0.25-d)/0.75);
						phantom[x + y*phantom.size(0) + z*phantom.size(1)*phantom.size(0)]+=opacity;
					}
		}
		app.progressEnd();
		if (cancel_clicked)
			phantom.set(0x0,0);
		if (drr.raycaster) delete drr.raycaster;
		drr.raycaster=0x0;
		if (drr.voxel_data) delete drr.voxel_data;
		drr.voxel_data=0x0;
		if (!!phantom) drr.voxel_data=new VolumeRendering::VoxelData(phantom,false);
	}
	
	void BeadPhantomSimulator::load_volume(const NRRD::ImageView<float>& phantom)
	{
		if (drr.raycaster) delete drr.raycaster;
		drr.raycaster=0x0;
		if (drr.voxel_data) delete drr.voxel_data;
		drr.voxel_data=0x0;
		if (!!phantom) drr.voxel_data=new VolumeRendering::VoxelData(phantom,false);
	}
	
	void BeadPhantomSimulator::project(NRRD::ImageView<float>& out, const Geometry::ProjectionMatrix& P) const
	{
		// Out must be allocated
		if (out.dimension()!=2) out.set(0,0x0);
		// Must call voxelize before use.
		if (!drr.voxel_data) {
			out.set(0,0x0);
			return;
		}

		// Set up raycaster
		if (!drr.raycaster)
			drr.raycaster=new VolumeRendering::Raycaster(*drr.voxel_data);
		drr.raycaster->raycastPass<VolumeRendering::DigitallyReconstructedRadiograph>();//.setTransferFunction();
		drr.raycaster->setSamplesPerVoxel(1.5);

		// Allocate output image and raycast
		NRRD::Image<float> tmp(4,out.size(0), out.size(1));
		drr.raycaster->render(tmp,P);

		// Copy temporary image
		int l=out.length();
		for (int i=0;i<l;i++) out[i]=tmp[i*4];
		out.meta_info["Projection Matrix"]=toString(P);
	}

	void BeadPhantomSimulator::gui_declare_section (const GetSetGui::Section& section)
	{
		GetSet<Eigen::Vector3i              >("Voxelization/Voxel Number"  , section, voxelization.voxel_number  ).setDescription("Volume dimension in voxels.");
		GetSet<Eigen::Vector3d              >("Voxelization/Voxel Spacing" , section, voxelization.voxel_spacing ).setDescription("Spacing between voxels in mm.");
		GetSetGui::Section("Voxelization",section).setGrouped();
	}

	void BeadPhantomSimulator::gui_retreive_section(const GetSetGui::Section& section) 
	{
		voxelization.voxel_number    =GetSet<Eigen::Vector3i              >("Voxelization/Voxel Number"  ,section);
		voxelization.voxel_spacing   =GetSet<Eigen::Vector3d              >("Voxelization/Voxel Spacing" ,section);
	}

} // namespace Calibration
