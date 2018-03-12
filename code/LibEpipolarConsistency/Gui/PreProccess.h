// Created by A. Aichert on Tue Mar 21st 2017.
#ifndef __pre_process_h
#define __pre_process_h

#include <GetSet/GetSetObjects.h>
#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include <LibProjectiveGeometry/EigenToStr.hxx>

#include <NRRD/nrrd_image_view.hxx>

namespace EpipolarConsistency
{
	/// Pre-processing of X-ray projection images.
	struct PreProccess : public GetSetGui::Configurable {
	
		/// Modifying intensities (first normalize, then apply bias and scale, then logarithm)
		struct Intensity {
			bool   normalize        =false;
			double bias             =0.0;
			double scale            =1.0;
			bool   apply_log        =false;
		} intensity;

		/// Low-pass filter. Applied after intensity changes.
		struct Lowpass {
			double gaussian_sigma    =1.84;
			int    half_kernel_width =5;
		} lowpass;
	
		/// Modifying geometry
		struct ImageGeometry {
			bool   flip_u           =false;
			bool   flip_v           =false;
		} image_geometry;

		/// Masking and feathering image borders. Offsets are left, right, bottom, top.
		struct Border {
			Eigen::Vector4i zero=Eigen::Vector4i::Constant(1);
			Eigen::Vector4i feather=Eigen::Vector4i::Constant(16);
		} border;

		void gui_declare_section (const GetSetGui::Section& section);
		void gui_retreive_section(const GetSetGui::Section& section);

		void process(NRRD::ImageView<float>& image) const;

		void apply_weight_cos_principal_ray(NRRD::ImageView<float>& image, const Geometry::ProjectionMatrix& P) const;

	};

} // namespace EpipolarConsistency

#endif // __pre_process_h
