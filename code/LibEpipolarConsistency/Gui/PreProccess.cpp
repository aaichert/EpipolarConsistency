#include "PreProccess.h"
#include <NRRD/nrrd_lowpass.hxx>

#include <random>


/// f(x)=1-x^2+x^4 is zero at +/-1, has zero derivative at +/-1 and a maxiumum at f(0)=1; Values outside [-1,1] are clamped to zero. 
inline double weighting(double x)
{
	if (x<-1.0||x>1.0) return 0;
	double xx=x*x;
	return 1.0-2*xx+xx*xx;
}

namespace EpipolarConsistency
{

	void PreProccess::gui_declare_section(const GetSetGui::Section& section)
	{
		GetSet<bool>           ("Intensity/Normalize"             , section, intensity.normalize      );
		GetSet<double>         ("Intensity/Bias"                  , section, intensity.bias           );
		GetSet<double>         ("Intensity/Scale"                 , section, intensity.scale          );
		GetSet<bool>           ("Intensity/Apply Minus Logarithm" , section, intensity.apply_log      );
		GetSet<double>         ("Lowpass Filter/Gaussian Sigma"   , section, lowpass.gaussian_sigma   );
		GetSet<int>            ("Lowpass Filter/Half Kernel Width", section, lowpass.half_kernel_width);
		GetSet<bool>           ("Geometry/Flip u-Axis"            , section, image_geometry.flip_u    );
		GetSet<bool>           ("Geometry/Flip v-Axis"            , section, image_geometry.flip_v    );
		GetSet<Eigen::Vector4i>("Border/Zero Border"              , section, border.zero              );
		GetSet<Eigen::Vector4i>("Border/Feather"                  , section, border.feather           );
		GetSet<std::vector<Eigen::Vector4i> >("Border/Blanks"     , section, border.blanks            );
		section.subsection("Intensity"      ).setCollapsed();
		section.subsection("Lowpass Filter" ).setGrouped();
		section.subsection("Geometry"       ).setCollapsed();
		section.subsection("Border"         ).setGrouped();
		GetSetGui::StaticText  ("Intensity/_info"                , section)="Simple conversion from X-ray intenstiy to line-integral data.\nThese pixel-wise operations are applied as the first step.";
		GetSetGui::StaticText  ("Lowpass Filter/_info"           , section)="Avoid sampling artifacts and reduce noise.";
		GetSetGui::StaticText  ("Geometry/_info"                 , section)="Image is flipped as a last step\n(before Gaussian convolution).";
		GetSetGui::StaticText  ("Border/_info"                   , section)="Remove strong edges close to the image borders, in case of collimation, truncation etc.";

	}
	
	void PreProccess::gui_retreive_section(const GetSetGui::Section& section)
	{
		intensity.normalize      =GetSet<bool>           ("Intensity/Normalize"             ,section);
		intensity.bias           =GetSet<double>         ("Intensity/Bias"                  ,section);
		intensity.scale          =GetSet<double>         ("Intensity/Scale"                 ,section);
		intensity.apply_log      =GetSet<bool>           ("Intensity/Apply Minus Logarithm" ,section);
		lowpass.gaussian_sigma   =GetSet<double>         ("Lowpass Filter/Gaussian Sigma"   ,section);
		lowpass.half_kernel_width=GetSet<int>            ("Lowpass Filter/Half Kernel Width",section);
		image_geometry.flip_u    =GetSet<bool>           ("Geometry/Flip u-Axis"            ,section);
		image_geometry.flip_v    =GetSet<bool>           ("Geometry/Flip v-Axis"            ,section);
		border.zero              =GetSet<Eigen::Vector4i>("Border/Zero Border"              ,section);
		border.feather           =GetSet<Eigen::Vector4i>("Border/Feather"                  ,section);
		border.blanks            =GetSet<std::vector<Eigen::Vector4i> >("Border/Blanks"     ,section);
	}

	void PreProccess::process(NRRD::ImageView<float>& img) const 
	{

		// Apply Preprocessing/Intensity Bias, scale and logarithm per pixel (-> ray integrals)
		int l = img.length();

		float scale=(float)intensity.scale;
		float bias =(float)intensity.bias;

		// Find maximum value
		if (intensity.normalize)
		{
			float max = img[0];
			for (int i = 0; i<l; i++)
				if (img[i]>max) max = img[i];
			std::cout << "Normalizing with maximum intenisty: " << max << std::endl;
			bias = 0;
			float scale_max = (float)intensity.scale;
			scale = scale_max / max;
		}

		// Apply bias scale
		#pragma omp parallel for
		for (int i = 0; i < l; i++)
		{
			auto& pixel=img[i];
			pixel=pixel*scale+bias;
			if (intensity.apply_log)
				pixel = (float)-std::log(pixel);
			if (pixel<0 || std::isnan(pixel) || std::isinf(pixel))
				pixel = 0;
		}

		// Border.
		// left
		#pragma omp parallel for
		for (int y = 0; y < img.size(1); y++)
			for (int b = 0; b < border.zero[0] + border.feather[0]; b++)
				img.pixel(b, y, 0) *= b <= border.zero[0] ? 0 : (float)weighting(1 - (float)(b - border.zero[0]) / border.feather[0]);
		// right
		#pragma omp parallel for
		for (int y = 0; y < img.size(1); y++)
			for (int b = 1; b <= border.zero[1] + border.feather[1]; b++)
				img.pixel(img.size(0) - b, y, 0) *= b <= border.zero[1] ? 0 : (float)weighting(1 - (float)(b - border.zero[1]) / border.feather[1]);
		// bottom
		#pragma omp parallel for
		for (int b = 1; b <= border.zero[2] + border.feather[2]; b++)
			for (int x = 0; x < img.size(0); x++)
				img.pixel(x, img.size(1) - b, 0) *= b <= border.zero[2] ? 0 : (float)weighting(1 - (float)(b - border.zero[2]) / border.feather[2]);
		// top
		#pragma omp parallel for
		for (int b = 0; b < border.zero[3] + border.feather[3]; b++)
			for (int x = 0; x < img.size(0); x++)
				img.pixel(x, b, 0) *= b <= border.zero[3] ? 0 : (float)weighting(1 - (float)(b - border.zero[3]) / border.feather[3]);

		// blanks
		for (auto blank=border.blanks.begin(); blank!=border.blanks.end();++blank)
			for (int y = std::max(0,(*blank)[1]); y < img.size(0) && y < (*blank)[3]; y++)
				for (int x = std::max(0,(*blank)[0]); x < img.size(0) && x < (*blank)[2]; x++)
					img.pixel(x, y, 0) = 0;

		// Flip u/v images
		int  w = img.size(0);
		int  h = img.size(1);
		if (image_geometry.flip_u)
		{
			for (int y = 0; y < h; y++)
				for (int x = 0; x < w / 2; x++)
					std::swap(img.pixel(x, y), img.pixel(w - 1 - x, y));
		}
		if (image_geometry.flip_v)
		{
			for (int y = 0; y < h / 2; y++)
				for (int x = 0; x < w; x++)
					std::swap(img.pixel(x, y), img.pixel(x, h-1-y));
		}
		
		//// Apply Gaussian noise (experiments only)
		//std::default_random_engine generator;
		//std::normal_distribution<double> dist(0, 0.1);
		//for (int i = 0; i < l; i++)
		//	img[i]+=dist(generator);

		// Apply low-pass
		if (lowpass.gaussian_sigma>0 && lowpass.half_kernel_width>1)
			NRRD::lowpass2D(img, lowpass.gaussian_sigma, lowpass.half_kernel_width);

	}

	void PreProccess::apply_weight_cos_principal_ray(NRRD::ImageView<float>& image, const Geometry::ProjectionMatrix& P) const
	{
		image.meta_info["Projection Matrix"]=toString(P);
		if (P.isZero()) return;
		int n_u=image.size(0);
		int n_v=image.size(1);
		auto K=Geometry::getCameraIntrinsics(P);
		float sdd_px=(float)K(0,0);
		float ppu   =(float)K(0,2);
		float ppv   =(float)K(1,2);
#pragma omp parallel for
		for (int v=0;v<n_v;v++)
			for (int u=0;u<n_u;u++)
			{
				auto &pixel=image.pixel(u,v);
				float pou=(float)u-ppu;
				float pov=(float)v-ppv;
				float cos_weight=sdd_px/sqrtf(pou*pou+pov*pov+sdd_px*sdd_px);
				pixel*=cos_weight;
			}
	}

} // namespace EpipolarConsistency
