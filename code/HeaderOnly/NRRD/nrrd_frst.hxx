#ifndef __nrrd_frst_hxx
#define __nrrd_frst_hxx

#include <NRRD/nrrd_image.hxx>
#include <NRRD/nrrd_lowpass.hxx>

namespace NRRD {

	// img must be dimension 2, radii is a list of radii of interest, radial_strictness is the fall-off.
	bool frst(NRRD::ImageView<float>& img, const std::vector<double>& radii, double sigma = 3.68, int k= 5, double radial_strictness =1, double threshold_gradient=0)
	{
		// Make sure we have a 2D image and get its size
		 if (img.dimension()!=2 || radii.empty())
			 return false;
		int l=img.length();
		int n_x=img.size(0);
		int n_y=img.size(1);
		// Allocate temporary images
		NRRD::Image<float> On(n_x,n_y),Mn(n_x,n_y);
		#pragma omp parallel for schedule(dynamic, 128)
		for (int i=0;i<l;i++)
			Mn[i]=On[i]=0;
		// Iterate over image and increase p+ve and decrease p-ve
		#pragma omp parallel for schedule(static, 48) // Potential Bug: To avoid scheduling conflicts, radii must not contain values larger than 24.
		for (int y=1;y<n_y-1;y+=1)
		{
			for (int x=1;x<n_x-1;x+=1)
			{
				double dx=img(x+1,y)-img(x-1,y);
				double dy=img(x,y+1)-img(x,y-1);
				double norm_grad=std::sqrt(dx*dx+dy*dy);
				if (norm_grad<threshold_gradient) continue;
				dx/=norm_grad;
				dy/=norm_grad;
				for (auto it=radii.begin();it!=radii.end();++it)
				{
					const double& radius=*it;
					int px=x-(int)(dx*radius+0.5);
					int py=y-(int)(dy*radius+0.5);
					int mx=x+(int)(dx*radius+0.5);
					int my=y+(int)(dy*radius+0.5);
					if (px>=0&&py>=0&&px<n_x&&py<n_y)
					{
						On.pixel(px,py)++;
						Mn.pixel(px,py)+=(float)norm_grad;
					}
					if (mx>=0&&my>=0&&mx<n_x&&my<n_y)
					{
						On.pixel(mx,my)--;
						Mn.pixel(mx,my)-=(float)norm_grad;
					}
				}
			}
		}

		// max-reduction for normalization
		float max_o=On[0];
		float max_m=Mn[0];
		#pragma omp parallel for
		for (int i=0;i<l;i++)
		{
			#pragma omp critical
			if (max_o<On[i])max_o=On[i];
			#pragma omp critical
			if (max_m<Mn[i])max_m=Mn[i];
		}
		// Normalize
		#pragma omp parallel for schedule(dynamic, 128)
		for (int i=0;i<l;i++)
		{
			On[i]/=max_o;
			Mn[i]/=max_m;
		}
		// Compute FRST score
		#pragma omp parallel for schedule(dynamic, 128)
		for (int i=0;i<l;i++)
			img[i]=std::pow(On[i],(float)radial_strictness)*Mn[i];
		// Final convolution
		auto kernel=NRRD::gaussianKernel(sigma, k);
		NRRD::convolve2D(img,k,k,kernel,kernel,On);
		return true;
	}


} // namespace NRRD

#endif // __nrrd_frst_hxx
