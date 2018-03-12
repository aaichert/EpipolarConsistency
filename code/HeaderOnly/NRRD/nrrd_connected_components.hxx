// Created by A. Aichert on April 17th 2014

#ifndef __nrrd_connected_components_hxx
#define __nrrd_connected_components_hxx

#include "nrrd_image.hxx"

#include <algorithm>

namespace NRRD {

	struct ConnectedComponent {
		int aabb_min[3];
		int aabb_max[3];
		int size;
		double mass;
		double com[3];
	};

	// Analyze size and position of components
	template <typename T1>
	std::map<unsigned short, ConnectedComponent> analyzeCompoents(const NRRD::ImageView<T1>& image, const NRRD::ImageView<unsigned short>& components)
	{
		std::map<unsigned short, ConnectedComponent> ret;
		// Iterate over all voxels
		for (int z=0;z<image.size(2);z++)
			for (int y=0;y<image.size(1);y++)
				for (int x=0;x<image.size(0);x++)
				{
					int idx=x+y*image.size(0)+z*image.size(0)*image.size(1);
					unsigned short c=components[idx];
					if (c==0) continue;
					bool new_component=ret.find(c)==ret.end();
					ConnectedComponent &component(ret[c]);
					if (new_component)
					{
						component.aabb_min[0]=component.aabb_max[0]=x;
						component.aabb_min[1]=component.aabb_max[1]=y;
						component.aabb_min[2]=component.aabb_max[2]=z;
					}
					else
					{
						if (x<component.aabb_min[0]) component.aabb_min[0]=x;
						if (y<component.aabb_min[1]) component.aabb_min[1]=y;
						if (z<component.aabb_min[2]) component.aabb_min[2]=z;
						if (x>component.aabb_max[0]) component.aabb_max[0]=x;
						if (y>component.aabb_max[1]) component.aabb_max[1]=y;
						if (z>component.aabb_max[2]) component.aabb_max[2]=z;
					}
					double intensity=(double)image[idx];
					component.size++;
					component.mass+=intensity;
					component.com[0]+=x*intensity;
					component.com[1]+=y*intensity;
					component.com[2]+=z*intensity;
				}
		for (auto it=ret.begin();it!=ret.end();++it)
		{
			it->second.com[0]/=it->second.mass;
			it->second.com[1]/=it->second.mass;
			it->second.com[2]/=it->second.mass;
		}
		return ret;
	}

	// Compute connected component image. Zero is background, index of largest component is returned along with its size.
	template <typename T1>
	std::pair<int,int> connected_components_2D(
		const NRRD::ImageView<T1>& image, T1 activeLabel, bool negate,
		NRRD::ImageView<unsigned short>& components)
	{
		// Map from a label to a joint label
		std::vector<int> labelMap;
		labelMap.reserve(128);
		labelMap.push_back(0);

		// Set components to all inactive
		components.set(image.size(0),image.size(1));
		int l=components.length();
		#pragma omp parallel for
		for (int i=0;i<l;i++) components[i]=0;
		
		// Define neighborhood
		//struct Offset {
		//	Offset(int _x, int _y) : x(_x), y(_y) {}
		//	int x,y;
		//};
		//std::vector<Offset> neighborhood;
		//neighborhood.push_back(Offset( 0, 0));
		//neighborhood.push_back(Offset(-1, 0));
		//neighborhood.push_back(Offset( 0,-1));
		//neighborhood.push_back(Offset(-1,-1));
		// Iterate over all voxels
		for (int y=1;y<image.size(1)-1;y++)
			for (int x=1;x<image.size(0)-1;x++)
			{
				// Ignore inactive voxels
				const auto &intensity(image(x,y));
				if (!negate && (intensity!=activeLabel)) continue;
				if ( negate && (intensity==activeLabel)) continue;
				// Find lowest component in neighborhood
				int lowest_neighbor=0;
				//for (auto o=neighborhood.begin();o!=neighborhood.end();++o)
				#define FOR_2D_NEIGHBOR(OX,OY)                           \
				{                                                        \
					int l=components(x+OX,y+OY);                         \
					while (l!=labelMap[l]) l=labelMap[l];                \
					if (l!=0 && (l<lowest_neighbor||lowest_neighbor==0)) \
						lowest_neighbor=l;                               \
				}
				FOR_2D_NEIGHBOR( 0, 0);
				FOR_2D_NEIGHBOR(-1, 0);
				FOR_2D_NEIGHBOR( 0,-1);
				FOR_2D_NEIGHBOR(-1,-1);
				#undef FOR_2D_NEIGHBOR
				// If no other voxels in neighborhood are active, we find a new label
				if (lowest_neighbor==0)
				{
					int max=(int)labelMap.size();
					labelMap.push_back(max);
					components.pixel(x,y)=max;
				}
				else
				{
					// Get correct label for the join of corrected subcomponents
					while (lowest_neighbor!=labelMap[lowest_neighbor])
						lowest_neighbor=labelMap[lowest_neighbor];
					// We update the neighborhood to have the same index
					components.pixel(x,y)=lowest_neighbor;
				//	for (auto o=neighborhood.begin();o!=neighborhood.end();++o)
					#define FOR_2D_NEIGHBOR(OX,OY)                           \
					{                                                        \
						unsigned short& l=components.pixel(x+OX,y+OY);       \
						if (l!=0) l=labelMap[l]=lowest_neighbor;             \
					}
					FOR_2D_NEIGHBOR( 0, 0);
					FOR_2D_NEIGHBOR(-1, 0);
					FOR_2D_NEIGHBOR( 0,-1);
					FOR_2D_NEIGHBOR(-1,-1);
					#undef FOR_2D_NEIGHBOR
				}
			}
		// Finally, iterate over all voxels and update indices to lowest joint component label
		#pragma omp parallel for
		for (int i=0;i<l;i++)
		{
			int l0=components[i];
			int orig=l0;
			while (l0!=labelMap[l0]) l0=labelMap[l0]; // 2do max two lookups would be enough
			labelMap[orig]=l0;
			components[i]=l0;
		}
		// Determine size of components
		for (auto i=labelMap.begin();i!=labelMap.end();++i) *i=0;
		for (int i=0;i<l;i++) labelMap[components[i]]++;
		labelMap[0]=0; // ignore background
		// Return index and size of largest component
		int max_px=0;
		int max_idx=0;
		for (int i=0;i<(int)labelMap.size();i++)
			if (max_px<labelMap[i]) {
				max_idx=i;
				max_px=labelMap[i];
			}
		return std::pair<int,int>(max_idx,max_px);
	}

	// Compute connected component image. Zero is background, index of largest component is returned along with its size.
	template <typename T1>
	std::pair<int,int> connected_components_3D(
		const NRRD::ImageView<T1>& image, T1 activeLabel, bool negate,
		NRRD::ImageView<unsigned short>& components)
	{
		// Map from a label to a joint label
		std::map<int,int> labelMap;
		labelMap[0]=0;
		// Set components to all inactive
		int l=components.length();
		#pragma omp parallel for
		for (int i=0;i<l;i++) components[i]=0;
		
		// Define neighborhood
		struct Offset {
			Offset(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}
			int x,y,z;
		};
		std::vector<Offset> neighborhood;
		neighborhood.push_back(Offset(-1,0,0));
		neighborhood.push_back(Offset(0,-1,0));
		neighborhood.push_back(Offset(0,0,-1));
		neighborhood.push_back(Offset(-1,-1,0));
		neighborhood.push_back(Offset(-1,0,-1));
		neighborhood.push_back(Offset(0,-1,-1));
		neighborhood.push_back(Offset(-1,-1,-1));
		// Iterate over all voxels
		for (int z=1;z<image.size(2)-1;z++)
			for (int y=1;y<image.size(1)-1;y++)
				for (int x=1;x<image.size(0)-1;x++)
				{
					// Ignore inactive voxels
					const auto &intensity(image(x,y,z));
					if (!negate && (intensity!=activeLabel)) continue;
					if ( negate && (intensity==activeLabel)) continue;
					// Find lowest component in neighborhood
					int lowest_neighbor=0;
					for (auto o=neighborhood.begin();o!=neighborhood.end();++o)
					{
						int l=components(x+o->x,y+o->y,z+o->z);
						while (l!=labelMap[l]) l=labelMap[l];
						if (l!=0 && (l<lowest_neighbor||lowest_neighbor==0))
							lowest_neighbor=l;
					}
					// If no other voxels in neighborhood are active, we find a new label
					if (lowest_neighbor==0)
					{
						int max=0;
						for (auto i=labelMap.begin();i!=labelMap.end();++i)
							if (max<i->first) max=i->first; // labelMap.back().first ??
						max++;
						components.pixel(x,y,z)=labelMap[max]=max;
					}
					else
					{
						// Get correct label for the join of corrected subcomponents
						while (lowest_neighbor!=labelMap[lowest_neighbor])
							lowest_neighbor=labelMap[lowest_neighbor];
						// We update the neighborhood to have the same index
						components.pixel(x,y,z)=lowest_neighbor;
						for (auto o=neighborhood.begin();o!=neighborhood.end();++o) {
							unsigned short& l=components.pixel(x+o->x,y+o->y,z+o->z);
							if (l!=0) l=labelMap[l]=lowest_neighbor;
						}
					}
				}
		// Finally, iterate over all voxels and update indices to lowest joint component label
		for (int i=0;i<l;i++)
		{
			int l0=components[i];
			int orig=l0;
			while (l0!=labelMap[l0]) l0=labelMap[l0];
			labelMap[orig]=l0;
			components[i]=l0;
		}
		// Determine size of components
		for (auto i=labelMap.begin();i!=labelMap.end();++i) i->second=0;
		for (int i=0;i<l;i++) labelMap[components[i]]++;
		labelMap[0]=0; // ignore background
		// Return index and size of largest component
		int max_px=0;
		int max_idx=0;
		for (auto i=labelMap.begin();i!=labelMap.end();++i)
			if (max_px<i->second) {
				max_idx=i->first;
				max_px=i->second;
			}
		return std::pair<int,int>(max_idx,max_px);
	}

} // namespace NRRD

#endif // __nrrd_connected_components_hxx
