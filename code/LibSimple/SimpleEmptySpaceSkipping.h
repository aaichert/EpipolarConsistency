#ifndef SIMPLE_ESS
#define SIMPLE_ESS

/** @brief	Simple Empty Space Skipping for SimpleVR
 *  @author	Andre Aichert
 */

#include "SimpleVR.h"

#include <vector>

namespace SimpleVR
{

	/// Render ray entry and exit textures for ray caster. Subdivides cube and turns sub-cubes on and off.
	class ProxySubCubes: public ProxyGeomSetup
	{
		float	*vertices;	// x y z min max size is n_x+1 x n_y+1 x n_z+1
		std::vector<int> indices;
		float	range_min;
		float	range_max;
		int		n_x;
		int		n_y;
		int		n_z;
	public:
		ProxySubCubes(const MHD::Image& vol, int subsample=16)
			: ProxyGeomSetup() 
			, vertices(0x0)
			, indices(0x0)
		{
			range_min=range_max=0;
			// compute number of sub-cubdes
			n_x=vol.getData()->dim(0)/subsample;
			n_y=vol.getData()->dim(1)/subsample;
			n_z=vol.getData()->dim(2)/subsample;
			// loop over all vertices (there are n+1 vertices in each direction)
			vertices=new float[(n_x+1)*(n_y+1)*(n_z+1)*5];
			#pragma omp parallel for
			for (int z=0;z<n_z+1;z++)
				for (int y=0;y<n_y+1;y++)
					for (int x=0;x<n_x+1;x++)
					{
						// compute location 
						int idx=(n_x+1)*(n_y+1)*z+(n_x+1)*y+x;
						vertices[idx*5+0]=(float)x/n_x;
						vertices[idx*5+1]=(float)y/n_y;
						vertices[idx*5+2]=(float)z/n_z;
						// find min and max in subcube (for min corner of subcube only)
						if (x!=n_x&&y!=n_y&&z!=n_z)
						{
							unsigned char *data=(unsigned char *)vol.getData()->raw();
							unsigned char rmin=255;
							unsigned char rmax=0;
							for (int dz=-1;dz<subsample+1;dz++)
								for (int dy=-1;dy<subsample+1;dy++)
									for (int dx=-1;dx<subsample+1;dx++)
									{
										int vx=(subsample*x+dx);
										int vy=(subsample*y+dy);
										int vz=(subsample*z+dz);
										if (vx<0||vx>=vol.getData()->dim(0)) continue;
										if (vy<0||vy>=vol.getData()->dim(1)) continue;
										if (vz<0||vz>=vol.getData()->dim(2)) continue;
										unsigned char v=data[vz*vol.getData()->dim(0)*vol.getData()->dim(1)+vy*vol.getData()->dim(0)+vx];
										if (rmin>v) rmin=v;
										else if (rmax<v) rmax=v;
									}
							vertices[idx*5+3]=rmin/255.0f;
							vertices[idx*5+4]=rmax/255.0f;
						}
						else
						{
							// invalid because this vertex is not a min corner of any cube
							vertices[idx*5+3]=-1;
							vertices[idx*5+4]=-1;						
						}
					}
			std::cout << n_x*n_y*n_z << " cubes.\n";
		}
		
		~ProxySubCubes()
		{
			delete vertices;
		}

		virtual void prolog()
		{
			glPushAttrib(GL_ALL_ATTRIB_BITS);
			glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);
			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_COLOR_ARRAY);
			glVertexPointer(3,GL_FLOAT,5*sizeof(float),vertices);
			glColorPointer(3,GL_FLOAT,5*sizeof(float),vertices);
		}

		virtual void renderGeometry()
		{
			if (!indices.empty())
				glDrawElements(GL_QUADS,(int)indices.size(),GL_UNSIGNED_INT,&indices[0]);
		}

		virtual void epilog()
		{
			glPopClientAttrib();
			glPopAttrib();
		}

		/// Define intensity range guaranteed by empty space skipping
		virtual void setIntensityRange(float min=0, float max=1)
		{
			if (range_min==min && range_max==max)
				return; // nothing to be done
			indices.clear();
			range_min=min;
			range_max=max;
			std::cout << "Intensity range (normalized) " << min << " to " << max << std::endl;
			unsigned char *active=new unsigned char [n_x*n_y*n_z];
			int n_active=0;
			for (int z=0;z<n_z;z++)
				for (int y=0;y<n_y;y++)
					for (int x=0;x<n_x;x++)
					{
						int idx_vertices=z*(n_x+1)*(n_y+1)+y*(n_x+1)+x;
						int idx_active=z*n_x*n_y+y*n_x+x;
						float range_cube_min=vertices[5*idx_vertices+3];
						float range_cube_max=vertices[5*idx_vertices+4];
						if (range_cube_max<min||range_cube_min>max)
							active[idx_active]=0;
						else
						{
							active[idx_active]=1;
							n_active++;
						}
					}
			std::cout << n_active << " active cells.\n";
			// Check for active faces and add face indices
			for (int z=0;z<n_z;z++)
				for (int y=0;y<n_y;y++)
					for (int x=0;x<n_x;x++)
					{
						int idx_vertices=z*(n_x+1)*(n_y+1)+y*(n_x+1)+x;
						int idx_active=z*n_x*n_y+y*n_x+x;
						if (!active[idx_active]) continue;
						if (x==0||active[z*n_x*n_y+y*n_x+x-1]==0)
						{
							indices.push_back((z+0)*(n_x+1)*(n_y+1)+(y+0)*(n_x+1)+(x+0));
							indices.push_back((z+1)*(n_x+1)*(n_y+1)+(y+0)*(n_x+1)+(x+0));
							indices.push_back((z+1)*(n_x+1)*(n_y+1)+(y+1)*(n_x+1)+(x+0));
							indices.push_back((z+0)*(n_x+1)*(n_y+1)+(y+1)*(n_x+1)+(x+0));
						}
						if (x==n_x-1||active[z*n_x*n_y+y*n_x+x+1]==0)
						{
							indices.push_back((z+0)*(n_x+1)*(n_y+1)+(y+0)*(n_x+1)+(x+1));
							indices.push_back((z+1)*(n_x+1)*(n_y+1)+(y+0)*(n_x+1)+(x+1));
							indices.push_back((z+1)*(n_x+1)*(n_y+1)+(y+1)*(n_x+1)+(x+1));
							indices.push_back((z+0)*(n_x+1)*(n_y+1)+(y+1)*(n_x+1)+(x+1));
						}
						if (y==0||active[z*n_x*n_y+(y-1)*n_x+x]==0)
						{
							indices.push_back((z+0)*(n_x+1)*(n_y+1)+(y+0)*(n_x+1)+(x+0));
							indices.push_back((z+0)*(n_x+1)*(n_y+1)+(y+0)*(n_x+1)+(x+1));
							indices.push_back((z+1)*(n_x+1)*(n_y+1)+(y+0)*(n_x+1)+(x+1));
							indices.push_back((z+1)*(n_x+1)*(n_y+1)+(y+0)*(n_x+1)+(x+0));

						}
						if (y==n_y-1||active[z*n_x*n_y+(y+1)*n_x+x]==0)
						{
							indices.push_back((z+0)*(n_x+1)*(n_y+1)+(y+1)*(n_x+1)+(x+0));
							indices.push_back((z+0)*(n_x+1)*(n_y+1)+(y+1)*(n_x+1)+(x+1));
							indices.push_back((z+1)*(n_x+1)*(n_y+1)+(y+1)*(n_x+1)+(x+1));
							indices.push_back((z+1)*(n_x+1)*(n_y+1)+(y+1)*(n_x+1)+(x+0));
						}
						if (z==0||active[(z-1)*n_x*n_y+y*n_x+x]==0)
						{
							indices.push_back((z+0)*(n_x+1)*(n_y+1)+(y+0)*(n_x+1)+(x+1));
							indices.push_back((z+0)*(n_x+1)*(n_y+1)+(y+1)*(n_x+1)+(x+1));
							indices.push_back((z+0)*(n_x+1)*(n_y+1)+(y+1)*(n_x+1)+(x+0));
							indices.push_back((z+0)*(n_x+1)*(n_y+1)+(y+0)*(n_x+1)+(x+0));

						}
						if (z==n_z-1||active[(z+1)*n_x*n_y+y*n_x+x]==0)
						{
							indices.push_back((z+1)*(n_x+1)*(n_y+1)+(y+0)*(n_x+1)+(x+1));
							indices.push_back((z+1)*(n_x+1)*(n_y+1)+(y+1)*(n_x+1)+(x+1));
							indices.push_back((z+1)*(n_x+1)*(n_y+1)+(y+1)*(n_x+1)+(x+0));
							indices.push_back((z+1)*(n_x+1)*(n_y+1)+(y+0)*(n_x+1)+(x+0));
						}
					}
			std::cout << indices.size() << " active faces.\n";
			delete active;
		}

	};
	
} // namespace SimpleVR

#endif // SIMPLE_ESS
