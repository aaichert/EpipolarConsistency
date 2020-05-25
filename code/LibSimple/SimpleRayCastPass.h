#ifndef SIMPLE_RAYCAST_PASS
#define SIMPLE_RAYCAST_PASS

#include "SimpleGL.h"

#define CREATE_RAY_SHADER																			\
	"// Created by Andre Aichert, derived from endai\n"												\
	"uniform sampler3D  volume;           // volume texture\n"										\
	"uniform vec3       numberOfVoxels;   // number of voxels of volume\n"							\
	"uniform vec3       elementSpacing;   // spacing [mm] between voxels\n"							\
	"uniform float      samplesPerVoxel;  // approximate distance between samples\n"				\
	"uniform sampler2D  rayEntry;         // front geometry texture\n"								\
	"uniform sampler2D  rayExit;          // back geometry texture\n"								\
	"\n"																							\
	"struct Ray {                         // all in tex coords\n"									\
	"   vec3    start;                    // entry point to volume\n"								\
	"   vec3    end;                      // exit point\n"											\
	"   vec3    dir;                      // ray direction (scaled to match samplesPerVoxel)\n"		\
	"   float   adv;                      // adv=1.0/#samples\n"									\
	"   float   samplesPerMM;             // Samples per millimeter\n"								\
	"};\n"																							\
	"\n"																							\
	"Ray createRay()\n"																				\
	"{\n"																							\
	"   Ray ray;\n"																					\
	"   ray.start=texture2D(rayEntry,gl_TexCoord[0].st).xyz;\n"										\
	"	//ray.start+=ray.dir*sin(1000.0*gl_TexCoord[0].s+1000.0*gl_TexCoord[0].t);\n"				\
	"   ray.end=texture2D(rayExit,gl_TexCoord[0].st).xyz;\n"										\
	"   ray.dir=ray.end-ray.start;\n"																\
	"   ray.adv=1.0/(length(ray.dir*numberOfVoxels)*samplesPerVoxel);\n"							\
	"   ray.samplesPerMM=samplesPerVoxel/length(normalize(ray.dir)*elementSpacing);\n"							\
	"   ray.dir*=ray.adv;\n"																		\
	"   if (ray.adv<0.001) ray.adv=1.0; // ignore rays with more that 1000 samples\n"				\
	"   return ray;\n"																				\
	"}\n"																							\
	"\n"

#define GRADIENT_SHADER																				\
	"vec3 gradient(in vec3 pos)\n"																	\
	"{\n"																							\
	"   vec4 texStep=vec4(vec3(1.0)/numberOfVoxels,0.0);\n"											\
	"   vec3 n=vec3(texture3D(volume,pos-texStep.xww).x-texture3D(volume,pos+texStep.xww).x,\n"		\
	"               texture3D(volume,pos-texStep.wyw).x-texture3D(volume,pos+texStep.wyw).x,\n"		\
	"               texture3D(volume,pos-texStep.wwz).x-texture3D(volume,pos+texStep.wwz).x);\n"	\
	"   n/=elementSpacing;\n"																		\
	"   return n;\n"																				\
	"}\n"																							\
	"\n"																							\
	"// Lambertian shading\n"																		\
	"float shade(vec3 surfacenormal, vec3 raydirection)\n"											\
	"{\n"																							\
	"   return abs(dot(normalize(surfacenormal),normalize(raydirection)));\n"						\
	"}\n"																							\
	"\n"

namespace SimpleVR
{
	/// Interface for all DVR visualization methods (ISO,MIP,1DTF,pre-integrated etc. etc.)
	class RayCastPass
	{
	protected:
		/// GLSL program with pass-specific fragment shader
		SimpleGL::GLSLProgram					m_pass;

		/// Uniform in m_pass defining size of volume in voxels. Same as volume.dim(0-2).
		SimpleGL::Uniform<SimpleGL::Vector3f>	numberOfVoxels;
		/// Uniform in m_pass defining physical of one voxel in mm. Same as volume.spacing(0-2)
		SimpleGL::Uniform<SimpleGL::Vector3f>	elementSpacing;

	public:
		/// Uniform in m_pass defining a 3D texture used as volume.
		SimpleGL::Uniform<SimpleGL::Texture*>	volume;
		/// Uniform in m_pass defining a 2D texture for ray entry positions
		SimpleGL::Uniform<SimpleGL::Texture*>	rayEntry;
		/// Uniform in m_pass defining a 2D texture for ray exit positions
		SimpleGL::Uniform<SimpleGL::Texture*>	rayExit;
		/// Uniform in m_pass defining the distance between samples along ray.
		SimpleGL::Uniform<float>				samplesPerVoxel;

		/// Create m_pass from GLSL fragment shader source
		RayCastPass(const char* source);

		/// Set 3d volume texture to be rendered
		virtual void setVolume(SimpleGL::Texture& volumeTex);

		/// Set textures (as created by ProxyGeomSetup)
		virtual void setTextrues(SimpleGL::Texture& rayEntryTex, SimpleGL::Texture& rayExitTex);

		/// Define the distance between samples along ray. Same as samplesPerVoxel=spv;
		virtual void setSamplesPerVoxel(float spv=1.5);

		/// Renders volume using rayEntry and rayExit as provided by setTextures(...). Spawns one ray per viewport pixel.
		virtual void render();

		/// Returns minimum intensity of interest to this pass (eg. TF min or iso value)
		virtual float intensityMin() {return 0;}
		/// Returns maximum intensity of interest to this pass (eg. TF max or iso value)
		virtual float intensityMax() {return 1;}
	};

	/// Implements a simple pass for debugging
	class DebugRaycastEnvironment : public RayCastPass
	{
	public:
		DebugRaycastEnvironment();

		/// Dummy value to test ESS
		virtual float intensityMin() {return 0.5;}
	};

	/// Implements maximum intensity projection (result = maximum along ray)
	class MaximumIntensityProjection : public RayCastPass
	{
	public:
		MaximumIntensityProjection();
	};

	/// Implements artificial X-Ray. Ramp transfer function.
	class IsoSurface : public RayCastPass
	{
	public:
		/// Construction. If no source is provided, x/y/z of hit position will be encoded.
		IsoSurface(const char* crScr=0x0);

		/// If true, iso surface will be phong shaded
		SimpleGL::Uniform<bool> shaded;

		/// Iso Value
		SimpleGL::Uniform<float> isoValue;

		/// Iso-value to max
		virtual float intensityMin() {return isoValue;}

	};

	/// Implements artificial X-Ray. Ramp transfer function.
	class DigitallyReconstructedRadiograph : public RayCastPass
	{
	public:
		DigitallyReconstructedRadiograph();

		/// Tranfer function (simple ramp: linear from zero at min to height at max, then constant)
		SimpleGL::Uniform<float> rampMin;
		SimpleGL::Uniform<float> rampMax;
		SimpleGL::Uniform<float> rampHeight;
	
		/// rampMin to infty
		virtual float intensityMin() {return rampMin;}
	};

#undef near
#undef far

	/// Implements depth-colored emission pass
	class ColorEmissionByDepth : public RayCastPass
	{
	public:
		ColorEmissionByDepth();

		/// Tranfer function for amount of emission (simple ramp)
		SimpleGL::Uniform<float> rampMin;
		SimpleGL::Uniform<float> rampMax;
		SimpleGL::Uniform<float> rampHeight;

		/// Color transfer function (RGB, 4th component is relative depth)
		SimpleGL::Uniform<SimpleGL::Vector4f>	near;
		SimpleGL::Uniform<SimpleGL::Vector4f>	center;
		SimpleGL::Uniform<SimpleGL::Vector4f>	far;

		/// rampMin to infty
		virtual float intensityMin() {return rampMin;}
	};
	
} // namespace Simple

#endif // SIMPLE_RAYCAST_PASS
