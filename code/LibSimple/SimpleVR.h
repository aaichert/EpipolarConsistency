#ifndef SIMPLE_VR
#define SIMPLE_VR

/** @brief	Simple OpenGL GLSL Volume Renderer
 *  @author	Andre Aichert
 */

#include "SimpleGL.h"
#include "SimpleRayCastPass.h"

namespace SimpleVR
{

	/// Render ray entry and exit textures for ray caster. Handles near and far plane clipping
	class ProxyGeomSetup
	{
	protected:
		/// an FBO for offscreen rendering
		SimpleGL::FramebufferObject *m_fbo;

	public:
		ProxyGeomSetup() : m_fbo(0x0) {}
		virtual ~ProxyGeomSetup() {}
		
		/// Set up
		virtual void prolog();
		/// Write ray geometry to front and back textures, core implementation.
		virtual void renderGeometry();
		/// Clean up
		virtual void epilog();

		/// Define intensity range guaranteed by empty space skipping for subclasses (unused in ProxyGeomSetup base). 
		virtual void setIntensityRange(float min=0, float max=1);

		/// Render ray geometry.
		virtual void render(SimpleGL::Texture& front,
							SimpleGL::Texture& back,
							SimpleGL::Texture& depth,
							MHD::DataInterface* volume);
	};

	class VolumeRenderer
	{
	public://protected:
		/// Pointer to an instance of ProxyGeomSetup
		ProxyGeomSetup*							m_rayGeom;
		/// Pointer to volume data RayCastPass
		RayCastPass*							m_rayCastPass;
		/// Pointer to volume data
		SimpleGL::Texture*						m_pVolume;
		/// True if m_pVolume points to anSimpleGL::Texture allocated by this instance
		bool									m_ownsVolume;

		/// Depth texture used for depth test during rednering front and back
		SimpleGL::Texture						m_depthTex;
		/// Ray Geometry texture: front (ray start)
		SimpleGL::Texture						m_frontTex;
		/// Ray Geometry texture: back (ray exit)
		SimpleGL::Texture						m_backTex;

		// Optional for geometry interaction

		/// A callback function to render opaque OpenGL geometry.
		void (*m_drawSceneFunc)();

		/// Back project depth texture and update entry/exit
		SimpleGL::GLSLProgram					m_rayGeomUpdate;
		/// Volume size in mm
		SimpleGL::Uniform<SimpleGL::Vector3f>	m_size;
		/// Depth Texture in m_rayGeomUpdate
		SimpleGL::Uniform<SimpleGL::Texture*>	m_depth;
		/// Uniform in m_rayGeomUpdate defining a 2D texture for ray entry positions
		SimpleGL::Uniform<SimpleGL::Texture*>	m_rayEntry;
		/// Uniform in m_rayGeomUpdate defining a 2D texture for ray exit positions
		SimpleGL::Uniform<SimpleGL::Texture*>	m_rayExit;
		/// 1: clip part in front of geometry, 0: occlusion with geometry, -1 clip part behind geometry
		SimpleGL::Uniform<int>					m_clipping;

		/// Updated ray entry or exit texture, depending on m_clipping
		SimpleGL::Texture						m_updateTex;
		/// Pointer to an rgba texture with the OpenGL scene without the volume (owned if m_drawSceneFunc!=0x0)
		SimpleGL::Texture*						m_geometryTex;
		/// Pointer to a depth texture used for occlusion or clipping (owned if m_drawSceneFunc!=0x0)
		SimpleGL::Texture*						m_geometryDepthTex;
		
		/// Framebuffer Object for offscreen rendering
		SimpleGL::FramebufferObject				*m_fbo;

		/// Render scene if we have a function to do so
		void drawScene();

		/// Handle occlusion or clipping with solid geometry (output written to target)
		void rayGeometryUpdate(SimpleGL::Texture& target);

		/// Draw scene if available and perform raycasting
		void raycast();

	public:
		VolumeRenderer();
		~VolumeRenderer();

		/// Allocated textures of specified size.
		virtual bool resize(int w, int h, SimpleGL::Texture* target=0x0, bool color=true);

		/// Initialize volume rendering. Loads volume and takes ownerwhip of rayGeom and rayCastPass.
		bool init(const std::string& mhdfile,
			SimpleVR::ProxyGeomSetup* rayGeom=0x0,
			SimpleVR::RayCastPass* rayCastPass=0x0);

		/// Initialize volume rendering with an existing 3D Texture
		bool init(SimpleGL::Texture* vol3d,
			SimpleVR::ProxyGeomSetup* rayGeom=0x0,
			SimpleVR::RayCastPass* rayCastPass=0x0);

		/// Change 3d texture of volume.
		void setVolume(SimpleGL::Texture* vol3d);
		/// Returns pointer to volume 3D texture
		SimpleGL::Texture* getVolume();
		/// Returns pointer to volume 3D texture. Const overload.
		const SimpleGL::Texture* getVolume() const;

		/// Set the ray cast pass that defines the output (DRR, ISO etc.)
		void setRayCastPass(RayCastPass* rcp);
		
		/// Render DVR. Call resize(...) at least once in advance. If target is not supplied, render to the framebuffer.
		virtual void render(SimpleGL::Texture* target=0x0);

		// Note to the caller of setDrawSceneFunc(...).
		// The callback function *render should use a few simple GL calls to render a polygonal solid object.
		// - It will be called to render to an FBO with correct projection and active modelview matrix for the volume
		// - It may not call glClear, glViewport, glMatrixMode among other functions
		// - It may not change the OpenGL state
		// - For complex scenes, please supply color and depth textures of your scene instead
		// 2do explain clipping a 'lil better :-)

		/// Function that draws some solid geometry to be correctly shown alongside the volume (geometry can also be used for clipping)
		void setDrawGeometryFunc(void (*render)(), int clipping=0);

		/// Set Scene color and depth textures. Plase make sure they have the dimensions specified by resize(...). Geometry can also be used for clipping.
		void setGeometryTextures(SimpleGL::Texture& color, SimpleGL::Texture& depth, int clipping=0);
	};

}

#endif
