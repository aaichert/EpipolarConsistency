#include "SimpleVR.h"

namespace SimpleVR
{

	const char *g_vertexPassThrough= // FIXME should be a func in SimpleGL::GLSLProgram
		"void main()\n"
		"{\n"
		"	gl_TexCoord[0]=gl_MultiTexCoord0;\n"
		"	gl_Position=gl_Vertex;\n"
		"}\n"
		;
		
	const char* geomClipPlanes= // FIXME modelview incorrect?
		"uniform float	plane;\n"
		"uniform vec3	size;"
		"void main() {\n"
		"	vec4 clip=gl_ModelViewProjectionMatrixInverse*vec4(gl_TexCoord[0].st*2.0-1.0,plane,1.0);\n"
		"	clip/=vec4(size,clip.w);\n"
		"	gl_FragData[0]=clip;\n"
		"}\n"
		;
		
	const char* geomUpdateSource=
		"#version 120\n"
		"// Created by Andre Aichert, derived from endai\n"
		"uniform sampler2D  rayEntry;   // front geometry texture\n"
		"uniform sampler2D  rayExit;    // back geometry texture\n"
		"uniform sampler2D  depth;		// opaque geometry depth\n"
		"uniform vec3		size;		// physical volume size [mm]\n"
		"uniform int		clip;		// 1: cliping 0 : occlusions\n"
		"void main()\n"
		"{\n"
		// ray projection for comparisons (is geometry within ray volume)
		"	vec4 start=texture2D(rayEntry,gl_TexCoord[0].st);\n"
		"	vec4 startclip=gl_ModelViewProjectionMatrix*(start*vec4(size,1.0));\n"
		"	startclip/=startclip.w;\n"
		"	vec4 end=texture2D(rayExit,gl_TexCoord[0].st);\n"
		"	vec4 endclip=gl_ModelViewProjectionMatrix*(end*vec4(size,1.0));\n"
		"	endclip/=endclip.w;\n"
		// scene back projection
		"	float sceneclipz=(texture2D(depth,gl_TexCoord[0].st).r-gl_DepthRange.near)*(2.0/gl_DepthRange.diff)-1;\n"
		"	vec4 scene=gl_ModelViewProjectionMatrixInverse*vec4(gl_TexCoord[0].st*2-1,sceneclipz,1.0);\n"
		"	scene/=scene.w;\n"
		"	scene/=vec4(size,1.0);\n"
		// choose face to update
		"	if (clip<=0) // clip volume occluded by geometry\n"
		"	{\n"
		"		if (sceneclipz<endclip.z) // in front of end\n"
		"		{\n"
		"			if (sceneclipz<startclip.z) // in front of start => not within volume\n"
		"				gl_FragData[0]=start;\n"
		"			else\n"
		"				gl_FragData[0]=scene;\n"
		"			return;\n"
		"		}\n"
		"		else\n"
		"		{\n"
		"			if (clip==0)\n"
		"				gl_FragData[0]=end;\n"
		"			else\n"
		"				gl_FragData[0]=start; // discards rays that are not being clipped\n"
		"		}\n"
		"	}\n"
		"	else // clip volume in front of geometry\n"
		"	{\n"
		"		if (sceneclipz>startclip.z) // behind start \n"
		"		{\n"
		"			if (sceneclipz>endclip.z) // behind end => not within volume\n"
		"				gl_FragData[0]=end;\n"
		"			else\n"
		"				gl_FragData[0]=scene;\n"
		"			return;\n"
		"		}\n"
		"		else\n"
		"			gl_FragData[0]=start;\n"
		"	}\n"
		"}\n"
		;
	/*
	static SimpleGL::GLSLProgram colorCodePlane;
	static SimpleGL::Uniform<float> plane=SimpleGL::Uniform<float>("plane",colorCodePlane);
	static SimpleGL::Uniform<SimpleGL::Vector3f> size=SimpleGL::Uniform<SimpleGL::Vector3f>("size",colorCodePlane);
	if (!colorCodePlane.isValid())
		colorCodePlane.compile(geomClipPlanes.c_str(),geomPassThroughSource.c_str());
	SimpleGL::Vector3f v;
	for (int i=0;i<3;i++) v[i]=volume->physicalSize(i);
	size=v;
	*/

	void ProxyGeomSetup::setIntensityRange(float min, float max) {}

	void ProxyGeomSetup::prolog()
	{
		static float corners[]={0,0,0, 1,0,0, 1,1,0, 0,1,0, 0,0,1, 1,0,1, 1,1,1, 0,1,1 };
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		glVertexPointer(3,GL_FLOAT,0,corners);
		glColorPointer(3,GL_FLOAT,0,corners);
	}

	void ProxyGeomSetup::renderGeometry()
	{
		static unsigned short indices[]={0,1,2,3, 7,6,5,4, 1,5,6,2, 3,7,4,0, 3,2,6,7, 4,5,1,0};
		glDrawElements(GL_QUADS,24,GL_UNSIGNED_SHORT,indices);
	}

	void ProxyGeomSetup::epilog()
	{
		glPopClientAttrib();
		glPopAttrib();
	}

	void ProxyGeomSetup::render(SimpleGL::Texture& front,
								SimpleGL::Texture& back,
								SimpleGL::Texture& depth,
								MHD::DataInterface* volume)
	{
		if (!m_fbo) m_fbo=new SimpleGL::FramebufferObject();
		CHECK_GL_ERROR
		prolog();
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glScaled(volume->physicalSize(0), volume->physicalSize(1), volume->physicalSize(2));
		{
			SimpleGL::RenderToTexture rtt(front, depth, m_fbo,0);
			glClearColor(0,0,0,0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glDisable(GL_BLEND);
			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LEQUAL);
			renderGeometry();
		}
		{
			SimpleGL::RenderToTexture rtt(back, depth, m_fbo,0);
			glClear(GL_COLOR_BUFFER_BIT);
			glDepthFunc(GL_GREATER);
			renderGeometry();
		}
		glDepthFunc(GL_LEQUAL);
		glPopMatrix();
		epilog();
		CHECK_GL_ERROR
	}

	VolumeRenderer::VolumeRenderer()
		: m_rayGeom(0x0)
		, m_rayCastPass(0x0)
		, m_pVolume(0x0)
		, m_ownsVolume(0x0)
		, m_drawSceneFunc(0x0)
		, m_depth("depth",m_rayGeomUpdate)
		, m_rayEntry("rayEntry",m_rayGeomUpdate)
		, m_rayExit("rayExit",m_rayGeomUpdate)
		, m_size("size",m_rayGeomUpdate)
		, m_clipping("clip",m_rayGeomUpdate)
		, m_geometryTex(0x0)
		, m_geometryDepthTex(0x0)
		, m_fbo(0x0)
	{}

	VolumeRenderer::~VolumeRenderer()
	{
		if (m_pVolume&&m_ownsVolume)
			delete m_pVolume;
		m_pVolume=0x0;
		if (m_drawSceneFunc)
		{
			if (m_geometryTex) delete m_geometryTex;
			if (m_geometryDepthTex) delete m_geometryDepthTex;
			m_geometryTex=0x0;
			m_geometryDepthTex=0x0;
			m_drawSceneFunc=0x0;
		}
	}

	template <class Tex> inline bool needsResize(Tex& t, int w, int h)
	{
		return !(t.getData() && t.getData()->dim(0)==w && t.getData()->dim(1)==h);
	}

	bool VolumeRenderer::resize(int w, int h, SimpleGL::Texture* target, bool color)
	{
		CHECK_GL_ERROR
		bool any=false;
		if (any|=needsResize(m_updateTex,w,h)) m_updateTex.createEmptyTexture<float>(w,h,1,3);
		if (any|=needsResize(m_depthTex,w,h)) m_depthTex.createEmptyTexture<float>(w,h,1,1,GL_DEPTH_COMPONENT, GL_DEPTH_COMPONENT24);
		if (any|=needsResize(m_frontTex,w,h) || needsResize(m_backTex,w,h))
		{
			m_frontTex.createEmptyTexture<float>(w,h,1,3);
			m_backTex.createEmptyTexture<float>(w,h,1,3);
			if (m_rayCastPass) m_rayCastPass->setTextrues(m_frontTex, m_backTex);
			if (m_fbo) delete m_fbo;
			m_fbo=new SimpleGL::FramebufferObject();
		}
		if (any|=target && needsResize(*target,w,h)) target->createEmptyTexture<float>(w,h,1,color?4:1);
		return any;
	}

	bool VolumeRenderer::init(const std::string& mhdfile, SimpleVR::ProxyGeomSetup* rayGeom, SimpleVR::RayCastPass* rayCastPass)
	{
		// Create new volume
		SimpleGL::Texture* vol=new SimpleGL::Texture();
		std::cout << "Loading Volume.\n";
		if (!vol->loadMHD(mhdfile))
		{
			delete vol;
			vol=0x0;
			return false;
		}
		std::cout << "Creating 3D texture.\n";
		vol->createTexture();
		bool ret=init(vol,rayGeom,rayCastPass);
		m_ownsVolume=1;
		return ret;
	}

	bool VolumeRenderer::init(SimpleGL::Texture* vol3d, SimpleVR::ProxyGeomSetup* rayGeom, SimpleVR::RayCastPass* rayCastPass)
	{
		// If not specified, use simple cube geometry (no ESS->slow)
		if (!rayGeom) rayGeom=new ProxyGeomSetup();
		// If not specified, use maximum intensity projection
		if (!rayCastPass) rayCastPass=new DebugRaycastEnvironment(); 
		m_rayGeom=rayGeom;
		m_rayCastPass=rayCastPass;
		setVolume(vol3d);
		CHECK_GL_ERROR
		// don't forget to call resize after init.
		return true;
	}

	void VolumeRenderer::setVolume(SimpleGL::Texture* vol3d)
	{
		if (m_pVolume&&m_ownsVolume)
			delete m_pVolume;
		m_pVolume=vol3d;
		m_ownsVolume=false;
		if (m_rayCastPass)
			m_rayCastPass->setVolume(*m_pVolume);
	}

	SimpleGL::Texture* VolumeRenderer::getVolume()
	{
		return m_pVolume;
	}

	const SimpleGL::Texture* VolumeRenderer::getVolume() const
	{
		return m_pVolume;
	}

	void VolumeRenderer::setRayCastPass(RayCastPass* rcp)
	{
		m_rayCastPass=rcp;
		if (m_rayCastPass)
		{
			if (m_pVolume)
				m_rayCastPass->setVolume(*m_pVolume);
			m_rayCastPass->setTextrues(m_frontTex,m_backTex);
		}
	}

	void VolumeRenderer::raycast()
	{
		glClear(GL_COLOR_BUFFER_BIT);
		if (m_geometryTex)
			m_geometryTex->drawToViewport();
		m_rayCastPass->render();	
	}

	void VolumeRenderer::render(SimpleGL::Texture* target)
	{
		glPushAttrib(GL_COLOR_BUFFER_BIT);
		glDisable(GL_BLEND);
		if (!m_pVolume) return;
		// Render ray geometry
		m_rayGeom->setIntensityRange(m_rayCastPass->intensityMin(),m_rayCastPass->intensityMax());
		m_rayGeom->render(m_frontTex, m_backTex, m_depthTex, m_pVolume->getData());
		// If we have a m_drawSceneFunc, render opaque objects.
		drawScene();
		// Update ray geometry to take opaque objects into account
		rayGeometryUpdate(m_updateTex);

//#define DEBUG_RAY_GEOMETRY_UPDATE
#ifdef DEBUG_RAY_GEOMETRY_UPDATE
//		m_updateTex.drawToViewport();
		m_frontTex.drawToViewport();
//		m_backTex.drawToViewport();
#else
		// Choose appropriate order of textures for clipping or occlusion
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		if (m_geometryTex)
		{
			if ((int)m_clipping<=0)
			{
				if ((int)m_clipping==0) // FIXME
					m_rayCastPass->setTextrues(m_frontTex,m_updateTex);
			}
			else
				m_rayCastPass->setTextrues(m_updateTex,m_backTex);
		}
		// Actual ray casting
		if (target)
		{
			SimpleGL::RenderToTexture rtt(*target,m_fbo);
			raycast();
		}
		else
			raycast();

#endif
		glPopAttrib();
	}

	void VolumeRenderer::drawScene()
	{
		if (m_drawSceneFunc)
		{
			if ( m_geometryTex && // same assumed for m_geometryDepthTex
					( !m_geometryTex->getData() 
					|| m_geometryTex->getData()->dim(0)!=m_frontTex.getData()->dim(0)
					|| m_geometryTex->getData()->dim(1)!=m_frontTex.getData()->dim(1)
					|| m_geometryTex->getData()->dim(2)!=m_frontTex.getData()->dim(2)
					))
			{
				delete m_geometryTex;
				delete m_geometryDepthTex;
				m_geometryTex=0x0;
				m_geometryDepthTex=0x0;
			}
			if (!m_geometryTex)
			{
				m_geometryTex=new SimpleGL::Texture();
//				std::cout << "Creating geometry texture...\n";
				m_geometryTex->createEmptyTexture<unsigned char>(
					m_frontTex.getData()->dim(0),
					m_frontTex.getData()->dim(1),
					m_frontTex.getData()->dim(2),
					4);
			}
			if (!m_geometryDepthTex)
			{
				m_geometryDepthTex=new SimpleGL::Texture();
//				std::cout << "Creating geometry depth texture...\n";
				m_geometryDepthTex->createEmptyTexture<float>(
					m_frontTex.getData()->dim(0),
					m_frontTex.getData()->dim(1),
					1,1,GL_DEPTH_COMPONENT, GL_DEPTH_COMPONENT24);
			}
			SimpleGL::RenderToTexture rtt(*m_geometryTex,*m_geometryDepthTex,m_fbo,GL_ALL_ATTRIB_BITS);
			glClearColor(0.0,0.0,0.0,0.0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LEQUAL);
			m_drawSceneFunc();
		}
	}

	void VolumeRenderer::rayGeometryUpdate(SimpleGL::Texture& target)
	{
		// If we have no geometry we do not need to do anything
		if (!m_geometryTex) return;
		CHECK_GL_ERROR

		// compile back projection and update shader
		if (!m_rayGeomUpdate.isValid())
		{
			if (!m_rayGeomUpdate.compile(geomUpdateSource,g_vertexPassThrough))
			{
				std::cerr << "Without a geometry update, opaque scene and clipping geometry will be ignored!\n";
				if (m_drawSceneFunc)
				{
					delete m_geometryTex;
					delete m_geometryDepthTex;
				}
				m_geometryTex=0x0;
				m_geometryDepthTex=0x0;
				m_drawSceneFunc=0x0;
			}
		}

		// set uniforms depending on whether we have a front or back update
		m_depth=m_geometryDepthTex;
		m_rayEntry=&m_frontTex;
		m_rayExit=&m_backTex;

		SimpleGL::Vector3f v;
		v.x=(float)m_pVolume->getData()->physicalSize(0);
		v.y=(float)m_pVolume->getData()->physicalSize(1);
		v.z=(float)m_pVolume->getData()->physicalSize(2);
		m_size=v;
		
		m_rayGeomUpdate.enable();
		SimpleGL::RenderToTexture rtt(target,m_fbo);
		glClearColor(0.0,0.0,0.0,0.0);
		glClear(GL_COLOR_BUFFER_BIT);
		glBegin(GL_QUADS);
			glTexCoord2f(0,0);
			glVertex2f(-1,-1);
			glTexCoord2f(1,0);
			glVertex2f(1,-1);
			glTexCoord2f(1,1);
			glVertex2f(1,1);
			glTexCoord2f(0,1);
			glVertex2f(-1,1);
		glEnd();
		m_rayGeomUpdate.disable();
		CHECK_GL_ERROR
	}

	void VolumeRenderer::setDrawGeometryFunc(void (*render)(), int clipping)
	{
		if (!render)
		{
			if (m_geometryTex) delete m_geometryTex;
			if (m_geometryDepthTex) delete m_geometryDepthTex;
			m_rayCastPass->setTextrues(m_frontTex,m_backTex);
		}
		m_drawSceneFunc=render;
		m_clipping=clipping;
		CHECK_GL_ERROR
	}

	void VolumeRenderer::setGeometryTextures(SimpleGL::Texture& color, SimpleGL::Texture& depth, int clipping)
	{
		/// If we own geometry textures, delete them
		if (m_drawSceneFunc)
		{
			if (m_geometryTex) delete m_geometryTex;
			if (m_geometryDepthTex) delete m_geometryDepthTex;
			m_geometryTex=0x0;
			m_geometryDepthTex=0x0;
			m_drawSceneFunc=0x0;
		}
		m_geometryTex=&color;
		m_geometryDepthTex=&depth;
		m_clipping=clipping;
	}

} // namespace SimpleVR
