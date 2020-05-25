#include "SimpleRayCastPass.h"

namespace SimpleVR
{
		extern const char *g_vertexPassThrough;

		RayCastPass::RayCastPass(const char * source)
			: m_pass()
			, volume("volume", m_pass)
			, rayEntry("rayEntry", m_pass)
			, rayExit("rayExit", m_pass)
			, numberOfVoxels("numberOfVoxels", m_pass)
			, elementSpacing("elementSpacing", m_pass)
			, samplesPerVoxel("samplesPerVoxel", m_pass)
		{
			if (source)
			{
				m_pass.compile(source,g_vertexPassThrough);
				samplesPerVoxel=1.5;
			}
		}

		void RayCastPass::setVolume(SimpleGL::Texture& volumeTex)
		{
			SimpleGL::Vector3f size;
				size.x=(float)volumeTex.getData()->dim(0);
				size.y=(float)volumeTex.getData()->dim(1);
				size.z=(float)volumeTex.getData()->dim(2);
			SimpleGL::Vector3f spacing;
				spacing.x=(float)volumeTex.getData()->spacing(0);
				spacing.y=(float)volumeTex.getData()->spacing(1);
				spacing.z=(float)volumeTex.getData()->spacing(2);
			volume=&volumeTex;
			numberOfVoxels=size;
			elementSpacing=spacing;
		}

		void RayCastPass::setTextrues(SimpleGL::Texture& rayEntryTex, SimpleGL::Texture& rayExitTex)
		{
			rayEntry=&rayEntryTex;
			rayExit=&rayExitTex;
		}

		void RayCastPass::setSamplesPerVoxel(float spv)
		{
			samplesPerVoxel=spv;
		}

		void RayCastPass::render()
		{
			m_pass.enable();
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
			m_pass.disable();
		}

	const char * g_debugRaycastEnvironment =
		CREATE_RAY_SHADER
		"\n"
		"void main()\n"
		"{\n"
		"   Ray ray=createRay();\n"
		"   if (gl_TexCoord[0].s<0.5)\n"
		"   {\n"
		"       if (gl_TexCoord[0].t<0.5)\n"
		"           gl_FragColor=vec4(vec2(gl_TexCoord[0].st),0.0,1.0);\n"
		"       else\n"
		"		{\n"
		"			if (sin(gl_TexCoord[0].s*50.0)>0.0)\n"
		"				gl_FragColor=vec4(vec3(ray.start),1.0);\n"
		"			else\n"
		"				gl_FragColor=vec4(vec3(ray.end),1.0);\n"
		"		}\n"
		"       return;"
		"   }\n"
		"   float m=0.0;\n"
		"   vec3 pos=ray.start;\n"
		"   for (float a=0.0;a<1.0;a+=ray.adv)\n"
		"   {\n"
		"       float sample=texture3D(volume,pos).r;\n"
		"       if (m<sample)\n"
		"           m=sample;\n"
		"       pos+=ray.dir;\n"
		"   }\n"
		"   gl_FragColor=vec4(m);\n"
		"}\n"
		"\n"
		;

	DebugRaycastEnvironment::DebugRaycastEnvironment() : RayCastPass(g_debugRaycastEnvironment) {}

	const char * g_mipShader =
		CREATE_RAY_SHADER
		"\n"
		"void main()\n"
		"{\n"
		"   Ray ray=createRay();\n"
		"   if (all(equal(ray.start,ray.end)))\n"
		"       discard;\n"
		"   float m=0.0;\n"
		"   vec3 pos=ray.start;\n"
		"   for (float a=0.0;a<1.0;a+=ray.adv)\n"
		"   {\n"
		"       float sample=texture3D(volume,pos).r;\n"
		"       if (m<sample)\n"
		"           m=sample;\n"
		"       pos+=ray.dir;\n"
		"   }\n"
		"   gl_FragColor=vec4(vec3(m),1.0);\n"
		"}\n"
		"\n"
		;

	MaximumIntensityProjection::MaximumIntensityProjection() : RayCastPass(g_mipShader) {}


	const char * g_isoShader =
		CREATE_RAY_SHADER
		GRADIENT_SHADER
		"\n"
		"uniform float iso; // iso value for surface\n"
		"uniform bool shaded; // white phong shaded or color-coded iso hit\n"
		"\n"
		"void main()\n"
		"{\n"

		//"gl_FragData[0]=texture3D(volume,vec3(gl_TexCoord[0].xy,iso));\n"
		//"return;\n"

		"   Ray ray=createRay();\n"
		"   if (all(equal(ray.start,ray.end)))\n"
		"       discard;\n"
		"   vec3 pos=ray.start;\n"
		"	float a;\n"
		"   for (a=0.0;a<1.0;a+=ray.adv)\n"
		"   {\n"
		"       float sample=texture3D(volume,pos).r;\n"		
		"       if (sample>iso)\n"
		"           break;\n"
		"       pos+=ray.dir;\n"
		"   }\n"
	
		"	if (a>=1.0)\n"
		"	    discard;\n"
		// hit refinement
		"	vec3 _in=pos;\n"
		"	vec3 _out=pos-ray.dir;\n"
		"	pos=0.5*(_in+_out);\n"
		"	for (int i=0;i<5;i++)\n"
		"	{\n"
		"		if (texture3D(volume,pos).r<iso)\n"
		"			_out=pos;\n"
		"		else\n"
		"			_in=pos;\n"
		"		pos=0.5*(_in+_out);\n"
		"	}\n"
		"	if (shaded)\n"
		"       gl_FragColor=vec4(vec3(shade(gradient(pos),ray.dir)),1.0);\n"
		"   else\n"
		"       gl_FragColor=vec4(pos,1.0);\n"
		"}\n"
		"\n"
		;


	IsoSurface::IsoSurface(const char* rcSrc)
		: RayCastPass(0x0)
		, shaded("shaded",m_pass)
		, isoValue("iso",m_pass)
	{
		if (!rcSrc) rcSrc=g_isoShader;
		m_pass.compile(rcSrc,g_vertexPassThrough);
		samplesPerVoxel=1.5;
	}


	const char * g_drrShader =
		CREATE_RAY_SHADER
		"\n"
		"uniform float rampMin;\n"
		"uniform float rampMax;\n"
		"uniform float rampHeight;\n"
		"\n"
		"void main()\n"
		"{\n"
		"	Ray ray=createRay();\n"
		"	if (all(equal(ray.start,ray.end)))\n"
		"		discard;\n"
		"	vec3 pos=ray.start;\n"
		"	float a;\n"
		"	float sum=0.0;\n"
		"	for (a=0.0;a<1.0;a+=ray.adv)\n"
		"	{\n"
		"		float sample=clamp((texture3D(volume,pos).r-rampMin)/(rampMax-rampMin),0.0,1.0)*rampHeight;\n"
		"		sum+=sample;\n"
		"		pos+=ray.dir;\n"
		"	}\n"
		"	gl_FragColor=vec4(vec3(exp(-sum/ray.samplesPerMM)),1.0);\n"
		"}\n"
		"\n"
		;

	DigitallyReconstructedRadiograph::DigitallyReconstructedRadiograph()
		: RayCastPass(g_drrShader)
		, rampMin("rampMin",m_pass)
		, rampMax("rampMax",m_pass)
		, rampHeight("rampHeight",m_pass)
	{}

	const char * g_depthColorShader =
		CREATE_RAY_SHADER
		"\n"
		"uniform float rampMin;\n"
		"uniform float rampMax;\n"
		"uniform float rampHeight;\n"
		"\n"
		"uniform vec4 near;\n"
		"uniform vec4 center;\n"
		"uniform vec4 far;\n"
		"\n"
		"void main()\n"
		"{\n"
		"	Ray ray=createRay();\n"
		"	if (all(equal(ray.start,ray.end)))\n"
		"		discard;\n"
		// compute layer through center of volume perpendicular to viewing direction and distance scaling
		"	vec3	size=numberOfVoxels*elementSpacing;\n"
		"	vec3	volCtr=size*0.5;\n"
		"	float	maxDist=2*length(volCtr);\n"
		"	vec4	plane=vec4(normalize(ray.dir*size),0.0);\n"
		"	plane.w=-dot(volCtr,plane.xzy);\n"
		
		"	vec3 pos=ray.start;\n"
		"	float a;\n"
		"	vec3 sum=vec3(0.0);\n"
		"	for (a=0.0;a<1.0;a+=ray.adv)\n"
		"	{\n"
		"		float sample=clamp((texture3D(volume,pos).r-rampMin)/(rampMax-rampMin),0.0,1.0)*rampHeight;\n"
		"		vec3 color;\n"
		"		float dist=dot(vec4(pos*size,1.0),plane);\n"
		"		dist=dist/maxDist;\n"
		"		dist+=(0.5-center.w)*2;\n"
		"		if (dist<0.0) // near\n"
		"			color=mix(center.rgb,near.rgb,dist/((near.w-center.w)*2.0));\n"
		"		else // far\n"
		"			color=mix(center.rgb,far.rgb,dist/((far.w-center.w)*2.0));\n"
		"		sum+=sample*color;\n"
		"		pos+=ray.dir;\n"
		"	}\n"
		"	gl_FragColor=vec4(sum,1.0);\n"
		"}\n"
		"\n"
		;

	ColorEmissionByDepth::ColorEmissionByDepth()
		: RayCastPass(g_depthColorShader)
		, rampMin("rampMin",m_pass)
		, rampMax("rampMax",m_pass)
		, rampHeight("rampHeight",m_pass)
		, near("near",m_pass)
		, center("center",m_pass)
		, far("far",m_pass)
	{}


} // namespace Simple
