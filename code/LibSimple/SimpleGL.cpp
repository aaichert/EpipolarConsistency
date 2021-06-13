#include <iostream>

#include "SimpleGL.h"

namespace SimpleGL
{
	bool checkError()
    {
		int nErr=0;
		while (1)
		{
			GLenum err=glGetError();
			switch ((int)err)
			{
				case  (int)GL_NO_ERROR:
					return nErr==0;
				break;
				case (int)GL_INVALID_OPERATION:
					std::cerr << "OpenGL Error:  An invalid operation error has been generated." << std::endl;
				break;
				case (int)GL_INVALID_ENUM:
					std::cerr << "OpenGL Error:  An invalid enum error has been generated." << std::endl;
				break;
				case (int)GL_INVALID_VALUE:
					std::cerr << "OpenGL Error:  An invalid value error has been generated." << std::endl;
				break;
				case (int)GL_STACK_OVERFLOW:
					std::cerr << "OpenGL Error:  A stack overflow error has been generated." << std::endl;
				break;
				case (int)GL_STACK_UNDERFLOW:
					std::cerr << "OpenGL Error:  A stack underflow error has been generated." << std::endl;
				break;
				case (int)GL_OUT_OF_MEMORY:
					std::cerr << "OpenGL Error:  Out of memory error has been generated." << std::endl;
				break;
				default:
					std::cerr << "OpenGL Error:  An unknown GL error has been generated." << std::endl;
				break;
			}
			nErr++; // break here to break on OpenGL Errors
		}
    }

	Texture::Texture() : MHD::Image(), m_id(0), m_target(0x0) {}
	Texture::~Texture()
	{
		if (m_id)
			glDeleteTextures(1,&m_id);
		m_id=0;
	}

	void Texture::destory()
	{
		if (m_id)
			glDeleteTextures(1,&m_id);
		m_id=0;
		MHD::Image::destroy();
	}

	GLenum Texture::getGLFormat(int c)
	{
		switch (c)
		{
			case 1: return GL_LUMINANCE;
			case 2: return GL_LUMINANCE_ALPHA;
			case 3: return GL_RGB;
			case 4: return GL_RGBA;
			default: return 0;
		}
	}

	void Texture::bind(int unit)
	{
		glActiveTexture(GL_TEXTURE0+unit);
		glBindTexture(m_target,m_id);
	}

	void Texture::unbind(int unit)
	{
		glActiveTexture(GL_TEXTURE0+unit);
		glBindTexture(GL_TEXTURE_2D,0);
		glBindTexture(GL_TEXTURE_3D,0);
	}

	void Texture::download(void* data)
	{
		if (m_target==GL_TEXTURE_3D)
			glTexSubImage3D(m_target,0,0,0,0,m_data->dim(0),m_data->dim(1),m_data->dim(2),
				getGLFormat(m_data->channels()),getGLTypeForMHDType(m_data->getType()),data);
		if (m_target==GL_TEXTURE_2D)
			glTexSubImage2D(m_target,0,0,0,m_data->dim(0),m_data->dim(1),
				getGLFormat(m_data->channels()),getGLTypeForMHDType(m_data->getType()),data);
		if (m_target==GL_TEXTURE_1D)
			glTexSubImage1D(m_target,0,0,m_data->dim(0),
				getGLFormat(m_data->channels()),getGLTypeForMHDType(m_data->getType()),data);
	}

	void Texture::upload(void *data)
	{
		CHECK_GL_ERROR
		if (!data)
		{
			if (m_data->raw()==0)
				m_data->alloc();
			data=m_data->raw();
		}
		glGetTexImage(m_target,0,getGLFormat(m_data->channels()),getGLTypeForMHDType(m_data->getType()),data);
		CHECK_GL_ERROR
	}

	void Texture::createTexture(GLenum format, GLenum internalformat)
	{
		CHECK_GL_ERROR
		if (m_id)
		{
			std::cout << "Deleted Texture " << m_id << std::endl;
			glDeleteTextures(1,&m_id);
		}
		glGenTextures(1,&m_id);
		std::cout << "Created Texture " << m_id << std::endl;
		if (!m_data)
			return;
		// choose appropriate format
		GLenum type=getGLTypeForMHDType(m_data->getType());
		if (format==0)
			format=getGLFormat(m_data->channels());
		if (internalformat==0)
		{
			if (type==GL_FLOAT)
				internalformat=GL_RGBA32F;
			else
			internalformat=m_data->channels();
		}
		if (m_data->dim(2)<2)
			m_target=GL_TEXTURE_2D;
		else
			m_target=GL_TEXTURE_3D;
		glBindTexture(m_target, m_id);
		glTexParameteri(m_target, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(m_target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(m_target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(m_target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(m_target, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		// do not do padding. ever. (effective beyond this function)
		glPixelStorei(GL_PACK_ALIGNMENT,1);
		glPixelStorei(GL_UNPACK_ALIGNMENT,1);
		// create/download
		if (m_target==GL_TEXTURE_2D)
			glTexImage2D(GL_TEXTURE_2D,0,internalformat,m_data->dim(0),m_data->dim(1),
				0, format, type, m_data->raw());
		else
			glTexImage3D(GL_TEXTURE_3D, 0, internalformat, m_data->dim(0), m_data->dim(1), m_data->dim(2),
				0, format, getGLTypeForMHDType(m_data->getType()), m_data->raw());
		glBindTexture(m_target, 0);
		CHECK_GL_ERROR
	}

	void Texture::drawToViewport(float z)
	{
		bind();
		glEnable(m_target);
		SimpleGL::drawQuad(z);
		glDisable(m_target);
		unbind(0);
	}

/////////////////////////////////////////////////

	FramebufferObject::FramebufferObject() : m_name(0) {}
	FramebufferObject::~FramebufferObject()
	{
		if (m_name)
			glDeleteFramebuffers(1,&m_name);
	}

	void FramebufferObject::attachColor(const Texture& tex, int colorAttachment)
	{
		attachTexture(GL_COLOR_ATTACHMENT0+colorAttachment, &tex);
	}

	void FramebufferObject::attachTexture(GLenum attachment, const Texture* tex)
	{
		if (tex==0x0)
		{
			std::map< GLenum, const Texture *>::iterator it=m_attachments.find(attachment);
			if (it!=m_attachments.end())
				m_attachments.erase(it);
		}
		else
			m_attachments[attachment]=tex;
	}

	void FramebufferObject::enable()
	{
		if (m_name==0)
			glGenFramebuffers(1,&m_name);
		glBindFramebuffer(GL_FRAMEBUFFER,m_name);
		for (std::map< GLenum, const Texture *>::iterator it=m_attachments.begin();it!=m_attachments.end();++it)
			if (it->second)
				glFramebufferTexture2D(GL_FRAMEBUFFER, it->first, GL_TEXTURE_2D, (GLuint)*(it->second), 0);
		glDrawBuffer(GL_COLOR_ATTACHMENT0);
	}

	void FramebufferObject::disable()
	{
		glBindFramebuffer(GL_FRAMEBUFFER,0);
	}

	void FramebufferObject::cleanUp()
	{
		glBindFramebuffer(GL_FRAMEBUFFER,m_name);
		for (std::map< GLenum, const Texture *>::iterator it=m_attachments.begin();it!=m_attachments.end();++it)
				glFramebufferTexture2D(GL_FRAMEBUFFER, it->first, GL_TEXTURE_2D, (GLuint)0, 0);
		m_attachments.clear();
		glBindFramebuffer(GL_FRAMEBUFFER,0);		
		glDrawBuffer(GL_BACK);
	}

	bool FramebufferObject::isValid()
	{
		std::string reason;
		int state=glCheckFramebufferStatus(GL_FRAMEBUFFER);
        CHECK_GL_ERROR
		switch (state)
		{
			case (int)GL_FRAMEBUFFER_COMPLETE:
				return true;
			break;
			case (int)GL_INVALID_VALUE:
				reason="because its name is not a valid framebuffer.";
			break;
			case (int)GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
				reason="because it has incomplete attachments.";
			break;
			case (int)GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
				reason="becuase it has incomplete or missing attachments.";
			break;
			case (int)GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
				reason="because it has incomplete read buffer.";
			break;
			case (int)GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
				reason="because it has incomplete draw buffer.";
			break;
			case (int)GL_INVALID_FRAMEBUFFER_OPERATION:
				reason="because of an invalid operation.";
			break;
			case (int)GL_FRAMEBUFFER_UNSUPPORTED:
				reason="because this computer does not support GL_framebuffer_object.";
			break;
			default:
				reason="for an unknown reason.";
			break;
		}
		std::cerr << "Failed to use FBO " << reason << std::endl;
		return false;
	}

/////////////////////////////////////////////////

	
void RenderToTexture::init(GLbitfield attribBits, FramebufferObject *fbo)
{
	CHECK_GL_ERROR
	m_fbo=fbo;
	m_ownsFBO=!fbo;
	m_pushedState=0!=attribBits;
	if (m_ownsFBO)
		m_fbo=new FramebufferObject();
	if (m_pushedState)
		glPushAttrib(attribBits);
	glViewport(0,0,m_x,m_y);
}


RenderToTexture::RenderToTexture(Texture& target, FramebufferObject *fbo, GLbitfield attribBits)
	: m_ownsFBO(0)
	, m_fbo(0x0)
	, m_pushedState(0)
	, m_hasDepth(0)
	, m_x(0)
	, m_y(0)
{
	if (!target.getData())
		return;
	m_x=target.getData()->dim(0);
	m_y=target.getData()->dim(1);
	init(attribBits,fbo);
	m_fbo->attachColor(target);
	m_fbo->enable();
	m_fbo->isValid();
}

RenderToTexture::RenderToTexture(Texture& color, Texture& depth, FramebufferObject *fbo, GLbitfield attribBits)
	: m_ownsFBO(0)
	, m_fbo(0x0)
	, m_pushedState(0)
	, m_hasDepth(1)
	, m_x(0)
	, m_y(0)
{
	if (!color.getData() || !depth.getData())
		return;
	m_x=color.getData()->dim(0);
	m_y=color.getData()->dim(1);
	init(attribBits,fbo);
	m_fbo->attachColor(color);
	m_fbo->attachTexture(GL_DEPTH_ATTACHMENT,&depth);
	m_fbo->enable();
	m_fbo->isValid();
}

RenderToTexture::~RenderToTexture()
{
	if (!m_fbo)
		return;
	if (m_hasDepth)
		m_fbo->attachTexture(GL_DEPTH_ATTACHMENT,0x0);
	m_fbo->disable();
	if (m_ownsFBO)
	{
		delete m_fbo;
		m_fbo=0x0;
	}
	if (m_pushedState)
		glPopAttrib();
	CHECK_GL_ERROR
}

/////////////////////////////////////////////////

	UniformInterface::UniformInterface(const std::string& name, GLSLProgram& program)
		: owner(&program)
		, m_location(-1)
		, m_dirty(1)
		, m_name(name)
	{}

	UniformInterface::~UniformInterface()
	{
		GLSLProgram::uniform_map::iterator it=this->owner->m_uniforms.find(m_name);
		// if (it->second==this) owner->m_uniforms.erase(it); // FIXME
	}

	int UniformInterface::location()
	{
		// update location and let owner now we're here
		if (owner->m_program)
			m_location=glGetUniformLocation(owner->m_program,m_name.c_str());
		else
			m_location=-1;
		if (m_location>=0)
		{
			m_dirty=1;
			owner->m_uniforms[m_name]=this;
		}
		else
		{
			GLSLProgram::uniform_map::iterator it=owner->m_uniforms.find(m_name);
			if (it!=owner->m_uniforms.end())
				owner->m_uniforms.erase(it);
			m_location=-1;
			m_dirty=0; // we are no longer active; no error, just ignore.
		}
		// test if we are dealing with a uniform sampler, which needs special treatment
		Uniform<Texture*>* sampler=dynamic_cast<Uniform<Texture*>* >(this);
		if (sampler)
		{
			if (m_location>=0)
			{
				// find a free texture unit
				int unit=0;
				bool found=1;
				while (found)
				{
					found=0;
					for (GLSLProgram::texture_uniform_map::iterator
						it=owner->m_textures.begin(); it!=owner->m_textures.end(); ++it)
						if (it->second==unit && it->first!=sampler)
						{
							found=1;
							break;
						}
					if (found) unit++;
				}
				owner->m_textures[sampler]=unit;
			}
			else
			{
				// let the owner know we're no longer active
				GLSLProgram::texture_uniform_map::iterator it=owner->m_textures.find(sampler);
				if (it!=owner->m_textures.end())
				{
					it=owner->m_textures.erase(it);
					int unit=0;
					// refresh all other samplers
					for (GLSLProgram::texture_uniform_map::iterator
						it=owner->m_textures.begin(); it!=owner->m_textures.end(); ++it)
					{
						it->first->m_dirty=1;
						it->second=unit++;
					}
				}
			}
		}
		return m_location;
	}

/////////////////////////////////////////////////

	GLSLProgram::GLSLProgram()
		: m_program(0)
		, m_frag(0)
		, m_vert(0)
	{}

	GLSLProgram::~GLSLProgram() {destroy();}

	void GLSLProgram::destroy()
	{
		if (m_program!=0)
		{
			disable();
			glDetachShader(m_program, m_frag);
			if (m_vert)
				glDetachShader(m_program, m_vert);
			glDeleteProgram(m_program);
		}
		if (m_frag!=0)
			glDeleteShader(m_frag);
		if (m_vert!=0)
			glDeleteShader(m_vert);
		m_frag=m_vert=m_program=0;
		m_textures.clear();
	}

	bool GLSLProgram::compile(const char* fss, const char* vss)
	{
		if (!fss) return false;
		destroy();
		CHECK_GL_ERROR
		if (vss)
		{
			m_vert=glCreateShader(GL_VERTEX_SHADER);
			glShaderSource(m_vert, 1, &vss,0x0);
			glCompileShader(m_vert);
		}
		m_frag=glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(m_frag, 1, &fss,0x0);
		glCompileShader(m_frag);
		CHECK_GL_ERROR
		m_program=glCreateProgram();
		if (m_vert)
			glAttachShader(m_program,m_vert);
		glAttachShader(m_program,m_frag);
		glLinkProgram(m_program);
		// check if we're good
		int len=-1;
		int linked=-1;
		glGetProgramiv(m_program, GL_INFO_LOG_LENGTH, &len);
		glGetProgramiv(m_program, GL_LINK_STATUS, (GLint*)&linked);
		// print error log
		if (linked<1)
		{
			char *log=new char[len];
			glGetProgramInfoLog(m_program, len, (GLsizei*)&len, log);
			std::cerr << "Program " << m_program
					  << " with Shader(s) " << m_frag << ", " << m_vert;
			if (vss)
				std::cerr << " failed to build!\n###\n" << fss << "\n###\n" << vss << "\n###\n" << log << "\n###\n";
			else
				std::cerr << " failed to build!\n###\n" << fss << "\n###\n" << log << "\n###\n";
			delete [] log;
			destroy();
			return false;
		}
		// update
		uniform_map old=m_uniforms;
		m_uniforms.clear();
		for (uniform_map::iterator it=old.begin();it!=old.end();++it)
			it->second->location();
		CHECK_GL_ERROR
		return true;
	}

	void GLSLProgram::enable()
	{
		CHECK_GL_ERROR
		if (m_program==0)
			return; // error. linking or compilation probably failed.
		glUseProgram(m_program);
		for (uniform_map::iterator it=m_uniforms.begin(); it!=m_uniforms.end();++it)
			if (it->second->m_dirty)
				it->second->set();
		for (texture_uniform_map::iterator it=m_textures.begin(); it!=m_textures.end();++it)
			if ((it->first->m_value))
				it->first->m_value->bind(it->second);
//		if (!SimpleGL::checkError()) std::cout << "A texture does not exist or some uniform locations have been removed by optimizer.\n";
		glActiveTexture(GL_TEXTURE0);
	}

	void GLSLProgram::disable()
	{
		glUseProgram(0);
		for (texture_uniform_map::iterator it=m_textures.begin(); it!=m_textures.end();++it)
			Texture::unbind(it->second);
		glActiveTexture(GL_TEXTURE0);
	}

	bool GLSLProgram::isValid()
	{
		return m_program!=0;
	}


} // namespace SimpleGL
