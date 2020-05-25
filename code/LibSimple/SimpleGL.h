#ifndef SIMPLE_GL
#define SIMPLE_GL

/** @brief	Simple OpenGL Texture, Framebuffer Object and GLSL Program & Fragment Shader classes
 *  @author	Andre Aichert
 */

#include "GL/glad.h"

#include "SimpleMHD.h"

#include <map>
#include <vector>
#include <string>
#include <iostream>

#define CHECK_GL_ERROR if (!SimpleGL::checkError()) std::cout << __FILE__ << ": "<< __LINE__ << std::endl << std::endl;

namespace SimpleGL
{
	/// Check for OpenGL errors. Also note CHECK_GL_ERROR macro
	bool checkError();

	/// Returns the OpenGL type constant for a MHD type string (such as "MET_UCHAR"
	inline GLenum getGLTypeForMHDType(const std::string& mhdTypeStr)
	{
		if (mhdTypeStr=="MET_CHAR") return GL_BYTE;
		if (mhdTypeStr=="MET_UCHAR") return GL_UNSIGNED_BYTE;
		if (mhdTypeStr=="MET_SHORT") return GL_SHORT;
		if (mhdTypeStr=="MET_USHORT") return GL_UNSIGNED_SHORT;
		if (mhdTypeStr=="MET_INT") return GL_INT;
		if (mhdTypeStr=="MET_UINT") return GL_UNSIGNED_INT;
		if (mhdTypeStr=="MET_FLOAT") return GL_FLOAT;
		if (mhdTypeStr=="MET_DOUBLE") return GL_DOUBLE;
		return 0;
	}

	/// Push MVP and go directly to clip space
	inline void pushAndLoadMatrices()
	{
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();		
	}

	// Pop MVP
	inline void popMatrices()
	{
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();	
	}

	/// Fill the viewport with a quad.
	inline void drawQuad(float z=0.5)
	{
		pushAndLoadMatrices();
		glBegin(GL_QUADS);
			glTexCoord3f(0,1,z);
			glVertex2f(-1,1);
			glTexCoord3f(1,1,z);
			glVertex2f(1,1);
			glTexCoord3f(1,0,z);
			glVertex2f(1,-1);
			glTexCoord3f(0,0,z);
			glVertex2f(-1,-1);
		glEnd();
		popMatrices();
	}
	 
	/// A basic 2D/3D texture class
	class Texture : public MHD::Image
	{
	private:
		/// Texture ID
		GLuint m_id;
		GLenum m_target; 

	public:
		Texture();
		~Texture();
	
		/// Destroy this texture (delete texture and release data)
		virtual void destory();

		/// Implicit cast to unsigned int by texture ID
		inline operator GLuint() const { return m_id; }

		/// Get GL type for a C-type (via MHD type 2do simplify)
		template <typename T> static GLenum getGLType(T* unused=0x0)
		{
			return getGLTypeForMHDType(MHD::getTypeStr<T>());
		}

		/// Get some GL format that matches number of channels
		static GLenum getGLFormat(int c);

		/// Bind the texture to a unit
		void bind(int unit=0);
		/// Unbinds whatever Texture is bound to unit
		static void unbind(int unit=0);

		/// Download data to GPU. Texture must be bound before calling download(...)
		void download(void* data);
		/// Readback texture to RAM. Texture must be bound before calling upload()
		void upload(void *data=0x0);

		/// Create an empty texture without allocating data on CPU.
		template <typename T> void createEmptyTexture(int w, int h, int d=1, int c=1, GLenum format=0, GLenum internalformat=0)
		{
			if (m_data)
				delete m_data;
			m_data=new MHD::Data<T>();
			m_data->setDimensions(w,h,d,c);
			createTexture(format, internalformat);
		}

		/// Create a tex id and download data if available.
		void createTexture(GLenum format=0, GLenum internalformat=0);
		
		/// Uses pipeline calls to render a texture full screen quad
		void drawToViewport(float z=0.5);
	};

	/// A basic FramebufferObject to render to textures
	class FramebufferObject
	{
	private:
		GLuint m_name;
		std::map< GLenum, const Texture *> m_attachments;

	public:
		FramebufferObject();
		~FramebufferObject();

		/// Set color attachment by index. Must be called before enable()
		void attachColor(const Texture& tex, int colorAttachment=0);

		/// Attach or detach a texture. Must be called before enable()
		void attachTexture(GLenum attachment, const Texture* tex=0x0);
		
		/// Attach textures and enable
		void enable();

		/// Go back to normal back buffer
		static void disable();

		/// Detach all textures and disable this fbo
		void cleanUp();

		/// Returns false and prints an error to cerr if the framebuffer cannot be used.
		bool isValid();

	};
	
	/// A Utility class to render to textures
	class RenderToTexture
	{
	protected:
		bool				 m_ownsFBO;		/// If true, the FBO will be deleted on destruction
		FramebufferObject	*m_fbo;			/// FBO used to render to texture
		bool				 m_pushedState; /// Has the constructor pushed the GL attrib stack?
		bool				 m_hasDepth;	/// True if a depth texture has been supplied to ctor
		int					 m_x;			/// Size of the render target X
		int					 m_y;			/// Size of the render target Y

		///  Common code for ctor
		void init(GLbitfield attribBits, FramebufferObject *fbo);

	public:
		RenderToTexture(Texture& target, FramebufferObject *fbo=0x0, GLbitfield attribBits=GL_VIEWPORT_BIT | GL_COLOR_BUFFER_BIT);
		RenderToTexture(Texture& color, Texture& depth, FramebufferObject *fbo=0x0, GLbitfield attribBits=GL_VIEWPORT_BIT | GL_COLOR_BUFFER_BIT);
		~RenderToTexture();

		/// Direct access to FBO
		FramebufferObject* fbo() { return m_fbo; }

		/// calls a render Function
		template <class Callback>
		static void call(const Callback& function)
		{ function(); }

		/// calls a render or resize function taking width and height of render target as arguments
		template <class Callback_int_int>
		static void callWithSize(Callback_int_int& function)
		{ function(m_x, m_y); }
				
	};

	/// Interface to allow GLSLProgram to share variables with C++
	class UniformInterface
	{
	protected:
		friend class GLSLProgram;
		UniformInterface(const std::string& name, GLSLProgram& program);
		virtual ~UniformInterface();

		GLSLProgram*	owner;		/// Program this Uniform belongs to
		int				m_location;	/// Uniform location in owner (or < 1)
		bool			m_dirty;	/// Flag indicating if the value of the uniform has been set yet
		std::string		m_name;		/// Name of this variable in the shader's source

		/// Update value of the variable on GPU. Produces GL_INVALID_VALUE_ERROR if inactive or not found
		virtual void set()=0;

		/// Query uniform location from driver (returns <0 if inactive or not found)
		int location();

	};

	/// A value shared between C++ and a GLSL shader. Behaves much like a C++ variable of type T.
	template <typename T> class Uniform : public UniformInterface
	{
	public:
		T m_value;
		Uniform(const std::string& name, GLSLProgram& program) : UniformInterface(name, program) {}
		inline operator const T& () const {return m_value;}
		inline T& operator=(const T& v) { location(); return m_value=v; }

		virtual void set() {std::cerr << "Uniform of unknown type used.\n";};
	};
	
	/// A basic GLSL program containing a fragment shader
	class GLSLProgram
	{
	protected:
		typedef std::map<std::string, UniformInterface*>	uniform_map;
		typedef std::map<Uniform<Texture*>*, int>			texture_uniform_map;

		GLuint					m_program;	/// Program id
		GLuint					m_frag;		/// Fragment Shader id
		GLuint					m_vert;		/// Vertex Shader id or 0
		uniform_map				m_uniforms;	/// Keeps track of all uniforms used in this program
		texture_uniform_map		m_textures; /// Keeps track of Textures used in this program
	
	public:
		GLSLProgram();
		~GLSLProgram();

		/// Clean up
		void destroy();

		/// compile and link a program containing just this shader
		bool compile(const char* fss, const char* vss=0x0);

		/// Use the program for this shader, also bind textures.
		void enable();

		/// Go back to pipeline, also unbinds textures.
		void disable();

		/// Check if program could be compiled and liked
		bool isValid();

		friend class UniformInterface;
		friend void Uniform<Texture*>::set();
	};

	/// Rudimentary 3-Vector Type
	typedef union {
		float& operator[](int i) {return data[i];}
		const float& operator[](int i) const {return data[i];}
		struct { float x,y,z; };
        float data[3];
    } Vector3f;

	/// Rudimentary 4-Vector Type
	typedef union {
		float& operator[](int i) {return data[i];}
		const float& operator[](int i) const {return data[i];}
		struct { float x,y,z,w; };
        float data[4];
    } Vector4f;
	
	template <typename VEC, int N> void setFloats(VEC& v, float* f)
	{
		for (int i=0;i<N;i++) v[i]=f[i];
	}

	/// Rudimentary 4-Vector Matrix
	struct Matrix4f {
		double& operator[](int i) {return data[i];}
		double& operator()(int i, int j) {return data[i*4+j];}
		const double& operator[](int i) const {return data[i];}
		const double& operator()(int i, int j) const {return data[i*4+j];}
        double data[16];
    };

	/// Type specializations
	template<> void Uniform<bool>::set()	{glUniform1i(m_location,(int)m_value);m_dirty=0;}
	template<> void Uniform<int>::set()		{glUniform1i(m_location,m_value);m_dirty=0;}
	template<> void Uniform<float>::set()	{glUniform1f(m_location,m_value);m_dirty=0;}
	template<> void Uniform<Vector3f>::set(){glUniform3fv(m_location,1,(float*)&m_value);m_dirty=0;}
	template<> void Uniform<Vector4f>::set(){glUniform4fv(m_location,1,(float*)&m_value);m_dirty=0;}
	template<> void Uniform<Matrix4f>::set(){glUniformMatrix4fv(m_location,1,false,(float*)&m_value);m_dirty=0;}

	/// type specializations for Uniform Samplers
	template<> void Uniform<Texture*>::set()
	{
		if (m_location<0)
			return; // incactive or not found; no error, just ignore
		int unit=owner->m_textures[this];
		glUniform1i(m_location,unit);
		m_dirty=0;
	}


} // namespace SimpleGL

#endif // SIMPLE_GL
