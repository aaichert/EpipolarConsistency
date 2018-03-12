#ifndef __projection_parameters_hxx
#define __projection_parameters_hxx

#include <GetSet/GetSetObjects.h>
#include <LibProjectiveGeometry/CameraOpenGL.hxx>

namespace UtilsQt {

	/// A user-defined ProjectionMatrix, which can be changed by mouse interaction
	struct ProjectionParameters : public GetSetGui::Configurable {

		double				fov;					//< Field of View
		double				viewing_distance;		//< Distance to center
		Eigen::Vector4d		angle;					//< Viewing angles X/Y and additional rotation of the object X/Y
		Eigen::Vector3d		center;					//< Center (which camera is looking at)
		Eigen::Vector2d		image_size;				//< Size of image in pixels

		ProjectionParameters()
			: fov(20)
			, viewing_distance(2500)
			, angle(0.785,0.785,0,0)
			, center(0,0,0)
			, image_size(800,600)
		{}

		virtual Geometry::ProjectionMatrix getProjectionMatrix(double w, double h)
		{
			image_size=Eigen::Vector2d(w,h);
			return getProjectionMatrix();
		}

		virtual ProjectionParameters& setFOV              (double                 _fov              ) {fov             =_fov             ; return *this;}
		virtual ProjectionParameters& setViewing_distance (double                 _viewing_distance ) {viewing_distance=_viewing_distance; return *this;}
		virtual ProjectionParameters& setAngle            (const Eigen::Vector4d& _angle            ) {angle           =_angle           ; return *this;}
		virtual ProjectionParameters& setCenter           (const Eigen::Vector3d& _center           ) {center          =_center          ; return *this;}

		virtual void mouse(int dx, int dy, bool left_button)
		{
			if (left_button)
			{
				angle[1]-=0.005*dx;
				angle[0]-=0.005*dy;
				if (angle[0]<-3.14*0.5) angle[0]=-3.14*0.5;
				if (angle[0]> 3.14*0.5) angle[0]=3.14*0.5;
			}
			else
			{
				angle[2]+=0.005*dx;
				angle[3]+=0.005*dy;
			}
		}

		virtual void wheel(int dy)
		{
 			viewing_distance-=0.25*dy;
			if (viewing_distance<0) viewing_distance=0;
		}

		virtual Geometry::ProjectionMatrix getProjectionMatrix()
		{
			Eigen::Vector3d eye=viewing_distance*Eigen::Vector3d(cos(angle[1])*cos(angle[0]), sin(angle[0]), sin(angle[1])*cos(angle[0]));
			auto K  =Geometry::cameraPerspective(fov/180*Geometry::Pi, image_size[0], image_size[1]);
			auto Rx =Geometry::RotationX(angle[2]);
			auto Rz =Geometry::RotationZ(angle[3]);
			return Geometry::cameraLookAt(K, eye, center)*Rx*Rz;
		}

		virtual void gui_declare_section (const GetSetGui::Section& section)
		{
			GetSet<double>         ("Field of View"     ,section,fov             ).setDescription("The field of view in degrees.");
			GetSet<double>         ("Distance to Center",section,viewing_distance).setDescription("Distance of camera center to focus point.");
			GetSet<Eigen::Vector4d>("Rotation"          ,section,angle           ).setDescription("Rotation of camera. Viewing angles X/Y and additional rotation of the object X/Y.");
			GetSet<Eigen::Vector3d>("Center"            ,section,center          ).setDescription("Center (which camera is looking at).");
			GetSet<Eigen::Vector2d>("Image Size"        ,section,image_size      ).setDescription("Size of image in pixels.");
		}

		virtual void gui_retreive_section(const GetSetGui::Section& section)
		{
			fov             =GetSet<double>         ("Field of View"     ,section);
			viewing_distance=GetSet<double>         ("Distance to Center",section);
			angle           =GetSet<Eigen::Vector4d>("Rotation"          ,section);
			center          =GetSet<Eigen::Vector3d>("Center"            ,section);
			image_size      =GetSet<Eigen::Vector2d>("Image Size"        ,section);
		}

	};

	/// A user-defined ProjectionMatrix, which can be changed by mouse interaction
	class ProjectionParametersGui : public ProjectionParameters, public GetSetGui::Object {
	public:

		ProjectionParametersGui(GetSetInternal::Section& section, GetSetGui::ProgressInterface* app)
			: ProjectionParameters()
			, GetSetGui::Object(section, app)
		{gui_declare_section(gui_section());}

		virtual ProjectionParameters& setFOV              (double                 _fov              ) {GetSet<double>         ("Field of View"     ,gui_section())=_fov             ; return *this;}
		virtual ProjectionParameters& setViewing_distance (double                 _viewing_distance ) {GetSet<double>         ("Distance to Center",gui_section())=_viewing_distance; return *this;}
		virtual ProjectionParameters& setAngle            (const Eigen::Vector4d& _angle            ) {GetSet<Eigen::Vector4d>("Rotation"          ,gui_section())=_angle           ; return *this;}
		virtual ProjectionParameters& setCenter           (const Eigen::Vector3d& _center           ) {GetSet<Eigen::Vector3d>("Center"            ,gui_section())=_center          ; return *this;}

		virtual void mouse(int dx, int dy, bool left_button)
		{
			ProjectionParameters::mouse(dx,dy,left_button);
			GetSet<Eigen::Vector4d>("Rotation",gui_section())=angle;
		}

		void gui_notify(const std::string& section, const GetSetInternal::Node& node)
		{ }

		virtual void wheel(int dy)
		{
			ProjectionParameters::wheel(dy);
			GetSet<double>("Distance to Center",gui_section())=viewing_distance;
		}

		virtual Geometry::ProjectionMatrix getProjectionMatrix(double w, double h)
		{
			if (image_size!=Eigen::Vector2d(w,h))
				GetSet<Eigen::Vector2d>("Image Size",gui_section())=image_size=Eigen::Vector2d(w,h);
			return getProjectionMatrix();
		}

		virtual Geometry::ProjectionMatrix getProjectionMatrix()
		{
			gui_retreive_section(gui_section());
			return ProjectionParameters::getProjectionMatrix();
		}

	};

} // namespace UtilsQt

#endif // __projection_parameters_hxx
