#ifndef __projtable_hxx
#define __projtable_hxx

// Created by A. Aichert on Thu Jan 23rd 2014

#include <ctime>

#include <HeaderOnly/Utils/UglyXML.hxx>

#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include <LibProjectiveGeometry/EigenToStr.hxx>
#include <LibProjectiveGeometry/CameraOpenGL.hxx>
#include <LibProjectiveGeometry/SingularValueDecomposition.h>

#include <GetSet/ProgressInterface.hxx>

namespace ProjTable
{
	/// Estimates parameters of a circular trajectory. Assumes iso-center to be at the origin.
	inline void ctCircularTrajectoryToParameters(
		const                           std::vector<Geometry::ProjectionMatrix>& Ps,  //< input: projection matrix
		double                          detector_pixel_spacing=1.0,                   //< input: pixel size on detector
		Geometry::RP3Plane		        *plane_of_rotation=0x0,	                      //< optional output: rotation plane
		std::vector<double>             *primary_angles=0x0,                          //< optional output: gantry primary angle
		std::vector<double>             *secondary_angles=0x0,                        //< optional output: gantry secondary angle
		std::vector<double>             *source_to_iso_center_distances=0x0,          //< optional output: source to iso-center distance
		std::vector<double>             *source_to_detector_distances=0x0,            //< optional output: source to detector distance
		std::vector<Geometry::RP3Point> *source_positions=0x0)                        //< optional output: source positions
	{
		// Number of views
		int n=(int)Ps.size();
		// Must be at least three to define a plane
		if (n<3) return;
		// Allocate space for output variables
		if (primary_angles) primary_angles->resize(n);
		if (secondary_angles) secondary_angles->resize(n);
		if (source_to_iso_center_distances) source_to_iso_center_distances->resize(n);
		if (source_to_detector_distances) source_to_detector_distances->resize(n);
		// Even if the user does not provide this vector, we need to compute them for a plane fit.
		std::vector<Geometry::RP3Point> tmp_source_positions;
		if (!source_positions) source_positions=&tmp_source_positions;
		source_positions->resize(n);

		// Linear plane fit to source trajectory
		Eigen::MatrixXd A(n,4);
		#pragma omp parallel for
		for (int i=0;i<(int)n;i++)
		{
			(*source_positions)[i]=Geometry::getCameraCenter(Ps[i]);
			A.block<1,4>(i,0)=((*source_positions)[i]/(*source_positions)[i].head(3).norm()).transpose();
		}
		Eigen::VectorXd		plane_fit=Geometry::nullspace(A);
		Geometry::RP3Plane	rotation_plane(plane_fit[0],plane_fit[1],plane_fit[2],plane_fit[3]);
		rotation_plane/=rotation_plane.head(3).norm();

		// These two values should be close to zero.
		double residual_distance_to_plane =(A*rotation_plane).sum()/n;
		double distance_of_plane_to_origin=rotation_plane[3];

		//Pick two projections as rotation direction reference
		auto P0=Ps.front();
		auto P1=Ps[Ps.size()/4];
		Geometry::normalizeProjectionMatrix(P0);
		Geometry::normalizeProjectionMatrix(P1);
		Eigen::Vector3d V0=Geometry::getCameraDirection(P0);
		Eigen::Vector3d V1=Geometry::getCameraDirection(P1);
		// Use some other projection (delta alpha<180°) to determine direction of rotation
		auto P2=Ps[Ps.size()/2];
		Geometry::normalizeProjectionMatrix(P2);
		Eigen::Vector3d V2=Geometry::getCameraDirection(P2);

		// Determine direction of rotation and correct axis of rotation, such that it is counter clock-wise
		Eigen::Vector3d plane_normal=rotation_plane.block<3,1>(0,0);
		Eigen::Matrix3d plane_proj=Eigen::Matrix3d::Identity()-plane_normal*plane_normal.transpose();
		bool plane_normal_flipped=plane_normal.dot((plane_proj*V1).cross(plane_proj*V2))<0;
		if (plane_normal_flipped) rotation_plane=-rotation_plane;
		plane_normal=rotation_plane.block<3,1>(0,0);
		if (plane_of_rotation) *plane_of_rotation=rotation_plane;

		// Projection to plane of rotation
		Eigen::Matrix3d primary_proj=Eigen::Matrix3d::Identity()-plane_normal*plane_normal.transpose();
		Eigen::Vector3d V0_in_plane=(primary_proj*V0).normalized();

		// Analyze trajectory
		for (int i=0;i<n;i++)
		{
			auto Pi=Ps[i];
			Geometry::normalizeProjectionMatrix(Pi);
			// Source position
			auto Ci=(*source_positions)[i];
			// Source to iso-center distance
			double sid=(Ci.block<3,1>(0,0)/Ci[3]).norm();
			if (source_to_iso_center_distances)
				(*source_to_iso_center_distances)[i]=sid;
			// Direction and projection to plane opf rotation
			Eigen::Vector3d Vi=Geometry::getCameraDirection(Pi).normalized();
			Eigen::Vector3d Vi_in_plane=(primary_proj*Vi).normalized();
			// Compute primary angle of rotation
			if (primary_angles)
			{
				double pa_cos=V0_in_plane.dot(Vi_in_plane);
				// fixme clamping should not be necessary
				if (pa_cos>1.0) pa_cos=1.0;
				if (pa_cos<-1.0) pa_cos=-1.0;
				double primary_angle=std::acos(pa_cos);
				if (V0_in_plane.cross(Vi_in_plane).dot(plane_normal)<0)
					primary_angle=2*Geometry::Pi-primary_angle;
				primary_angle *= 180.0/Geometry::Pi;
				(*primary_angles)[i]=primary_angle;
			}
			// Compute secondary angle of rotation
			if (secondary_angles)
			{
				// Compute secondary angle of rotation (fixme: clamping should not be necessary!)
				double cos_angle=Vi.dot(Vi_in_plane);
				if (cos_angle>1.0) cos_angle=1.0;
				if (cos_angle<-1.0) cos_angle=-1.0;
				// Store angle
				double secondary_angle=acos(cos_angle);
				if (plane_normal.dot(Vi.cross(Vi_in_plane))<0)
					secondary_angle*=-1;
				(*secondary_angles)[i]=secondary_angle;
			}
			// Compute source to detector distance
			if (source_to_detector_distances)
			{
				Eigen::Matrix3d K,R;
				Eigen::Vector3d t;
				Geometry::projectionMatrixDecomposition(Pi,K,R,t);
				double sdd=detector_pixel_spacing*0.5*(std::abs(K(0,0))+std::abs(K(1,1)));
				(*source_to_detector_distances)[i]=sdd;
			}
		}

	}

	// Create perfect circular Trajectory
	inline std::vector<Geometry::ProjectionMatrix> makeCircularTrajectory(int n_proj, double sid, double sdd, int n_u, int n_v, double max_angle, double pixel_spacing)
	{
		double detector_height=n_v*pixel_spacing;
		double fovy=std::atan(detector_height/sdd); //field of view

		Eigen::Matrix3d K=Geometry::cameraPerspective(fovy, n_u, n_v);

		Eigen::Matrix4d T_rot_x=Eigen::Matrix4d::Identity();

		using Geometry::Pi;
		double costheta=cos(0.5*Pi);
		double sintheta=sin(0.5*Pi);
		T_rot_x(1, 1)=costheta;
		T_rot_x(2, 2)=costheta;
		T_rot_x(1, 2)=-sintheta;
		T_rot_x(2, 1)=+sintheta;

		double primary_angle=0;
		std::vector<Geometry::ProjectionMatrix> Ps(n_proj);
		for (int i=0; i < n_proj; i++)
		{
			primary_angle=i*(double)(max_angle/n_proj)/180.0*Geometry::Pi; // convert from degree to rad
			Ps[i]=Geometry::cameraLookAt(K, Eigen::Vector3d(sid*std::cos(primary_angle), 0, sid*std::sin(primary_angle)), Eigen::Vector3d(0, 0, 0));
			Ps[i]=Ps[i]*T_rot_x;
			Geometry::normalizeProjectionMatrix(Ps[i]);
		}
		return Ps;
	}

	/// Read a text file with one matrix per line
	inline std::vector<Geometry::ProjectionMatrix> loadProjectionsOneMatrixPerLine(const std::string& file, std::map<std::string,std::string> *meta=0x0)
	{
		std::vector<Geometry::ProjectionMatrix> ret;
		std::ifstream pt(file);
		while (pt&&pt.good()&&!pt.eof())
		{
			std::string line;
			std::getline(pt,line);
			if (line[0]=='#')
			{
				if (meta && line[1]=='>')
					attrib_parse(line.substr(3),*meta);
				else if (meta // we also store the first comment
					&& meta->find("comment")==meta->end())
					(*meta)["comment"]=line.substr(1);
			}
			else if (!line.empty() &&  line[0]!='#')
				ret.push_back(stringTo<Geometry::ProjectionMatrix>(line));
		}
		return ret;
	}

	/// Can be used to store additional meta info in OMPL comments
	inline std::string toMetaAttrib(const std::map<std::string,std::string>& meta)
	{
		std::ostringstream strstr;
		for (auto it=meta.begin();it!=meta.end();++it)
			strstr << it->first << "=\"" << it->second << "\" ";
		return strstr.str();
	}

	/// Write a text file with one matrix per line and optional comments and meta info stored in those comments.
	inline bool saveProjectionsOneMatrixPerLine(const std::vector<Geometry::ProjectionMatrix>& Ps, const std::string& path,
		const std::string& first_line_comment="", // you can use: std::string("> ") + toMetaAttrib(meta_data)
		double spacing=0.0, const Eigen::Vector2i& detector_size_px=Eigen::Vector2i::Zero())
	{
		std::ofstream file(path);
		if (file)
		{
			if (!first_line_comment.empty())
				file << "#" << first_line_comment << std::endl;
			if (spacing!=0.0)
			{
				file << "#> " << "spacing=\"" << spacing << "\"";
				if (!detector_size_px.isZero()) file << " detector_size_px=\"" << toString(detector_size_px) << "\"";
				file << std::endl;
			}
			for (auto it=Ps.begin();it!=Ps.end();++it)
				file << toString(*it) << std::endl;
			return true;
		}
		else return false;
	}

	/// Read a Siemens projtable.xml file to a vector of projection matrices. V1.3 version.
	inline std::vector<Geometry::ProjectionMatrix> loadProjtableV13(const std::string& file)
	{
		std::vector<Geometry::ProjectionMatrix> ret;
		std::ifstream pt(file);
		while (pt&&pt.good()&&!pt.eof())
		{
			std::string line;
			std::getline(pt,line);
			trim(line);
			if (line=="<ProjectionMatrix>")
			// Found projection matrix
			{
				// Ignore everything, just get matrix
				Geometry::ProjectionMatrix P;
				for (int y=0;y<3;y++)
					for (int x=0;x<4;x++)
						pt >> P(y,x);
				ret.push_back(P);
			}
		}
		return ret;
	}

	/// Read a Siemens projtable.txt file to a vector of projection matrices
	inline std::vector<Geometry::ProjectionMatrix> loadProjtable(const std::string& file)
	{
		std::vector<Geometry::ProjectionMatrix> ret;
		std::ifstream pt(file);
		while (pt&&pt.good()&&!pt.eof())
		{
			std::string line;
			std::getline(pt,line);

			if (!line.empty() && line[0]=='@')
			// Found projection matrix
			{
				// Ignore primary and secondary angle
				std::getline(pt,line);
				Geometry::ProjectionMatrix P;
				for (int y=0;y<3;y++)
					for (int x=0;x<4;x++)
						pt >> P(y,x);
				ret.push_back(P);
			}
		}
		return ret;
	}

	/// Write projection matrices in a Siemens protable.txt format
	inline bool saveProjtable(const std::vector<Geometry::ProjectionMatrix>& Ps, const std::string& path)
	{
			// Analyze angles of trajectory
		std::vector<double> primary_angles,secondary_angles;
		ProjTable::ctCircularTrajectoryToParameters(Ps,1,0x0,&primary_angles,&secondary_angles);
		// Write matrices
		std::ofstream file(path);
		if (!file)
		 return false;

	 	file << "projtable version 3\n";
		file << "Mon Jan 01 1000 ompl2projtable\n" << " \n";
		file << "# format: angle/entries of projection matrices\n";
		file << toString(Ps.size()) << "\n" << "\n";

		for (int i=0;i<(int)Ps.size();i++)
		{
			auto Pi=Ps[i];
			Geometry::normalizeProjectionMatrix(Pi);
			Pi/=Pi(2,3);
			file	<< "@ " << i << "\n" << toString(primary_angles[i])+ " " + toString(secondary_angles[i])+" \n"
					 << Pi(0,0) << " " << Pi(0,1) << " " << Pi(0,2) << " " << Pi(0,3)  << "\n"
					 << Pi(1,0) << " " << Pi(1,1) << " " << Pi(1,2) << " " << Pi(1,3)  << "\n"
					 << Pi(2,0) << " " << Pi(2,1) << " " << Pi(2,2) << " " << Pi(2,3)  << "\n"
					<< std::endl; //changed by Lina since pull is not giving me the already corrected file (1.9)
		}
		return true;
	}

	/// EXPERIMENTAL Read a CONRAD configuration file, ignoring everything except "edu.stanford.rsl.conrad.geometry.Projection" arrays
	/// Expected format (anywhere in the XML):
	///<array class="edu.stanford.rsl.conrad.geometry.Projection" length="248">
	///   <void index="0">
	///    <object class="edu.stanford.rsl.conrad.geometry.Projection">
	///     <void property="PMatrixSerialization">
	///      <string>[[X X X X]; [X X X X]; [X X X X]]</string>
	///     </void>
	///    </object>
	///   </void>
	/// ...
	inline std::vector<Geometry::ProjectionMatrix> loadCONRAD(const std::string& file, GetSetGui::ProgressInterface *p=0x0)
	{
		std::vector<Geometry::ProjectionMatrix> ret;
		// Load and parse XML
		using GetSetInternal::UglyXML;
		UglyXML xml(fileReadString(file),p);
		// Find the first occurence of edu.stanford.rsl.conrad.geometry.Projection
		std::vector<UglyXML::Tag*> PMat_arrays=xml.dfs_search_tag("array","class","edu.stanford.rsl.conrad.geometry.Projection");
		if (PMat_arrays.empty()) {
			std::cerr << "No array of edu.stanford.rsl.conrad.geometry.Projection found.\n";
			return ret;
		}
		if (PMat_arrays.size()>1) std::cerr << "More that one array of edu.stanford.rsl.conrad.geometry.Projection found.\n   Using first one.\n";
		// Just iterate over all <string>...</string> tags
		UglyXML::Tag& PMat_array=*PMat_arrays.front();
		int n_expected=stringTo<int>(PMat_array.attrib["length"]);
		std::cout << "Trying to import " << n_expected << " edu.stanford.rsl.conrad.geometry.Projection objects...\n";
		// <void property="PMatrixSerialization">
		std::vector<UglyXML::Tag*> PMats=xml.dfs_search_tag("void","property","PMatrixSerialization");
		if ((int)PMats.size()!=n_expected)
			std::cerr << "Warning: found " << PMats.size() << " <string>...</string> tags!\n   Parsing is non-standard any probably returns invalid data.\n";
		// Convert string to ProjectionMatrix
		for (auto it=PMats.begin();it!=PMats.end();++it)
		{
			// Expecting exactly one <string>...</string> tag
			if ((*it)->tags.size()!=1 || (*it)->tags.front().tag!="string") {
				std::cerr << "Error: expecting exactly one <string>...</string> tag below a PMatrixSerialization tag...\n";
				ret.clear();
				return ret;
			}
			std::string& P_string=(*it)->tags[0].text;
			ret.push_back(stringTo<Geometry::ProjectionMatrix>(P_string));
		}
		return ret;
	}

} // namespace ProjTable

#endif // __projtable_hxx
