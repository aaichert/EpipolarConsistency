// Created by A. Aichert on Thu Jan 5th 2017
// RTK circular CT geometry. See also http://www.openrtk.org/Doxygen/geometry.pdf
#ifndef __model_circular_iec61217
#define __model_circular_iec61217

#include <sstream>

#include <LibOpterix/ParameterModel.hxx>
#include <LibProjectiveGeometry/ProjectionMatrix.h>

#define WITH_UGLY_XML_SUPPORT

#ifdef WITH_UGLY_XML_SUPPORT
#include <Utils/UglyXML.hxx>
#endif // WITH_UGLY_XML_SUPPORT

namespace Geometry {

	// Over-parametrization for a projective transformation of 2-space
	struct ModelTrajectoryCircularIEC61217 : public LibOpterix::ParameterModel<Geometry::ProjectionMatrix>
	{
		static const std::vector<std::string>& ParameterNames()
		{
			static std::vector<std::string> names;
			if (names.empty())
			{
				names.resize(9);
				names[0]="Gantry Angle";
				names[1]="In-plane Angle";
				names[2]="Out-of-plane Angle";
				names[3]="Source Offset X";
				names[4]="Source Offset Y";
				names[5]="Source-to-iso-center Distance";
				names[6]="Projection Offset X";
				names[7]="Projection Offset Y";
				names[8]="Source-to-detector Distance";
			}
			return names;
		}

		/// Tags for XML in/output by parameter
		static std::vector<std::string> ParameterTagsXML()
		{
			std::vector<std::string> names;
			if (names.empty())
			{
				names.resize(9);
				names[0]="GantryAngle";
				names[1]="InPlaneAngle";
				names[2]="OutOfPlaneAngle";
				names[3]="SourceOffsetX";
				names[4]="SourceOffsetY";
				names[5]="SourceToIsocenterDistance";
				names[6]="ProjectionOffsetX";
				names[7]="ProjectionOffsetY";
				names[8]="SourceToDetectorDistance";
			}
			return names;
		}

		ModelTrajectoryCircularIEC61217(std::set<int> _active=std::set<int>())
			: LibOpterix::ParameterModel<Geometry::ProjectionMatrix>(ParameterNames(),_active)
			//, image_size_u(0)
			//, image_size_v(0)
			, image_model_matrix_inverse(Geometry::RP2Homography::Identity())
		{}

		/////////////////////////////////////////////////
		/// Image coordinate system not defined per RTK
		/////////////////////////////////////////////////

		//double					image_size_u;
		//double					image_size_v;
		//inline const double& imageSizeU()	const	{return image_size_u;}
		//inline		 double& imageSizeU()			{return image_size_u;}
		//inline const double& imageSizeV()	const	{return image_size_v;}
		//inline		 double& imageSizeV()			{return image_size_v;}

		Geometry::RP2Homography image_model_matrix_inverse;
		inline const Geometry::RP2Homography&	getImageModelMatrixInverse()const {return image_model_matrix_inverse;}
		inline ModelTrajectoryCircularIEC61217&	setImageModelMatrixInverse(const Geometry::RP2Homography& v) {image_model_matrix_inverse=v; return *this;}


		////////////////////////////////////////
		/// Access to parameters for convenience.
		////////////////////////////////////////

		inline const double& getGantryAngle()				const {return current_values[0];}
		inline const double& getInPlaneAngle()				const {return current_values[1];}
		inline const double& getOutOfPlaneAngle()			const {return current_values[2];}
		inline const double& getSourceOffsetX()				const {return current_values[3];}
		inline const double& getSourceOffsetY()				const {return current_values[4];}
		inline const double& getSourceIsoCenterDistance()	const {return current_values[5];}
		inline const double& getProjectionOffsetX()			const {return current_values[6];}
		inline const double& getProjectionOffsetY()			const {return current_values[7];}
		inline const double& getSourceDetectorDistance()	const {return current_values[8];}
		inline ModelTrajectoryCircularIEC61217& setGantryAngle				(const double& v){current_values[0]=v;return *this;}
		inline ModelTrajectoryCircularIEC61217& setInPlaneAngle				(const double& v){current_values[1]=v;return *this;}
		inline ModelTrajectoryCircularIEC61217& setOutOfPlaneAngle			(const double& v){current_values[2]=v;return *this;}
		inline ModelTrajectoryCircularIEC61217& setSourceOffsetX			(const double& v){current_values[3]=v;return *this;}
		inline ModelTrajectoryCircularIEC61217& setSourceOffsetY			(const double& v){current_values[4]=v;return *this;}
		inline ModelTrajectoryCircularIEC61217& setSourceIsoCenterDistance	(const double& v){current_values[5]=v;return *this;}
		inline ModelTrajectoryCircularIEC61217& setProjectionOffsetX		(const double& v){current_values[6]=v;return *this;}
		inline ModelTrajectoryCircularIEC61217& setProjectionOffsetY		(const double& v){current_values[7]=v;return *this;}
		inline ModelTrajectoryCircularIEC61217& setSourceDetectorDistance	(const double& v){current_values[8]=v;return *this;}
		
		////////////////////////////////////////
		/// Assembly of projection matrix
		////////////////////////////////////////

		/// Compute matrix representing orientation of system
		Geometry::RP3Homography computeRotationMatrix() const
		{
			return Geometry::RotationZ(-getInPlaneAngle())*Geometry::RotationX(-getOutOfPlaneAngle())*Geometry::RotationY(-getGantryAngle());
		}

		/// Compute matrix representing orientation of system
		Geometry::ProjectionMatrix computeMagnificationMatrix() const
		{
			Geometry::ProjectionMatrix M=Geometry::ProjectionMatrix::Zero();
			if (getSourceDetectorDistance()==0)
			{
				// Parallel geometry
				M(0,0)=1.0;
				M(1,1)=1.0;
				M(2,2)=0.0;
				M(2,3)=1.0;
			}
			else
			{
				M(0,0)=-getSourceDetectorDistance();
				M(1,1)=-getSourceDetectorDistance();
				M(2,2)=1.0;
				M(2,3)=-getSourceIsoCenterDistance();
			}
			return M;
		}

		/// Compute RTK style projection matrix (everything in millimeters)
		Geometry::ProjectionMatrix getProjectionRTK() const 
		{
			return
				 Geometry::Translation(getSourceOffsetX()-getProjectionOffsetX(), getSourceOffsetY()-getProjectionOffsetY())
				*computeMagnificationMatrix()
				*Geometry::Translation(-getSourceOffsetX(), -getSourceOffsetY(), 0.0)
				*computeRotationMatrix();
		}
		
		/// Compute Projection Matrix
		virtual Geometry::ProjectionMatrix getInstance() const
		{
			return image_model_matrix_inverse*getProjectionRTK();
		}

		/// Utility function to convert values to XML (outputs only active parameters unless this_projection is unset)
		const std::string toXML(bool this_projection=true) const 
		{
			// Build explicit active set
			std::vector<bool> active(numberOfParameters(),false);
			for (auto it=active_parameters.begin();it!=active_parameters.end();++it)
				active[*it]=true;
			//  <?xml version="1.0"?>
			//  <!DOCTYPE RTKGEOMETRY>
			//  <RTKThreeDCircularGeometry version="3">
			//...
			//  "data[0].toXML(false)"
			//  "for each i"
			//  "data[i].toXML()"
			//...
			//  </RTKThreeDCircularGeometry>
			auto tags=ParameterTagsXML();
			std::ostringstream xml;
			if (!this_projection)
			{
				// First three parameters are angles and are stored in degrees in xml file
				for (int i=0;i<3;i++)
					if (!active[i]) xml << "  <" << tags[i] << ">" << std::setw(18) << current_values[i]/Geometry::Pi*180 << "</" << tags[i] << ">\n";
				for (int i=3;i<9;i++)
					if (!active[i]) xml << "  <" << tags[i] << ">" << std::setw(18) <<  current_values[i] << "</" << tags[i] << ">\n";
			}
			else
			{
				auto Pi=getProjectionRTK();
				xml << "  <Projection>\n";
				// First three parameters are angles and are stored in degrees in xml file
				for (int i=0;i<3;i++)
					if (active[i]) xml << "    <" << tags[i] << ">" << std::setw(18) << current_values[i]/Geometry::Pi*180 << "</" << tags[i] << ">\n";
				for (int i=3;i<9;i++)
					if (active[i]) xml << "    <" << tags[i] << ">" << std::setw(18) << current_values[i] << "</" << tags[i] << ">\n";
				xml << "    <Matrix>\n" << std::setw(18)
					<< "        " << Pi(0,0) << " " << Pi(0,1) << " " << Pi(0,2) << " " << Pi(0,3) << "\n"
					<< "        " << Pi(1,0) << " " << Pi(1,1) << " " << Pi(1,2) << " " << Pi(1,3) << "\n"
					<< "        " << Pi(2,0) << " " << Pi(2,1) << " " << Pi(2,2) << " " << Pi(2,3) << "\n"
					<< "    </Matrix>\n";
				xml << "  </Projection>\n";
			}
			return xml.str();
		}

	};

	inline bool saveCircularTrajectoryRTK(const std::string& xml_file, const std::vector<ModelTrajectoryCircularIEC61217>& rtk_params)
	{
		std::ofstream xml(xml_file);
		if (!xml) return false;
		xml <<
				"<?xml version=\"1.0\"?>\n"
				"<!DOCTYPE RTKGEOMETRY>\n"
				"<RTKThreeDCircularGeometry version=\"3\">\n";
		xml << rtk_params.front().toXML(false);
		for (int i=0;i<(int)rtk_params.size();i++)
			xml << rtk_params[i].toXML();
		xml <<	"</RTKThreeDCircularGeometry>\n";
		return true;
	}

#ifdef WITH_UGLY_XML_SUPPORT

	inline std::vector<ModelTrajectoryCircularIEC61217> loadCircularTrajectoryRTK(const std::string& xml_file)
	{
		// XML data will be read into stencil and projections
		Geometry::ModelTrajectoryCircularIEC61217 stencil;
		// Build explicit active set
		std::vector<bool> active(stencil.numberOfParameters(),false);
		std::vector<Geometry::ModelTrajectoryCircularIEC61217> projections;
		std::vector<Geometry::ProjectionMatrix>      projection_matrices_mm; // not needed, except for asserting correct loading
		// Parse XML contents
		GetSetInternal::UglyXML xml(fileReadString(xml_file));
		if (xml.tags.size()!=1
			|| xml.tags.front().tag!="RTKThreeDCircularGeometry"
			|| xml.tags.front().attrib.size()!=1
			//|| xml.tags.front().attrib.begin()->first!="version"
			//|| stringTo<double>(xml.tags.front().attrib.begin()->second)<=2
			)
		{
			std::cerr <<
				"Only very specific files are supported by this implementation.\n"
				"XML should have one root tag <RTKThreeDCircularGeometry version=\"2\">\n";
			return projections;
		}
		// Build a map from known XML tags to parameters index in ModelCircularIEC61217
		auto xml_tags=Geometry::ModelTrajectoryCircularIEC61217::ParameterTagsXML();
		std::map <std::string,int> parameter_index_by_xml_tag;
		for (int i=0;i<(int)xml_tags.size();i++)
			parameter_index_by_xml_tag[xml_tags[i]]=i;
		
		// Only the single RTKThreeDCircularGeometry tag will be used.
		auto& rtk_geom=xml.tags.front().tags;
		for (int i=0;i<(int)rtk_geom.size();i++)
		{
			// If a tag like <InPlaneAngle>, <SourceOffsetX> is given outside of a <Projection> tag,
			// we store the values in stencil, which we use as default values for <Projection>s. 
			// If however we are dealing with a <Projection>, we store values in a new element
			if (rtk_geom[i].tag=="Projection")
			{
				projections.push_back(stencil);
				// Loop over all children (only one level)
				for (int j=0;j<(int)rtk_geom[i].tags.size();j++)
				{
					if (rtk_geom[i].tags[j].tag=="Matrix")
					{
						projection_matrices_mm.push_back(stringTo<Geometry::ProjectionMatrix>(rtk_geom[i].tags[j].text));
						continue;
					}
					// Figure out index of associated parameer and parse value
					if (parameter_index_by_xml_tag.find(rtk_geom[i].tags[j].tag)==parameter_index_by_xml_tag.end())
						std::cerr << "RTKThreeDCircularGeometry: unknown projection tag: " << rtk_geom[i].tags[j].tag << std::endl;
					else
					{
						int p=parameter_index_by_xml_tag[rtk_geom[i].tag];
						projections.back().current_values[p]=stringTo<double>(rtk_geom[i].tags[j].text);
						active[p]=true;
						if (p<3) // parameter is an angle and needs conversion to radians
							projections.back().current_values[p]*=Geometry::Pi/180;
					}
				}
			}
			else
			{
				// Figure out index of associated parameer and parse value
				if (parameter_index_by_xml_tag.find(rtk_geom[i].tag)==parameter_index_by_xml_tag.end())
					std::cerr << "RTKThreeDCircularGeometry: unknown tag: " << rtk_geom[i].tag << std::endl;
				else
				{
					int p=parameter_index_by_xml_tag[rtk_geom[i].tag];
					stencil.current_values[p]=stringTo<double>(rtk_geom[i].text);
					if (p<3) // parameter is an angle and needs conversion to radians
						stencil.current_values[p]/=180*Geometry::Pi;
				}
			}
		}
		//// Verify loaded parameters with matrices
		// (2do make sure all active parameters are equal)
		if (projection_matrices_mm.size()==projections.size())
			for (int i=0;i<(int)projections.size();i++)
			{
				std::cout << "####### " << i << std::endl;
				std::cout << projections[i].getProjectionRTK() << std::endl;
				std::cout << std::endl;
				std::cout << projection_matrices_mm[i]<< std::endl;
			}
		// If only global tags were discovered
		if (projections.empty()) projections.push_back(stencil);
		return projections;
	}
#endif // WITH_UGLY_XML_SUPPORT

} // namespace Geometry 

#endif // __model_circular_iec61217

/*

cx=sym('cx')
cy=sym('cy')
sdd=sym('sdd')
sid=sym('sid')
detx=sym('detx')
dety=sym('dety')

ipa=sym('ipa')
oop=sym('oop')
alpha=sym('alpha')

# Def according to rtk
Rz=[ cos(-ipa)   -sin(-ipa)    0 0;
	 sin(-ipa)    cos(-ipa)    0 0;
	 0            0            1 0;
	 0            0            0 1];

Rx=[ 1 0            0          0;
	 0 cos(-oop)   -sin(-oop)  0;
	 0 sin(-oop)    cos(-oop)  0;
	 0 0            0          1]

Ry=[ cos(-alpha)   0     sin(-alpha)  0;
	 0             1     0            0;
	-sin(-alpha)   0     cos(-alpha)  0;
	 0             0     0            1]

MR=Rz*Rx*Ry

MSP=[1 0 cx-detx ;
     0 1 cy-dety ;
     0 0 1 ]

MSDD=[sid 0   0 0;
      0   sid 0 0;
      0   0   1 sid]

MSO=[1 0 0 -cx;
     0 1 0 -cy;
     0 0 1  0;
     0 0 0  1 ]
 
MP=MSP*MSDD*MSO


*/
