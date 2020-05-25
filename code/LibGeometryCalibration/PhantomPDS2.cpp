#include "PhantomPDS2.h"


/// f(x)=1-x^2+x^4 is zero at +/-1, has zero derivative at +/-1 and a maxiumum at f(0)=1; Values outside [-1,1] return zero. 
inline double weighting(double x)
{
	if (x<-1.0||x>1.0) return 0;
	double xx=x*x;
	return 1.0-2*xx+xx*xx;
}

namespace Calibration {

	const std::vector<bool>& PhantomDescriptorPDS2::defaultBeadSequence()
	{
		static std::vector<bool> default_sequence;
		if (default_sequence.empty())
			default_sequence=stringToVector<bool>(
				"false;false;false;false;false;false;false;true ;false;false;false;true ;"
				"false;true ;false;false;true ;true ;false;false;false;true ;true ;true ;"
				"true ;true ;false;true ;false;false;false;true ;true ;false;true ;false;"
				"true ;false;false;false;false;false;true ;true ;true ;false;false;true ;"
				"true ;false;true ;true ;true ;false;false;false;true ;false;false;true ;"
				"true ;true ;false;true ;false;true ;true ;false;true ;true ;false;false;"
				"false;false;true ;false;true ;true ;false;false;true ;false;false;true ;"
				"false;true ;false;true ;false;true ;true ;true ;true ;false;false;true ;"
				"false;true ;true ;true ;false;true ;true ;true ;true ;true ;true ;true  "
				,';');
		return default_sequence;
	}


	std::vector<Eigen::Vector4d> PhantomDescriptorPDS2::getBeads() const
	{
		// Copyright notice: This code has been adapted from CONRAD - https://www5.cs.fau.de/conrad/
		int n=(int)largeBeadSequence.size();
		std::vector<Eigen::Vector4d> ret(n);
		for (int i=0; i<n; i++){
			ret[i][0] = helix.radius*std::cos(i*-helix.angularIncrement);
			ret[i][1] = helix.radius*std::sin(i*-helix.angularIncrement);
			ret[i][2] = helix.height*0.5-helix.offset-beads.axialIncrement*i;
			ret[i][3] = largeBeadSequence[i]?beads.largeDiameter:beads.smallDiameter;
		}
		return ret;
	}
	
	int PhantomDescriptorPDS2::computeBeadNumber(const std::vector<bool>& code, const std::vector<bool>& largeBeadSequence)
	{
		// Copyright notice: This code has been copyied from CONRAD - https://www5.cs.fau.de/conrad/
		int numberOfBeads=(int)largeBeadSequence.size();
		int revan = -1;
		for (int i = 0; i < numberOfBeads; i++)
		{
			bool hit = true;
			for (int j = 0; j < (int)code.size(); j++)
			{
				if (j+i < numberOfBeads) {
					if (code[j] != largeBeadSequence[i+j]) hit = false;
				}
				else {
					// can't match if sequence exceeds the bead sequence.
					hit = false;
				}
			}
			// found a hit in the sequence.
			if (hit) revan = i;
		}
		return revan; // FIXME: i.e. -1?
	}
	

	void PhantomDescriptorPDS2::gui_declare_section (const GetSetGui::Section& section)
	{
		GetSet<double>("Helix/Height"             ,section,helix.height          );
		GetSet<double>("Helix/Offset"             ,section,helix.offset          );
		GetSet<double>("Helix/Radius"             ,section,helix.radius          );
		GetSet<double>("Helix/Angular Increment"  ,section,helix.angularIncrement);
		GetSet<double>("Cylinder/Outer Radius"    ,section,cylinder.outerRadius  );
		GetSet<double>("Cylinder/Inner Radius"    ,section,cylinder.innerRadius  );
		GetSet<double>("Cylinder/Density"         ,section,cylinder.density      );
		GetSet<double>("Beads/Axial Increment"    ,section,beads.axialIncrement  );
		GetSet<double>("Beads/Diameter/Small"     ,section,beads.smallDiameter   );
		GetSet<double>("Beads/Diameter/Large"     ,section,beads.largeDiameter   );
		GetSet<double>("Beads/Density"            ,section,beads.density         );
		GetSet<std::vector<bool> >("Bead Sequence",section,largeBeadSequence     );
		GetSetGui::Section("Beads"                ,section).setGrouped();
		GetSetGui::Section("Beads/Diameter"       ,section).setGrouped();
	}

	void PhantomDescriptorPDS2::gui_retreive_section(const GetSetGui::Section& section)
	{
		helix.height          =GetSet<double>("Helix/Height"           ,section);
		helix.offset          =GetSet<double>("Helix/Offset"           ,section);
		helix.radius          =GetSet<double>("Helix/Radius"           ,section);
		helix.angularIncrement=GetSet<double>("Helix/Angular Increment",section);
		cylinder.outerRadius  =GetSet<double>("Cylinder/Outer Radius"  ,section);
		cylinder.innerRadius  =GetSet<double>("Cylinder/Inner Radius"  ,section);
		cylinder.density      =GetSet<double>("Cylinder/Density"       ,section);
		beads.axialIncrement  =GetSet<double>("Beads/Axial Increment"  ,section);
		beads.smallDiameter   =GetSet<double>("Beads/Diameter/Small"   ,section);
		beads.largeDiameter   =GetSet<double>("Beads/Diameter/Large"   ,section);
		beads.density         =GetSet<double>("Beads/Density"          ,section);
		largeBeadSequence=GetSet<std::vector<bool> >("Bead Sequence"   ,section);
	}

	std::map<int,int> PhantomDetectionPDS2::match(const std::vector<Eigen::Vector3d>& detected_beads) const
	{
		// Figure out how to separate large and small beads.
		double bead_radius_threshold_mm = computeRadiusThresholdMM(beads);
		double bead_radius_threshold_px = computeRadiusThresholdPx(detected_beads);
		// Figure out sequence of large and small beads.
		std::vector<bool> largeBeadSequence(beads.size());
		for (int i=0;i<(int)beads.size();i++)
			largeBeadSequence[i]=beads[i][3]>bead_radius_threshold_mm;

		// Set up storage and 
		int n=(int)detected_beads.size();
		int n_angular_bins=36; // bins per 180°
		NRRD::Image<float>           bead_distance(n,n);
		NRRD::Image<Eigen::Vector2d> bead_connection(n,n);
		std::vector<int>             bead_closest(n);
		for (int i=0;i<n;i++) bead_distance.pixel(i,i)=0;
		// Pre-compute distance and direction between all beads.
		for (int i=0;i<n;i++)
		{
			for (int j=i;j<n;j++)
			{
				auto& dir=bead_connection.pixel(j,i)=detected_beads[i].head(2)-detected_beads[j].head(2);
				bead_connection.pixel(i,j)=-dir;
				double distance=(dir).norm();
				bead_distance.pixel(i,j)=(float)distance;
				bead_distance.pixel(j,i)=(float)distance;
			}
		}
		// Find closest beads
		for (int i=0;i<n;i++)
		{
			double closest_distance;
			int closest_index=-1;
			for (int j=0;j<n;j++)
				if (i==j) continue;
				else if (closest_index<0||closest_distance>bead_distance(i,j)) {
					closest_index=j;
					closest_distance=bead_distance(i,j);
				}
			bead_closest[i]=closest_index;
		}

		debug_dirs.clear(); // DELETE just for debugging

		// Slower but a little more accurate
		bool use_majority_voting=true;
		// Majority voting for index based on detected bead sequence
		std::map<int, std::map<int,int> > candidate_votes;
		// Final matching result
		std::map<int,int> index_2d_to_3d;
		// Figure out local bead sequence base on direction of closest bead.
		for (int i=0;i<n;i++)
		{
			Eigen::Vector2d dir=bead_connection(i,bead_closest[i]);
			debug_dirs.push_back(dir);
			dir.normalize();
			Eigen::Vector2d dir_ortho(dir[1],-dir[0]);
			// Sort all beads by distance
			std::map<double,int> index_by_signed_distance;
			for (int j=0;j<n;j++)
			{
				// Project connecting vector from bead i to j w.r.t. local angle
				Eigen::Vector2d connection=bead_connection(i,j);
				double along_spiral=dir.dot(connection);
				double ortho_spiral=dir_ortho.dot(connection);
				double angle=abs(atan(ortho_spiral/along_spiral))/Geometry::Pi*180;
				// Ignore points not roughly on the spiral or too close
				double min_distance=1+detected_beads[i][2]+detected_beads[j][2];
				if ( i!=j && (angle>paramMatching.angular_tolerance || abs(along_spiral)<min_distance)) continue;
				index_by_signed_distance[along_spiral]=j;
			}
			// Store contents of index_by_signed_distance as an index lut
			int first_positive=-1;
			std::vector<int> point_sequence(index_by_signed_distance.size());
			{
				int i=0;
				for (auto it=index_by_signed_distance.begin();it!=index_by_signed_distance.end();++it) {
					if (first_positive==-1 && it->first>-1) first_positive=i; // -1px for some slack
					point_sequence[i++]=it->second;
				}
			}
			// Make sure we have 4 beads in either direction
			if (first_positive<4) continue;
			if (first_positive+4>(int)point_sequence.size()) continue;
			// Store 4-neightborhood in bool array based on large/small bead radius
			std::vector<bool> fwd(8);
			for (int d=-4;d<4;d++) {
				int point_index=point_sequence[first_positive+d];
				fwd[d+4]=detected_beads[point_index][2]>=bead_radius_threshold_px;
			}
			// Reverse sequence (in case the spiral is bottom-up or image is flipped
			std::vector<bool> bwd(8);
			for (int r=0;r<8;r++) bwd[7-r]=fwd[r];
			// Decode both bead-sequences and increase candidate votes
			int first_fwd=PhantomDescriptorPDS2::computeBeadNumber(fwd,largeBeadSequence);
			int first_bwd=PhantomDescriptorPDS2::computeBeadNumber(bwd,largeBeadSequence);

			// Store result directly (used if use_majority_voting is false)
			if (first_fwd>0)
				index_2d_to_3d[i]=first_fwd+4;
			else if (first_bwd>0)
				index_2d_to_3d[i]=first_bwd+3;
			// Actual voting if desired.
			if (use_majority_voting)
			{
				for (int d=-4;d<4;d++)
				{
					int current_sequence_index=first_positive+d;
					int current_bead_index=point_sequence[current_sequence_index];
					int vote_fwd=first_fwd+d+4;
					int vote_bwd=first_bwd-d+3;
					if (first_fwd>=0)
					{
						// if this is the first "vote" we start counting from zero.
						if (candidate_votes[current_bead_index].find(vote_fwd)==candidate_votes[current_bead_index].end())
							candidate_votes[current_bead_index][vote_fwd]=0;
						// Increate vote by distance from central bead.
						if(vote_fwd>0) candidate_votes[current_bead_index][vote_fwd]+=std::abs(d);
					}
					if (first_bwd>=0)
					{
						if (candidate_votes[current_bead_index].find(vote_bwd)==candidate_votes[current_bead_index].end())
							candidate_votes[current_bead_index][vote_bwd]=0;
						// Increate vote by distance from central bead.
						if(vote_bwd>0) candidate_votes[current_bead_index][vote_bwd]+=std::abs(d);
					}
				}
			}
		}

		// Find best indices. For equal number of votes, take first best.
		if (use_majority_voting)
		{
			for (auto it=candidate_votes.begin();it!=candidate_votes.end();++it)
			{
				int max_vote=-1;
				int best_match=-1;
				for (auto vote=it->second.begin();vote!=it->second.end();++vote)
				{
					if (max_vote<vote->second)
					{
						max_vote=vote->second;
						best_match=vote->first;
					}
				}
				index_2d_to_3d[it->first]=best_match;
			}
		}
		return index_2d_to_3d;
	}

	void PhantomDetectionPDS2::gui_declare_section(const GetSetGui::Section& section)
	{
		BeadPhantomDetection::gui_declare_section (section);
		GetSet<double>("Matching PDS2/Distance Min"             ,section,paramMatching.distance_min         ).setDescription("Min. distance of closest neighboring bead");
		GetSet<double>("Matching PDS2/Angular Tolerance deg"    ,section,paramMatching.angular_tolerance    ).setDescription("Neighborhood: Max. degrees deviation from line");
		GetSetGui::Section("Matching PDS2",section).setGrouped();
	}

	void PhantomDetectionPDS2::gui_retreive_section(const GetSetGui::Section& section)
	{
		BeadPhantomDetection::gui_retreive_section(section);
		paramMatching.distance_min         =GetSet<double>("Matching PDS2/Distance Min"             ,section);
		paramMatching.angular_tolerance    =GetSet<double>("Matching PDS2/Angular Tolerance deg"    ,section);
	}

}  // namespace Calibration
