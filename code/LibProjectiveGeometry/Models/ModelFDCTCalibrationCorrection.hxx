// Created by Lina Felsner on Thu Aug 18th 2016

#ifndef __model_fdct_calibration_correction_hxx
#define __model_fdct_calibration_correction_hxx

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <LibProjectiveGeometry/ProjectionMatrix.h>
#include "Model.hxx"

#include <Utils/Projtable.hxx>

namespace Geometry {

	struct ModelFDCTCalibrationCorrection : public Model<7>
	{

		double pp_u;
		double pp_v;
		double spacing;
		double sid;
		double sdd;

		static std::vector<std::string> ParameterNames()
		{
			std::vector<std::string> names;
			// Set parameter names
			names.resize(size());
			names[0] = "Translation u";
			names[1] = "Translation v";
			names[2] = "Yaw";
			names[3] = "Pitch";
			names[4] = "Roll";
			names[5] = "Source Isocenter Distance";
			names[6] = "Source Detector Distance";
			return names;
		}

		enum ParameterSet { Identity, DetectorShifts, DetectorRigid2D, DetectorRotations, SDDandSID, All };
		static std::vector<std::string> ParameterSets()
		{
			std::vector<std::string> sets;
			sets[0] = "Identity";
			sets[1] = "DetectorShifts";
			sets[2] = "DetectorRigid2D";
			sets[4] = "DetectorRotations";
			sets[5] = "Source Detector and Iso-Center Distances";
			sets[6] = "All";
			return sets;
		}

		void computeMeanPPandSIDandSDD(const std::vector<Geometry::ProjectionMatrix> Ps)
		{
			std::vector<double> sids, sdds;
			ProjTable::ctCircularTrajectoryToParameters(Ps, spacing, 0x0, 0x0, 0x0, &sids, &sdds);

			// calculate mean
			double sumSID = std::accumulate(sids.begin(), sids.end(), 0.0);
			sid = sumSID / sids.size();
			double sumSDD = std::accumulate(sdds.begin(), sdds.end(), 0.0);
			sdd = sumSDD / sdds.size();

			std::cout << "Mean sid and sdd: " << sid << " " << sdd << std::endl;

			Geometry::RP2Point ppmean(0,0,0);
			for (int i = 0; i < (int)Ps.size(); i++)
			{
				Geometry::RP2Point pp=Ps[i].block<3, 3>(0, 0)*Ps[i].block<1, 3>(2, 0).transpose();
				pp = pp / pp(2);
				ppmean+=pp;
			}
			pp_u = ppmean[0] / ppmean(2);
			pp_v = ppmean[1] / ppmean(2);
			std::cout << "Mean principal point: " << pp_u << " " << pp_v << std::endl;
		}

		ModelFDCTCalibrationCorrection(int _nu, int _nv, double  _spacing, double sid, double sdd, ParameterSet active = Identity)
			: Model<7>(ParameterNames())
			, pp_u(0.5*_nu)
			, pp_v(0.5*_nv)
			, spacing(_spacing)
		{
			// Set parameter names
			names = ModelFDCTCalibrationCorrection::ParameterNames();
			// Set active parameters
			setActiveParameters(active);
		}

		ModelFDCTCalibrationCorrection(double  _spacing, const std::vector<Geometry::ProjectionMatrix>& Ps, ParameterSet active = Identity)
			: Model<7>(ParameterNames())
			, spacing(_spacing)
		{
			// Set parameter names
			names = ModelFDCTCalibrationCorrection::ParameterNames();
			// Set active parameters
			setActiveParameters(active);
			// Analyze geometry
			computeMeanPPandSIDandSDD(Ps);
		}

		/// Several common sets of active parameters
		void setActiveParameters(ParameterSet set)
		{
			for (int i = 0; i<size(); i++) active[i] = (set==All);
			
			switch (set)
			{	
				default:
				case Identity:
				case All:
					break;
				case DetectorShifts:
					active[0] = active[1] = true;
				break;
				case SDDandSID:
					active[5] = active[6] = true;
				break;
				case DetectorRotations:
					active[2]=active[3]=active[4]=true;
				break;
				case DetectorRigid2D:
					active[0] =active[1] = active[4] = true;
				break;
			}
		}

		// Transform the Projection Matrix via: H*H_init*P*T
		void transform(std::vector<Geometry::ProjectionMatrix> &Ps, Eigen::Matrix3d H_init = Eigen::Matrix3d::Identity()) const
		{

			//TODO 
			std::pair<Eigen::Matrix3d, Eigen::Matrix4d> HT = getTransforms(Ps);
			Eigen::Matrix3d H = HT.first;
			Eigen::Matrix4d T = HT.second;

			for (int i = 0; i < Ps.size(); i++)
			{
				Ps[i] = H*H_init*Ps[i]*T;
				Geometry::normalizeProjectionMatrix(Ps[i]);
			}
		}

		// Compose a homography from Parametrization
		std::pair<Eigen::Matrix3d, Eigen::Matrix4d> getTransforms(std::vector<Geometry::ProjectionMatrix> Ps) const
		{
			const double *x(param); //parameter vector

			Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
			Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

			Geometry::RP2Point ppmean(pp_u, pp_v, 1.0);

			double sid_scale = (sid + x[5]) / sid;
			double sdd_scale = (sdd + x[6]) / sdd;

//// H = Matrix 3x3
			Eigen::Matrix3d H_roll, H_shift, H_scale;
			H_roll = H_shift = H_scale = H;

			// using the principal point / center of the image
			auto Hpp = H, Hppinv = H;
			Hpp.block<3, 1>(0, 2) = ppmean;
			Hppinv(0, 2) = -ppmean[0];
			Hppinv(1, 2) = -ppmean[1];

			//Roll -> Rotation
			if (x[4] != 0)
			{
				H_roll <<
					+cos(x[4]), -sin(x[4]), 0,
					+sin(x[4]), +cos(x[4]), 0,
					0,			0,			1;
			}

			// Translation u and Yaw
			H_shift(0, 2) += x[0];
			H_shift(0, 2) += tan(x[2])*sdd;

			// Translation v and Pitch
			H_shift(1, 2) += x[1];
			H_shift(1, 2) += tan(x[3])*sdd;

			//SDD
			H_scale.block<2, 2>(0, 0) *= sdd_scale;

			H = H_shift * Hpp * H_roll * H_scale * Hppinv;

//// T = Matrix 4x4
			//SID
			T.block<3, 3>(0, 0) *= sid_scale;

//// HT
			std::pair<Eigen::Matrix3d, Eigen::Matrix4d> HT = { H, T };
			return HT;
		}

	};

}

#endif // __model_fdct_calibration_correction_hxx