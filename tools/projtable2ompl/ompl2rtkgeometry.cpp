
#include <iostream>
#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>
#include <Utils/Projtable.hxx>

int usage()
{
	std::cerr <<
		"Usage:\n"
		"   ompl2rtkgeometry file.ompl\n"
		"   ompl2rtkgeometryfile.ompl --sdd <source-detector-distance>\n"
//		"   ompl2rtkgeometryfile.ompl --px2mm <detect>\n"
		;
	return 1;
}

int main(int argc, char ** argv)
{
	double sdd=0;
	if (argc == 4)
	{
		if (argv[2]=="--sdd")
			sdd=std::abs(stringTo<double>(argv[3]));
		else if (argv[2]=="--px2mm")
		{
			sdd=-std::abs(stringTo<double>(argv[3]));
			std::cerr << "Error: --px2mm not implemented.\n";
			return 2;
		}
		else usage();
	}
	else if (argc!=2) return usage();
	auto Ps=ProjTable::loadProjectionsOneMatrixPerLine(argv[1]);
	if (sdd<0 && !Ps.empty())
	{
		Eigen::Matrix3d K,R;
		Eigen::Vector3d t;
		Geometry::projectionMatrixDecomposition(Ps.front(),K,R,t);
		sdd*=-0.5*(std::abs(K(0,0))+std::abs(K(1,1)));
	}
//	ProjTable::saveProjectionsRTK_circular(Ps,std::string(argv[1])+".rtk",sdd);
	return 0;
}
