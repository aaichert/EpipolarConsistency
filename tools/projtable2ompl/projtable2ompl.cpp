

#include <iostream>
#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>
#include <Utils\Projtable.hxx>

int main(int argc, char ** argv)
{
	if (argc!=2) return 1;
	auto Ps=ProjTable::loadProjtable(argv[1]);
	ProjTable::saveProjectionsOneMatrixPerLine(Ps,std::string(argv[1])+".ompl");
	return 0;
}
