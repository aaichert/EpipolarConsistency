

#include <iostream>
#include <LibProjectiveGeometry/ProjectiveGeometry.hxx>
#include <Utils/Projtable.hxx>

int main(int argc, char ** argv)
{
	if (argc!=2) return 1;
	auto Ps=ProjTable::loadProjectionsOneMatrixPerLine(argv[1]);
	ProjTable::saveProjtable(Ps,std::string(argv[1])+".txt");
	return 0;
}
