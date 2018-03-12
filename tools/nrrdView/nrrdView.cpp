// Qt5
#include <QApplication>
#include <QFileDialog>

// Utility
#include <LibUtilsQt/Figure.hxx>

#include <fstream>
#include <vector>

int main(int argc, char* argv[])
{
	QApplication app(argc,argv);
	if (argc==1) {
		std::string path=QFileDialog::getOpenFileName(0x0, "Open NRRD Image", "", "NRRD Image (*.nrrd);;All Files (*)").toStdString();
		if (path.empty()) return 2;
		NRRD::Image<float> img(path);
		if (!img) return 3;
		UtilsQt::Figure(path,img);
	}
	else if (argc==2) {		
		NRRD::Image<float> img(argv[1]);
		if (!img) return 4;
		UtilsQt::Figure(argv[1],img);
	}
	else {
		std::cerr << "Usage: nrrdView image.nrrd\n";
		return 0;
	}
	return app.exec();
}
