#ifndef __progress_interface
#define __progress_interface

#include <iostream>
class ProgressInterface {
	public:
		virtual void progressStart(const std::string& progress, const std::string& info, int maximum, bool *cancel_clicked=0x0)
			{ std::cout << progress << ": " << info << std::endl; }

		virtual void progressUpdate(int i) { std::cout << i << "..." << std::endl; }
		virtual void progressEnd() { std::cout << "Done." << std::endl; }

		virtual void info(const std::string& who, const std::string& what, bool show_dialog=false)
			{ std::cout << who << ": " << what << std::endl; }
		virtual void warn(const std::string& who, const std::string& what, bool only_inormative=true)
			{ std::cout << (only_inormative?"Warning - ":"Error - ") << who << ": " << what << std::endl; }
};

#endif // __progress_interface
