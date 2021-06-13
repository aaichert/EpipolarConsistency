#ifndef hr_timer
#define hr_timer
// Created by aaichert on Sept 12th 2014
// WIN32-only high precision timer.

#ifdef WIN32
	#include <windows.h>
#endif 

namespace Utils {

	class TimerWin32 {
	private:
#ifdef WIN32
		struct {
			LARGE_INTEGER start;
			LARGE_INTEGER current;
		} timer;
		LARGE_INTEGER frequency;
#endif 

	public:
		TimerWin32(){
#ifdef WIN32
			timer.start.QuadPart=0;
			timer.current.QuadPart=0;
			QueryPerformanceFrequency( &frequency );
			startTimer();
#endif 
		}

		/// (Re-) Start Timer
		void startTimer() {
#ifdef WIN32
			QueryPerformanceCounter(&timer.start);
			timer.current.QuadPart=timer.start.QuadPart;
#endif 
		}

		// Elapsed Time in Seconds since last call to getElapsedTime() or startTimer()
		double getElapsedTime() {
#ifdef WIN32
			LARGE_INTEGER now;
			QueryPerformanceCounter(&now);
			LONGLONG elapsed=now.QuadPart - timer.current.QuadPart;
			timer.current=now;
			return (double)elapsed/(double)frequency.QuadPart;
#else
			return 0;
#endif
		}

		// Elapsed Time in Seconds since startTimer()
		double getTotalTime() {
#ifdef WIN32
			LARGE_INTEGER now;
			QueryPerformanceCounter(&now);
			LONGLONG elapsed=now.QuadPart-timer.start.QuadPart;
			return (double)elapsed/(double)frequency.QuadPart;
#else
			return 0;
#endif
		}
	};

} // namespace Utils

#endif
