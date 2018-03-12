#ifndef hr_timer
#define hr_timer
// Created by aaichert on Sept 12th 2014
// WIN32-only high precision timer.

#include <windows.h>

namespace Utils {

	class TimerWin32 {
	private:
		struct {
			LARGE_INTEGER start;
			LARGE_INTEGER current;
		} timer;
		LARGE_INTEGER frequency;

	public:
		TimerWin32(){
			timer.start.QuadPart=0;
			timer.current.QuadPart=0;
			QueryPerformanceFrequency( &frequency );
			startTimer();
		}

		/// (Re-) Start Timer
		void startTimer() {
			QueryPerformanceCounter(&timer.start);
			timer.current.QuadPart=timer.start.QuadPart;
		}

		// Elapsed Time in Seconds since last call to getElapsedTime() or startTimer()
		double getElapsedTime() {
			LARGE_INTEGER now;
			QueryPerformanceCounter(&now);
			LONGLONG elapsed=now.QuadPart - timer.current.QuadPart;
			timer.current=now;
			return (double)elapsed/(double)frequency.QuadPart;
		}

		// Elapsed Time in Seconds since startTimer()
		double getTotalTime() {
			LARGE_INTEGER now;
			QueryPerformanceCounter(&now);
			LONGLONG elapsed=now.QuadPart-timer.start.QuadPart;
			return (double)elapsed/(double)frequency.QuadPart;

		}
	};

} // namespace Utils

#endif
