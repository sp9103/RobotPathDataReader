#ifndef __MOTIONTIMER_H
#define __MOTIONTIMER_H

#include <afx.h>
#include <cmath>
#pragma comment(lib, "winmm.lib")

namespace armsdk
{
	class MotionTimer
	{
	private:
		typedef struct
		{
			LARGE_INTEGER start;
			LARGE_INTEGER end;
		}time;

		double LI_to_msec(LARGE_INTEGER t);

		time mtime;
		LARGE_INTEGER freq;
		LARGE_INTEGER ElapsedTime;

	public:
		MotionTimer();
		~MotionTimer();

		void Start();
		void Stop();
		void Wait(double millisec);
		double GetElapsedTime();
	};



}

#endif