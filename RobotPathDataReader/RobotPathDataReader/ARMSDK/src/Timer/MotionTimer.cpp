#include "Timer\MotionTimer.h"

namespace armsdk
{

	MotionTimer::MotionTimer()
	{
		mtime.start.QuadPart=0;
		mtime.end.QuadPart=0; 
		QueryPerformanceFrequency( &freq ) ;
	}

	MotionTimer::~MotionTimer(void)
	{
	}

	double MotionTimer::LI_to_msec(LARGE_INTEGER t)
	{
		return ((double)t.QuadPart / (double)freq.QuadPart)*1000.0;
	}

	void MotionTimer::Start()
	{
		DWORD_PTR oldmask = SetThreadAffinityMask(GetCurrentThread(), 0);
		QueryPerformanceCounter(&mtime.start);
		SetThreadAffinityMask(::GetCurrentThread(),0);
	}

	void MotionTimer::Stop()
	{
		DWORD_PTR oldmask = SetThreadAffinityMask(GetCurrentThread, 0);
		QueryPerformanceCounter(&mtime.end);
		SetThreadAffinityMask(::GetCurrentThread(),0);
	}

	void MotionTimer::Wait(double millisec)
	{
		if(millisec <0.0)
			return;

		MotionTimer mtimer;
		mtimer.Start();
		while(1)
		{
			mtimer.Stop();
			if(millisec < mtimer.GetElapsedTime())
				break;
		}
	}

	double MotionTimer::GetElapsedTime()
	{
		ElapsedTime.QuadPart = mtime.end.QuadPart - mtime.start.QuadPart;
		return LI_to_msec(ElapsedTime);
	}

}