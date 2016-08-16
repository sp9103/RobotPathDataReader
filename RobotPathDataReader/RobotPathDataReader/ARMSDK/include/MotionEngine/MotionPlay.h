#ifndef __MotionPlay_H_
#define __MotionPlay_H_

#include "../ARMSDK_Math.h"
#include "Kinematics.h"
#include "TrajectoryGenerator.h"

namespace armsdk
{
	class MotionPlay
	{
	private:
		TrajectoryGenerator *mTraj;
		Kinematics *mRobot;
		MotionProfile *mMF;
		MotionProfile *mHandMF;
		vecd angleforplay;
		veci angleforhand;
		
		Pose3D sp, ep, vp, dp;
		Position3D cp;
		vecd xc, yc;
		double Length;
		
		int TimePeriod;
		double AdditionalCalcTime;
		
		int step, noMF;
		double donetime;
		double MotionTotalTime;

		int hstep, nohMF;
		double hdonetime;
		double CurrentTime, RemainingTime, ElapsedTime;
		
	public:
		MotionPlay(TrajectoryGenerator *mTraj);
		~MotionPlay(void);

		void All_Info_Reload(void);
		
		void Initialize(void);
		void Set_Time_Period(int MilliSecond);
		void Set_Additional_Calc_Time(double MilliSecond);
		
		vecd NextStepAtTime(double CurrentTime, int *ErrorStatus);
		veci NextStepAtTimeforHand(double CurrentTime);
		
		Pose3D InterpolationResult(double CurrentTime);
		vecd CalcIK(Pose3D desired, int *ErrorStatus);

		vecd NextStep(int *ErrorStatus);
		veci NextStepforHand(void);
		
		vecd GetCurrentAngle(void);
		Pose3D GetCurrentEndPose(void);
		double Get_CurrentTime(void);
		void Set_CurrentTime(double CurrentTimeInSec);

		vecd Get_ARM_1st_JointAngle(void);
	};
}

#endif