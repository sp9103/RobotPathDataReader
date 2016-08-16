#ifndef __TRAJECTORYGENERATOR_H_
#define __TRAJECTORYGENERATOR_H_

#include "../ARMSDK_Define.h"
#include "../ARMSDK_Math.h"
#include "Kinematics.h"
#include <vector>

using namespace std;

namespace armsdk
{
	class TrajectoryGenerator
	{
	private:
		vector<timeprofile> mTF;
		MotionProfile mMF;
		MotionPoseList PoseList;
		vecd *angleforplanning;
		vecd angleforstart;

		MotionProfile mHandMF;
		Kinematics *mRobot;

	public:
		TrajectoryGenerator(Kinematics *mRobot);
		~TrajectoryGenerator();

		void KinematicsInfoReload(void);

		void Set_P2P(vecd StartPose, vecd EndPose,  double TotalTime = 1.0, double AccelTime = 0.256);
		void Set_P2P(Pose3D StartPose, Pose3D EndPose,  double TotalTime = 1.0, double AccelTime = 0.256);

		void Set_LIN(vecd StartPose, vecd EndPose, double TotalTime = 1.0, double AccelTime = 0.256);
		void Set_LIN(Pose3D StartPose, Pose3D EndPose, double TotalTime = 1.0, double AccelTime = 0.256);

		void Set_CIRC(vecd StartPose, vecd ViaPose, vecd EndPose, double TotalTime = 1.0, double AccelTime = 0.256);
		void Set_CIRC(Pose3D StartPose, Pose3D ViaPose, Pose3D EndPose, double TotalTime = 1.0, double AccelTime = 0.256);


		//with Hand Profile
		void Set_P2PwithHand(vecd StartPose, vecd EndPose, veci Hand1, veci Hand2, double TotalTime = 1.0, double AccelTime = 0.256);
		void Set_P2PwithHand(Pose3D StartPose, Pose3D EndPose, veci Hand1, veci Hand2, double TotalTime = 1.0, double AccelTime = 0.256);
					
		void Set_LINwithHand(vecd StartPose, vecd EndPose, veci Hand1, veci Hand2, double TotalTime = 1.0, double AccelTime = 0.256);
		void Set_LINwithHand(Pose3D StartPose, Pose3D EndPose, veci Hand1, veci Hand2, double TotalTime = 1.0, double AccelTime = 0.256);

		void Set_CIRCwithHand(vecd StartPose, vecd ViaPose, vecd EndPose, veci Hand1, veci Hand2, double TotalTime = 1.0, double AccelTime = 0.256);
		void Set_CIRCwithHand(Pose3D StartPose, Pose3D ViaPose, Pose3D EndPose, veci Hand1, veci Hand2, double TotalTime = 1.0, double AccelTime = 0.256);

		void ClearMF(void);

		double GetMotionTotalTime(void);

		vecd GetAngleforStart(void);
		MotionProfile* GetMotionProfile(void);
		MotionProfile* GetHandMotionProfile(void);
		MotionPoseList* GetMotionPoseList(void);

		Kinematics* GetKinematics(void);
	};
}

#endif