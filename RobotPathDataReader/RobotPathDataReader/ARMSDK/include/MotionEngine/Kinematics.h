#ifndef __KINEMATICS_H_
#define __KINEMATICS_H_

#include <vector>
#include <iostream>
#include "RobotInfo.h"
#include "Error.h"

namespace armsdk
{
	class Kinematics
	{
	private:
		vector<JointData>* mRobotInfo;
		RobotInfo* mRobot;
		vector<matd> mRobotMat;
		Pose3D currentpose;
	//	Pose3D_Quaternion currentpose_Quaternion;
		vecd currentangle;
		unsigned int DOF;

	public:
		Kinematics(RobotInfo *_mRobot);
		Kinematics(){};
		~Kinematics();
		
		void RobotInfoReload(void);
		void InitRobot(RobotInfo *_mRobot);

		matd Forward(vecd angle);
		matd Forward(vecd angle, Pose3D *pose);
		void EndAxis(vecd angle, Pose3D *pose, Pose3D *x, Pose3D *y, Pose3D *z);		//Normalization 안된 결과
		//matd Forward(vecd angle, Pose3D_Quaternion *pose);
		matd Jacobian(void);
		vecd CalcError(Pose3D _desired, matd _current);
		void ComputeIK(Pose3D _desired, vecd *q, vecd Initangle, int *ErrorStatus);

		unsigned int GetNumberofJoint(void);
		vecd* GetCurrentAngle(void);
		Pose3D* GetCurrentPose(void);
		//Pose3D_Quaternion* GetCurrettPose_Quaternion(void);
		RobotInfo* GetRobotInfo(void);
		veci Rad2Value(vecd q);
		vecd Value2Rad(veci q);
		veci Get_IDList(void);
	};
}
#endif 