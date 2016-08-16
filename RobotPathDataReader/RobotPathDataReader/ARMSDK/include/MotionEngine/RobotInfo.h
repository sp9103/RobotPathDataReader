#ifndef __ROBOTINFO_H_
#define __ROBOTINFO_H_
#include "JointData.h"
#include "../ARMSDK_Define.h"
#include "../ARMSDK_Math.h"
#include <vector>

using namespace std;

namespace armsdk
{
	class RobotInfo
	{
	private:
		vector<JointData> mRobotInfo;

	public:
		RobotInfo();
		~RobotInfo();

		int AddJoint (double LinkLength, double LinkTwist, double JointOffset, double JointAngle, 
			double MaxAngleInRad, double MinAngleInRad, 
			int MaxAngleValue , int MinAngleValue, double MaxAngleLimitInRad, double MinAngleLimitInRad, unsigned int ID);

		JointData *GetJointInfo(int i);
		vector<JointData>* GetRobotInfo(void);
		void ClearRobotInfo(void);
		veci GetJointIDList(void);
	};
}
#endif
