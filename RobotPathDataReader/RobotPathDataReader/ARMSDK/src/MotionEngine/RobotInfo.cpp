#include "MotionEngine\RobotInfo.h"

namespace armsdk
{
	RobotInfo::RobotInfo()
	{	}

	RobotInfo::~RobotInfo()
	{	}

	int RobotInfo::AddJoint(double LinkLength, double LinkTwist, double JointOffset, double JointAngle, 
		double MaxAngleInRad, double MinAngleInRad, 
		int MaxAngleValue , int MinAngleValue, double MaxAngleLimitInRad, double MinAngleLimitInRad, unsigned int ID)
	{
		if(MaxAngleInRad < MinAngleInRad)
			return 1;
		if(MaxAngleValue < MinAngleValue)
			return 1;

		JointData* Jnt =  new JointData;
		Jnt->SetJointDataDH(LinkLength, LinkTwist, JointOffset, JointAngle);
		Jnt->SetMaxAngleInRad(MaxAngleInRad);
		Jnt->SetMinAngleInRad(MinAngleInRad);
		Jnt->SetMaxAngleInValue(MaxAngleValue);
		Jnt->SetMinAngleInValue(MinAngleValue);
		Jnt->SetJointID(ID);
		Jnt->SetMaxAngleLimitInRad(MaxAngleLimitInRad);
		Jnt->SetMinAngleLimitInRad(MinAngleLimitInRad);

		mRobotInfo.push_back(*Jnt);
		delete Jnt;

		return 0;
	}

	JointData* RobotInfo::GetJointInfo(int i)
	{
		return &mRobotInfo[i];
	}

	vector<JointData>* RobotInfo::GetRobotInfo(void)
	{
		return &mRobotInfo;
	}

	void RobotInfo::ClearRobotInfo(void)
	{
		mRobotInfo.clear();
	}

	veci RobotInfo::GetJointIDList(void)
	{
		veci ID;
		ID.resize(mRobotInfo.size());
		for(int i =0; i < ID.size() ; i++)
		{
			ID[i] = mRobotInfo[i].GetID();
		}
		return ID;
	}

}