#include "MotionEngine\MotionPlay.h"

namespace armsdk
{
	MotionPlay::MotionPlay(TrajectoryGenerator *mTraj)
	{
		TimePeriod= 8;
		CurrentTime = 0.0;
		AdditionalCalcTime = 0;
		ElapsedTime = 0.0;
		
		this->mTraj = mTraj;
		mMF=mTraj->GetMotionProfile();
		mRobot = this->mTraj->GetKinematics();
		angleforplay = (this->mTraj->GetAngleforStart());
		angleforhand.resize(3);
		mHandMF = mTraj->GetHandMotionProfile();
		
		noMF = mMF->size();
		donetime = 0.0;
		step = 1;
		CurrentTime = 0.0;
		MotionTotalTime = mTraj->GetMotionTotalTime();

		hstep = 1;
		nohMF = mHandMF->size();
		hdonetime = 0.0;

		xc.resize(3);
		yc.resize(3);
	}

	MotionPlay::~MotionPlay(void)
	{	}

	void MotionPlay::All_Info_Reload(void)
	{
		TimePeriod= 8;
		CurrentTime = 0.0;
		AdditionalCalcTime = 0;
		ElapsedTime = 0.0;
		
		mRobot->RobotInfoReload();
		mTraj->KinematicsInfoReload();
		mMF=mTraj->GetMotionProfile();

		angleforplay = (this->mTraj->GetAngleforStart());
		angleforhand.resize(3);
		
		mHandMF = mTraj->GetHandMotionProfile();
		noMF = mMF->size();

		donetime = 0.0;
		step = 1;
		CurrentTime = 0.0;

		hstep = 1;
		nohMF = mHandMF->size();
		hdonetime = 0.0;
	}

	void MotionPlay::Initialize(void)
	{
		noMF = mMF->size();
		donetime = 0.0;
		step = 1;
		CurrentTime = 0.0;

		hstep = 1;
		nohMF = mHandMF->size();
		hdonetime = 0.0;
	}

	void MotionPlay::Set_Time_Period(int MilliSecond)
	{
		if(TimePeriod == 0)
			TimePeriod = 8;
		else
			TimePeriod = MilliSecond;
	}

	void MotionPlay::Set_Additional_Calc_Time(double MilliSecond)
	{
		AdditionalCalcTime = MilliSecond;
	}

	vecd MotionPlay::NextStepAtTime(double CurrentTime, int *ErrorStatus)
	{
		if(CurrentTime - donetime > (*mMF)[step-1][0].totoaltime )
		{
			donetime += (*mMF)[step-1][0].totoaltime;
			
			if(mMF->size() > step)
				step +=1;
		}

		if(donetime >= MotionTotalTime)
			CurrentTime = donetime;
		else
			CurrentTime -= donetime;

		if(step > noMF)
		{
			*ErrorStatus |= ARMSDK_NO_ERROR;
			return angleforplay;
		}

		if((*mMF)[step-1][0].Method == Linear )
		{	
			//if( CurrentTime <= TimePeriod*0.001 )
			//{
			//	sp=(*mTraj->GetMotionPoseList())[step-1].StartPose3D;
			//	ep=(*mTraj->GetMotionPoseList())[step-1].EndPose3D;

			//	Length = (*mMF)[step-1][0].distance;

			//	angleforplay = (*mTraj->GetMotionPoseList())[step-1].StartPose;
			//	*ErrorStatus = NoError;
			//	return angleforplay;
			//}
			//else

			if(donetime <0.000001 && CurrentTime < TimePeriod*0.001)
			{
				sp=(*mTraj->GetMotionPoseList())[step-1].StartPose3D;
				ep=(*mTraj->GetMotionPoseList())[step-1].EndPose3D;

				Length = (*mMF)[step-1][0].distance;

				angleforplay = (*mTraj->GetMotionPoseList())[step-1].StartPose;
				*ErrorStatus |= ARMSDK_NO_ERROR;
				return angleforplay;
			}
			else if(CurrentTime < TimePeriod*0.001)
			{
				sp=(*mTraj->GetMotionPoseList())[step-1].StartPose3D;
				ep=(*mTraj->GetMotionPoseList())[step-1].EndPose3D;

				Length = (*mMF)[step-1][0].distance;

				mRobot->ComputeIK(sp, &angleforplay, angleforplay, ErrorStatus);
				*ErrorStatus |= ARMSDK_NO_ERROR;
				return angleforplay;
			}
			else
			{
				if(CurrentTime >= TimePeriod*0.001  && CurrentTime < (*mMF)[step-1][0].ta)
				{
					double ds = ((*mMF)[step-1][0].a2[0]*CurrentTime*CurrentTime + (*mMF)[step-1][0].a1[0] *CurrentTime + (*mMF)[step-1][0].a0[0])/Length;
					dp.x = sp.x+ds*(ep.x-sp.x);
					dp.y = sp.y+ds*(ep.y-sp.y);
					dp.z = sp.z+ds*(ep.z-sp.z);
					dp.Roll = sp.Roll+ds*(ep.Roll-sp.Roll);
					dp.Pitch = sp.Pitch+ds*(ep.Pitch-sp.Pitch);
					dp.Yaw = sp.Yaw+ds*(ep.Yaw-sp.Yaw);

					mRobot->ComputeIK(dp, &angleforplay, angleforplay, ErrorStatus);
					return angleforplay;
				}
				else if(CurrentTime >= (*mMF)[step-1][0].ta && CurrentTime < (*mMF)[step-1][0].ta + (*mMF)[step-1][0].tc)
				{
					double ds = ((*mMF)[step-1][0].a2[1]*CurrentTime*CurrentTime + (*mMF)[step-1][0].a1[1] *CurrentTime + (*mMF)[step-1][0].a0[1])/Length;
					dp.x = sp.x+ds*(ep.x-sp.x);
					dp.y = sp.y+ds*(ep.y-sp.y);
					dp.z = sp.z+ds*(ep.z-sp.z);
					dp.Roll = sp.Roll+ds*(ep.Roll-sp.Roll);
					dp.Pitch = sp.Pitch+ds*(ep.Pitch-sp.Pitch);
					dp.Yaw = sp.Yaw+ds*(ep.Yaw-sp.Yaw);
					mRobot->ComputeIK(dp, &angleforplay, angleforplay, ErrorStatus);
					return angleforplay;
				}
				else if(CurrentTime >= (*mMF)[step-1][0].ta + (*mMF)[step-1][0].tc  && CurrentTime < (*mMF)[step-1][0].totoaltime)
				{
					double ds = ((*mMF)[step-1][0].a2[2]*CurrentTime*CurrentTime + (*mMF)[step-1][0].a1[2] *CurrentTime + (*mMF)[step-1][0].a0[2])/Length;
					dp.x = sp.x+ds*(ep.x-sp.x);
					dp.y = sp.y+ds*(ep.y-sp.y);
					dp.z = sp.z+ds*(ep.z-sp.z);
					dp.Roll = sp.Roll+ds*(ep.Roll-sp.Roll);
					dp.Pitch = sp.Pitch+ds*(ep.Pitch-sp.Pitch);
					dp.Yaw = sp.Yaw+ds*(ep.Yaw-sp.Yaw);

					mRobot->ComputeIK(dp, &angleforplay, angleforplay, ErrorStatus);
					return angleforplay;
				}
				else
				{
					*ErrorStatus |= ARMSDK_NO_ERROR;
					return angleforplay;
				}
			}

		}
		else if((*mMF)[step-1][0].Method == PtoP)
		{
			if( CurrentTime <= TimePeriod*0.001 )
			{
				angleforplay = (*mTraj->GetMotionPoseList())[step-1].StartPose;
				*ErrorStatus |= ARMSDK_NO_ERROR;
				return angleforplay;
			}
			else
			{
				if(CurrentTime > TimePeriod*0.001 && CurrentTime < (*mMF)[step-1][0].ta)
				{
					for(unsigned int i=0; i<mRobot->GetNumberofJoint();i++)
					{
						(angleforplay)(i) = (*mMF)[step-1][i].a2[0]*CurrentTime*CurrentTime + (*mMF)[step-1][i].a1[0]*CurrentTime + (*mMF)[step-1][i].a0[0];
					}
					mRobot->Forward(angleforplay);
					*ErrorStatus |= ARMSDK_NO_ERROR;
					return angleforplay;
				}
				else if(CurrentTime >= (*mMF)[step-1][0].ta && CurrentTime < (*mMF)[step-1][0].ta + (*mMF)[step-1][0].tc)
				{
					for(unsigned int i=0; i<mRobot->GetNumberofJoint();i++)
					{
						(angleforplay)(i) = (*mMF)[step-1][i].a2[1]*CurrentTime*CurrentTime + (*mMF)[step-1][i].a1[1]*CurrentTime + (*mMF)[step-1][i].a0[1];
					}
					mRobot->Forward(angleforplay);
					*ErrorStatus |= ARMSDK_NO_ERROR;
					return angleforplay;
				}
				else if(CurrentTime >= (*mMF)[step-1][0].ta + (*mMF)[step-1][0].tc  && CurrentTime < (*mMF)[step-1][0].totoaltime)
				{
					for(unsigned int i=0; i<mRobot->GetNumberofJoint();i++)
					{
						(angleforplay)(i) = (*mMF)[step-1][i].a2[2]*CurrentTime*CurrentTime + (*mMF)[step-1][i].a1[2]*CurrentTime + (*mMF)[step-1][i].a0[2];
					}
					mRobot->Forward(angleforplay);
					*ErrorStatus |= ARMSDK_NO_ERROR;
					return angleforplay;
				}
				else
				{
					mRobot->Forward(angleforplay);
					*ErrorStatus |= ARMSDK_NO_ERROR;
					return angleforplay;
				}
			}
		}
		else
		{
			if( CurrentTime <= TimePeriod*0.001 )
			{
				sp=(*mTraj->GetMotionPoseList())[step-1].StartPose3D;
				ep=(*mTraj->GetMotionPoseList())[step-1].EndPose3D;
				cp=(*mTraj->GetMotionPoseList())[step-1].CenterPosition;
				vp=(*mTraj->GetMotionPoseList())[step-1].ViaPose3D;
				
				Length = (*mMF)[step-1][0].distance;

				angleforplay = (*mTraj->GetMotionPoseList())[step-1].StartPose;
				*ErrorStatus |= ARMSDK_NO_ERROR;

				xc(0) = sp.x - cp.x;
				xc(1) = sp.y - cp.y;
				xc(2) = sp.z - cp.z;

				yc(0) = vp.x - cp.x;
				yc(1) = vp.y - cp.y;
				yc(2) = vp.z - cp.z;

				yc = yc - (yc.dot(xc))*xc;
				yc = yc * xc.norm() / yc.norm();

				return angleforplay;
			}
			else
			{
				if(CurrentTime > TimePeriod*0.001 && CurrentTime < (*mMF)[step-1][0].ta)
				{
					double ds = ((*mMF)[step-1][0].a2[0]*CurrentTime*CurrentTime + (*mMF)[step-1][0].a1[0] *CurrentTime + (*mMF)[step-1][0].a0[0]);
					if( ds < (*mMF)[step-1][0].distance1)
					{
						dp.x = cos(ds) * xc(0) + sin(ds) * yc(0) + cp.x;
						dp.y = cos(ds) * xc(1) + sin(ds) * yc(1) + cp.y;
						dp.z = cos(ds) * xc(2) + sin(ds) * yc(2) + cp.z;
						dp.Roll  = sp.Roll+ds*(vp.Roll-sp.Roll)/(*mMF)[step-1][0].distance1;
						dp.Pitch = sp.Pitch+ds*(vp.Pitch-sp.Pitch)/(*mMF)[step-1][0].distance1;
						dp.Yaw   = sp.Yaw+ds*(vp.Yaw-sp.Yaw)/(*mMF)[step-1][0].distance1;
					}
					else
					{
						dp.x = cos(ds) * xc(0) + sin(ds) * yc(0) + cp.x;
						dp.y = cos(ds) * xc(1) + sin(ds) * yc(1) + cp.y;
						dp.z = cos(ds) * xc(2) + sin(ds) * yc(2) + cp.z;
						dp.Roll  = vp.Roll  +   (ep.Roll-vp.Roll) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
						dp.Pitch = vp.Pitch + (ep.Pitch-vp.Pitch) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
						dp.Yaw   = vp.Yaw   +   (ep.Yaw - vp.Yaw) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
					}

					mRobot->ComputeIK(dp, &angleforplay, angleforplay, ErrorStatus);
					return angleforplay;
				}
				else if(CurrentTime >= (*mMF)[step-1][0].ta && CurrentTime < (*mMF)[step-1][0].ta + (*mMF)[step-1][0].tc)
				{
					double ds = ((*mMF)[step-1][0].a2[1]*CurrentTime*CurrentTime + (*mMF)[step-1][0].a1[1] *CurrentTime + (*mMF)[step-1][0].a0[1]);
					if( ds < (*mMF)[step-1][0].distance1)
					{
						dp.x = cos(ds) * xc(0) + sin(ds) * yc(0) + cp.x;
						dp.y = cos(ds) * xc(1) + sin(ds) * yc(1) + cp.y;
						dp.z = cos(ds) * xc(2) + sin(ds) * yc(2) + cp.z;
						dp.Roll  = sp.Roll+ds*(vp.Roll-sp.Roll)/(*mMF)[step-1][0].distance1;
						dp.Pitch = sp.Pitch+ds*(vp.Pitch-sp.Pitch)/(*mMF)[step-1][0].distance1;
						dp.Yaw   = sp.Yaw+ds*(vp.Yaw-sp.Yaw)/(*mMF)[step-1][0].distance1;
					}
					else
					{
						dp.x = cos(ds) * xc(0) + sin(ds) * yc(0) + cp.x;
						dp.y = cos(ds) * xc(1) + sin(ds) * yc(1) + cp.y;
						dp.z = cos(ds) * xc(2) + sin(ds) * yc(2) + cp.z;
						dp.Roll  = vp.Roll  +   (ep.Roll-vp.Roll) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
						dp.Pitch = vp.Pitch + (ep.Pitch-vp.Pitch) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
						dp.Yaw   = vp.Yaw   +   (ep.Yaw - vp.Yaw) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
					}

					mRobot->ComputeIK(dp, &angleforplay, angleforplay, ErrorStatus);
					return angleforplay;
				}
				else if(CurrentTime >= (*mMF)[step-1][0].ta + (*mMF)[step-1][0].tc  && CurrentTime < (*mMF)[step-1][0].totoaltime)
				{
					double ds = ((*mMF)[step-1][0].a2[2]*CurrentTime*CurrentTime + (*mMF)[step-1][0].a1[2] *CurrentTime + (*mMF)[step-1][0].a0[2]);
					if( ds < (*mMF)[step-1][0].distance1)
					{
						dp.x = cos(ds) * xc(0) + sin(ds) * yc(0) + cp.x;
						dp.y = cos(ds) * xc(1) + sin(ds) * yc(1) + cp.y;
						dp.z = cos(ds) * xc(2) + sin(ds) * yc(2) + cp.z;
						dp.Roll  = sp.Roll+ds*(vp.Roll-sp.Roll)/(*mMF)[step-1][0].distance1;
						dp.Pitch = sp.Pitch+ds*(vp.Pitch-sp.Pitch)/(*mMF)[step-1][0].distance1;
						dp.Yaw   = sp.Yaw+ds*(vp.Yaw-sp.Yaw)/(*mMF)[step-1][0].distance1;
					}
					else
					{
						dp.x = cos(ds) * xc(0) + sin(ds) * yc(0) + cp.x;
						dp.y = cos(ds) * xc(1) + sin(ds) * yc(1) + cp.y;
						dp.z = cos(ds) * xc(2) + sin(ds) * yc(2) + cp.z;
						dp.Roll  = vp.Roll  +   (ep.Roll-vp.Roll) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
						dp.Pitch = vp.Pitch + (ep.Pitch-vp.Pitch) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
						dp.Yaw   = vp.Yaw   +   (ep.Yaw - vp.Yaw) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
					}

					mRobot->ComputeIK(dp, &angleforplay, angleforplay, ErrorStatus);
					return angleforplay;
				}
				else
				{
					*ErrorStatus |= ARMSDK_NO_ERROR;
					return angleforplay;
				}
			}
		}
	}

	veci MotionPlay::NextStepAtTimeforHand(double CurrentTime)
	{

		if(CurrentTime - hdonetime > (*mHandMF)[hstep-1][0].totoaltime )
		{
			hdonetime += (*mHandMF)[hstep-1][0].totoaltime;
			hstep +=1;
		}

		if(hdonetime >= MotionTotalTime)
			CurrentTime = hdonetime;
		else
			CurrentTime -= hdonetime;

		if(hstep >nohMF)
		{
			return angleforhand;
		}

		if( CurrentTime <= TimePeriod*0.001 )
		{
			angleforhand(0) = (int)(*mHandMF)[hstep-1][0].a0[0];
			angleforhand(1) = (int)(*mHandMF)[hstep-1][1].a0[0];
			angleforhand(2) = (int)(*mHandMF)[hstep-1][2].a0[0];
			return angleforhand;
		}
		else
		{
			if(CurrentTime > TimePeriod*0.001 && CurrentTime < (*mHandMF)[hstep-1][0].ta)
			{
				for(unsigned int i=0; i < 3 ;i++)
				{
					angleforhand(i) = (*mHandMF)[hstep-1][i].a2[0]*CurrentTime*CurrentTime + (*mHandMF)[hstep-1][i].a1[0]*CurrentTime + (*mHandMF)[hstep-1][i].a0[0];
				}
				return angleforhand;
			}
			else if(CurrentTime >= (*mHandMF)[hstep-1][0].ta && CurrentTime < (*mHandMF)[hstep-1][0].ta + (*mHandMF)[hstep-1][0].tc)
			{
				for(unsigned int i=0; i<3;i++)
				{
					angleforhand(i) = (*mHandMF)[hstep-1][i].a2[1]*CurrentTime*CurrentTime + (*mHandMF)[hstep-1][i].a1[1]*CurrentTime + (*mHandMF)[hstep-1][i].a0[1];
				}
				return angleforhand;
			}
			else if(CurrentTime >= (*mHandMF)[hstep-1][0].ta + (*mHandMF)[hstep-1][0].tc  && CurrentTime < (*mHandMF)[hstep-1][0].totoaltime)
			{
				for(unsigned int i=0; i<3;i++)
				{
					angleforhand(i) = (*mHandMF)[hstep-1][i].a2[2]*CurrentTime*CurrentTime + (*mHandMF)[hstep-1][i].a1[2]*CurrentTime + (*mHandMF)[hstep-1][i].a0[2];
				}
				return angleforhand;
			}
			else
			{
				return angleforhand;
			}
		}
	}

	Pose3D MotionPlay::InterpolationResult(double CurrentTime)
	{
		if(CurrentTime - donetime > (*mMF)[step-1][0].totoaltime )
		{
			donetime += (*mMF)[step-1][0].totoaltime;
			step +=1;
		}

		CurrentTime -= donetime;

		if(step > noMF)
		{
			Pose3D pose;
			mRobot->Forward(angleforplay, &pose);
			return pose;
		}

		if((*mMF)[step-1][0].Method == Linear )
		{	
			if( CurrentTime <= TimePeriod*0.001 )
			{
				sp=(*mTraj->GetMotionPoseList())[step-1].StartPose3D;
				ep=(*mTraj->GetMotionPoseList())[step-1].EndPose3D;

				Length = (*mMF)[step-1][0].distance;

				angleforplay = (*mTraj->GetMotionPoseList())[step-1].StartPose;
				
				Pose3D pose;
				mRobot->Forward(angleforplay, &pose);
				
				return pose;
			}
			else
			{
				if(CurrentTime > TimePeriod*0.001 && CurrentTime < (*mMF)[step-1][0].ta)
				{
					double ds = ((*mMF)[step-1][0].a2[0]*CurrentTime*CurrentTime + (*mMF)[step-1][0].a1[0] *CurrentTime + (*mMF)[step-1][0].a0[0])/Length;
					dp.x = sp.x+ds*(ep.x-sp.x);
					dp.y = sp.y+ds*(ep.y-sp.y);
					dp.z = sp.z+ds*(ep.z-sp.z);
					dp.Roll = sp.Roll+ds*(ep.Roll-sp.Roll);
					dp.Pitch = sp.Pitch+ds*(ep.Pitch-sp.Pitch);
					dp.Yaw = sp.Yaw+ds*(ep.Yaw-sp.Yaw);
					return dp;
				}
				else if(CurrentTime >= (*mMF)[step-1][0].ta && CurrentTime < (*mMF)[step-1][0].ta + (*mMF)[step-1][0].tc)
				{
					double ds = ((*mMF)[step-1][0].a2[1]*CurrentTime*CurrentTime + (*mMF)[step-1][0].a1[1] *CurrentTime + (*mMF)[step-1][0].a0[1])/Length;
					dp.x = sp.x+ds*(ep.x-sp.x);
					dp.y = sp.y+ds*(ep.y-sp.y);
					dp.z = sp.z+ds*(ep.z-sp.z);
					dp.Roll = sp.Roll+ds*(ep.Roll-sp.Roll);
					dp.Pitch = sp.Pitch+ds*(ep.Pitch-sp.Pitch);
					dp.Yaw = sp.Yaw+ds*(ep.Yaw-sp.Yaw);
					return dp;
				}
				else if(CurrentTime >= (*mMF)[step-1][0].ta + (*mMF)[step-1][0].tc  && CurrentTime < (*mMF)[step-1][0].totoaltime)
				{
					double ds = ((*mMF)[step-1][0].a2[2]*CurrentTime*CurrentTime + (*mMF)[step-1][0].a1[2] *CurrentTime + (*mMF)[step-1][0].a0[2])/Length;
					dp.x = sp.x+ds*(ep.x-sp.x);
					dp.y = sp.y+ds*(ep.y-sp.y);
					dp.z = sp.z+ds*(ep.z-sp.z);
					dp.Roll = sp.Roll+ds*(ep.Roll-sp.Roll);
					dp.Pitch = sp.Pitch+ds*(ep.Pitch-sp.Pitch);
					dp.Yaw = sp.Yaw+ds*(ep.Yaw-sp.Yaw);

					return dp;
				}
				else
				{
					return dp;
				}
			}

		}
		else if((*mMF)[step-1][0].Method == PtoP)
		{
			if( CurrentTime <= TimePeriod*0.001 )
			{
				angleforplay = (*mTraj->GetMotionPoseList())[step-1].StartPose;
				
				Pose3D pose;
				mRobot->Forward(angleforplay, &pose);
				
				return pose;
			}
			else
			{
				if(CurrentTime > TimePeriod*0.001 && CurrentTime < (*mMF)[step-1][0].ta)
				{
					for(unsigned int i=0; i<mRobot->GetNumberofJoint();i++)
					{
						(angleforplay)(i) = (*mMF)[step-1][i].a2[0]*CurrentTime*CurrentTime + (*mMF)[step-1][i].a1[0]*CurrentTime + (*mMF)[step-1][i].a0[0];
					}

					Pose3D pose;
					mRobot->Forward(angleforplay, &pose);
				
					return pose;
				}
				else if(CurrentTime >= (*mMF)[step-1][0].ta && CurrentTime < (*mMF)[step-1][0].ta + (*mMF)[step-1][0].tc)
				{
					for(unsigned int i=0; i<mRobot->GetNumberofJoint();i++)
					{
						(angleforplay)(i) = (*mMF)[step-1][i].a2[1]*CurrentTime*CurrentTime + (*mMF)[step-1][i].a1[1]*CurrentTime + (*mMF)[step-1][i].a0[1];
					}

					Pose3D pose;
					mRobot->Forward(angleforplay, &pose);
				
					return pose;
				}
				else if(CurrentTime >= (*mMF)[step-1][0].ta + (*mMF)[step-1][0].tc  && CurrentTime < (*mMF)[step-1][0].totoaltime)
				{
					for(unsigned int i=0; i<mRobot->GetNumberofJoint();i++)
					{
						(angleforplay)(i) = (*mMF)[step-1][i].a2[2]*CurrentTime*CurrentTime + (*mMF)[step-1][i].a1[2]*CurrentTime + (*mMF)[step-1][i].a0[2];
					}

					Pose3D pose;
					mRobot->Forward(angleforplay, &pose);
				
					return pose;
				}
				else
				{
					Pose3D pose;
					mRobot->Forward(angleforplay, &pose);
				
					return pose;
				}
			}
		}
		else
		{
			if( CurrentTime <= TimePeriod*0.001 )
			{
				sp=(*mTraj->GetMotionPoseList())[step-1].StartPose3D;
				ep=(*mTraj->GetMotionPoseList())[step-1].EndPose3D;
				cp=(*mTraj->GetMotionPoseList())[step-1].CenterPosition;
				vp=(*mTraj->GetMotionPoseList())[step-1].ViaPose3D;
				
				Length = (*mMF)[step-1][0].distance;

				angleforplay = (*mTraj->GetMotionPoseList())[step-1].StartPose;

				xc(0) = sp.x - cp.x;
				xc(1) = sp.y - cp.y;
				xc(2) = sp.z - cp.z;

				yc(0) = vp.x - cp.x;
				yc(1) = vp.y - cp.y;
				yc(2) = vp.z - cp.z;

				yc = yc - (yc.dot(xc))*xc;
				yc = yc * xc.norm() / yc.norm();


				Pose3D pose;
				mRobot->Forward(angleforplay, &pose);
				
				return pose;
			}
			else
			{
				if(CurrentTime > TimePeriod*0.001 && CurrentTime < (*mMF)[step-1][0].ta)
				{
					double ds = ((*mMF)[step-1][0].a2[0]*CurrentTime*CurrentTime + (*mMF)[step-1][0].a1[0] *CurrentTime + (*mMF)[step-1][0].a0[0]);
					if( ds < (*mMF)[step-1][0].distance1)
					{
						dp.x = cos(ds) * xc(0) + sin(ds) * yc(0) + cp.x;
						dp.y = cos(ds) * xc(1) + sin(ds) * yc(1) + cp.y;
						dp.z = cos(ds) * xc(2) + sin(ds) * yc(2) + cp.z;
						dp.Roll  = sp.Roll+ds*(vp.Roll-sp.Roll)/(*mMF)[step-1][0].distance1;
						dp.Pitch = sp.Pitch+ds*(vp.Pitch-sp.Pitch)/(*mMF)[step-1][0].distance1;
						dp.Yaw   = sp.Yaw+ds*(vp.Yaw-sp.Yaw)/(*mMF)[step-1][0].distance1;
					}
					else
					{
						dp.x = cos(ds) * xc(0) + sin(ds) * yc(0) + cp.x;
						dp.y = cos(ds) * xc(1) + sin(ds) * yc(1) + cp.y;
						dp.z = cos(ds) * xc(2) + sin(ds) * yc(2) + cp.z;
						dp.Roll  = vp.Roll  +   (ep.Roll-vp.Roll) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
						dp.Pitch = vp.Pitch + (ep.Pitch-vp.Pitch) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
						dp.Yaw   = vp.Yaw   +   (ep.Yaw - vp.Yaw) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
					}

					return dp;
				}
				else if(CurrentTime >= (*mMF)[step-1][0].ta && CurrentTime < (*mMF)[step-1][0].ta + (*mMF)[step-1][0].tc)
				{
					double ds = ((*mMF)[step-1][0].a2[1]*CurrentTime*CurrentTime + (*mMF)[step-1][0].a1[1] *CurrentTime + (*mMF)[step-1][0].a0[1]);
					if( ds < (*mMF)[step-1][0].distance1)
					{
						dp.x = cos(ds) * xc(0) + sin(ds) * yc(0) + cp.x;
						dp.y = cos(ds) * xc(1) + sin(ds) * yc(1) + cp.y;
						dp.z = cos(ds) * xc(2) + sin(ds) * yc(2) + cp.z;
						dp.Roll  = sp.Roll+ds*(vp.Roll-sp.Roll)/(*mMF)[step-1][0].distance1;
						dp.Pitch = sp.Pitch+ds*(vp.Pitch-sp.Pitch)/(*mMF)[step-1][0].distance1;
						dp.Yaw   = sp.Yaw+ds*(vp.Yaw-sp.Yaw)/(*mMF)[step-1][0].distance1;
					}
					else
					{
						dp.x = cos(ds) * xc(0) + sin(ds) * yc(0) + cp.x;
						dp.y = cos(ds) * xc(1) + sin(ds) * yc(1) + cp.y;
						dp.z = cos(ds) * xc(2) + sin(ds) * yc(2) + cp.z;
						dp.Roll  = vp.Roll  +   (ep.Roll-vp.Roll) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
						dp.Pitch = vp.Pitch + (ep.Pitch-vp.Pitch) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
						dp.Yaw   = vp.Yaw   +   (ep.Yaw - vp.Yaw) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
					}

					return dp;
				}
				else if(CurrentTime >= (*mMF)[step-1][0].ta + (*mMF)[step-1][0].tc  && CurrentTime < (*mMF)[step-1][0].totoaltime)
				{
					double ds = ((*mMF)[step-1][0].a2[2]*CurrentTime*CurrentTime + (*mMF)[step-1][0].a1[2] *CurrentTime + (*mMF)[step-1][0].a0[2]);
					if( ds < (*mMF)[step-1][0].distance1)
					{
						dp.x = cos(ds) * xc(0) + sin(ds) * yc(0) + cp.x;
						dp.y = cos(ds) * xc(1) + sin(ds) * yc(1) + cp.y;
						dp.z = cos(ds) * xc(2) + sin(ds) * yc(2) + cp.z;
						dp.Roll  = sp.Roll+ds*(vp.Roll-sp.Roll)/(*mMF)[step-1][0].distance1;
						dp.Pitch = sp.Pitch+ds*(vp.Pitch-sp.Pitch)/(*mMF)[step-1][0].distance1;
						dp.Yaw   = sp.Yaw+ds*(vp.Yaw-sp.Yaw)/(*mMF)[step-1][0].distance1;
					}
					else
					{
						dp.x = cos(ds) * xc(0) + sin(ds) * yc(0) + cp.x;
						dp.y = cos(ds) * xc(1) + sin(ds) * yc(1) + cp.y;
						dp.z = cos(ds) * xc(2) + sin(ds) * yc(2) + cp.z;
						dp.Roll  = vp.Roll  +   (ep.Roll-vp.Roll) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
						dp.Pitch = vp.Pitch + (ep.Pitch-vp.Pitch) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
						dp.Yaw   = vp.Yaw   +   (ep.Yaw - vp.Yaw) * (ds - (*mMF)[step-1][0].distance1)/((*mMF)[step-1][0].distance - (*mMF)[step-1][0].distance1);
					}

					return dp;
				}
				else
				{
					return dp;
				}
			}
		}
	}

	vecd MotionPlay::CalcIK(Pose3D desired, int *ErrorStatus)
	{
		mRobot->ComputeIK(desired, &angleforplay, angleforplay, ErrorStatus);
		return angleforplay;
	}

	vecd MotionPlay::NextStep(int *ErrorStatus)
	{
		vecd qdf;
		vecd qdi = angleforplay;

		qdf = NextStepAtTime(CurrentTime, ErrorStatus);

		CurrentTime += TimePeriod*0.001;

		for(int i = 0; i< qdf.size() ; i++)
		{
			if((qdi - qdf)(i) > ML_PI_4 )
			{
				*ErrorStatus |= ARMSDK_TOO_MUCH_ANGLE_CHANGE;
				break;
			}
		}

		qdi = qdf;

		return qdi;
	}

	veci MotionPlay::NextStepforHand(void)
	{
		return NextStepAtTimeforHand(CurrentTime);
	}

	vecd MotionPlay::GetCurrentAngle(void)
	{
		return *mRobot->GetCurrentAngle();
	}

	Pose3D MotionPlay::GetCurrentEndPose(void)
	{
		return *mRobot->GetCurrentPose();
	}

	double MotionPlay::Get_CurrentTime(void)
	{
		return CurrentTime;
	}

	void MotionPlay::Set_CurrentTime(double CurrentTimeInSec)
	{
		CurrentTime = CurrentTimeInSec;
	}


	vecd MotionPlay::Get_ARM_1st_JointAngle(void)
	{
		return (*mTraj->GetMotionPoseList())[0].StartPose;
	}
}