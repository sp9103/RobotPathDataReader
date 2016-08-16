#include "MotionEngine\TrajectoryGenerator.h"

namespace armsdk
{
	TrajectoryGenerator::TrajectoryGenerator(Kinematics *mRobot)
	{
		this->mRobot = mRobot;
		angleforplanning = this->mRobot->GetCurrentAngle();
		angleforstart = *angleforplanning;
	}

	TrajectoryGenerator::~TrajectoryGenerator()
	{   }

	double TrajectoryGenerator::GetMotionTotalTime(void)
	{
		double t=0.0;
		for( unsigned int i = 0; i<mMF.size(); i++)
		{
			t += mMF[i][0].totoaltime;
		}
		return t;
	}

	void TrajectoryGenerator::KinematicsInfoReload(void)
	{
		this->mRobot->RobotInfoReload();
	}

	void TrajectoryGenerator::Set_P2P(vecd StartPose, vecd EndPose, double TotalTime, double AccelTime)
	{
		timeprofile mtime;
		MotionPose mPose;

		if(AccelTime >= TotalTime*0.5)
		{
			AccelTime = TotalTime*0.5;
		}

		mPose.StartPose = StartPose;
		mPose.EndPose = EndPose;

		Pose3D SP, EP;
		mRobot->Forward(StartPose, &SP);
		mRobot->Forward(EndPose, &EP);
		mPose.StartPose3D = SP;
		mPose.EndPose3D = EP;

		PoseList.push_back(mPose);

		*angleforplanning = EndPose;

		if(PoseList.size() == 1)
		{
			angleforstart = StartPose;
		}

		vecd DP = EndPose - StartPose;
		for(unsigned int i = 0; i < mRobot->GetNumberofJoint() ;i++)
		{
			mtime.ta = AccelTime;
			mtime.tc = TotalTime - 2 * AccelTime;
			mtime.td = AccelTime;
			mtime.totoaltime = TotalTime;
			mtime.Method = PtoP;

			if(abs(DP(i)) > 0.00001)
			{
				double V = DP(i)/(TotalTime-AccelTime);
				double a = V/AccelTime;

				mtime.distance = DP(i);
				mtime.a0[0] = StartPose(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.5*a;

				mtime.a0[1] = -0.5*a*AccelTime*AccelTime+StartPose(i);
				mtime.a1[1] = V;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = DP(i) - 0.5*a*TotalTime*TotalTime+StartPose(i);
				mtime.a1[2] = a*TotalTime;
				mtime.a2[2] = -0.5*a;
				mTF.push_back(mtime);
			}
			else
			{
				mtime.distance = 0.0;
				mtime.a0[0] = EndPose(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.0;

				mtime.a0[1] = EndPose(i);
				mtime.a1[1] = 0.0;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = EndPose(i);
				mtime.a1[2] = 0.0;
				mtime.a2[2] = 0.0;
				mTF.push_back(mtime);
			}
		}

		mMF.push_back(mTF);
		mTF.clear();
	}

	void TrajectoryGenerator::Set_P2P(Pose3D StartPose, Pose3D EndPose, double TotalTime, double AccelTime)
	{
		int ErrorStatus;
		mRobot->ComputeIK(StartPose, angleforplanning, *angleforplanning, &ErrorStatus);
		vecd SP = *angleforplanning;

		mRobot->ComputeIK(EndPose, angleforplanning, *angleforplanning,&ErrorStatus);
		vecd EP = *angleforplanning;
		MotionPose mPose;
		mPose.StartPose = SP;
		mPose.EndPose = EP;
		mPose.StartPose3D = StartPose;
		mPose.EndPose3D = EndPose;

		PoseList.push_back(mPose);

		Set_P2P(SP, EP, TotalTime, AccelTime);

		PoseList.pop_back();
	}

	void TrajectoryGenerator::Set_LIN(vecd StartPose, vecd EndPose, double TotalTime, double AccelTime )
	{
		if(AccelTime >= TotalTime*0.5)
		{
			AccelTime = TotalTime*0.5;
		}

		MotionPose mPose;
		mPose.StartPose = StartPose;
		mPose.EndPose = EndPose;

		Pose3D SP, EP;
		mRobot->Forward(StartPose, &SP);
		mRobot->Forward(EndPose, &EP);
		mPose.StartPose3D = SP;
		mPose.EndPose3D = EP;

		PoseList.push_back(mPose);

		*angleforplanning = EndPose;

		if(PoseList.size() == 1)
		{
			angleforstart = StartPose;
		}

		double L = sqrt((SP.x - EP.x)*(SP.x - EP.x) + (SP.y - EP.y)*(SP.y - EP.y) + (SP.z - EP.z)*(SP.z - EP.z));
		
		if ( L < 0.0000000001 )
		{
			Set_P2P(StartPose, EndPose, AccelTime, TotalTime);
		}
		else
		{
			double V = L/(TotalTime-AccelTime);
			double a = V/AccelTime;
	
			timeprofile mtime;
			mtime.Method = Linear;
			mtime.ta = AccelTime;
			mtime.tc = TotalTime - 2 * AccelTime;
			mtime.td = AccelTime;
			mtime.totoaltime = TotalTime;

			mtime.distance = L;

			mtime.a0[0] = 0.0;
			mtime.a1[0] = 0.0;
			mtime.a2[0] = 0.5*a;

			mtime.a0[1] = -0.5*a*AccelTime*AccelTime;
			mtime.a1[1] = V;
			mtime.a2[1] = 0.0;

			mtime.a0[2] = L - 0.5*a*TotalTime*TotalTime;
			mtime.a1[2] = a*TotalTime;
			mtime.a2[2] = -0.5*a;

			mTF.push_back(mtime);
			mMF.push_back(mTF);
			mTF.clear();
		}
	}

	void TrajectoryGenerator::Set_LIN(Pose3D StartPose, Pose3D EndPose, double TotalTime, double AccelTime)
	{
		int ErrorStatus;
		mRobot->ComputeIK(StartPose, angleforplanning, *angleforplanning, &ErrorStatus);
		vecd SP = *angleforplanning;

		mRobot->ComputeIK(EndPose, angleforplanning, *angleforplanning,&ErrorStatus);
		vecd EP = *angleforplanning;

		MotionPose mPose;
		mPose.StartPose = SP;
		mPose.StartPose3D = StartPose;
		mPose.EndPose = EP;
		mPose.EndPose3D = EndPose;

		PoseList.push_back(mPose);
		
		if(PoseList.size() == 1)
		{
			angleforstart = SP;
		}

		Set_LIN(SP, EP, TotalTime, AccelTime);
		PoseList.pop_back();
	}

	void TrajectoryGenerator::Set_CIRC(vecd StartPose, vecd ViaPose, vecd EndPose, double TotalTime, double AccelTime)
	{
		MotionPose mPose;

		if(AccelTime >= TotalTime*0.5)
		{
			AccelTime = TotalTime*0.5;
		}

		mPose.StartPose = StartPose;
		mPose.EndPose = EndPose;

		Pose3D SP, EP, VP;
		mRobot->Forward(StartPose, &SP);
		mRobot->Forward(EndPose, &EP);
		mRobot->Forward(ViaPose, &VP);
		mPose.StartPose3D = SP;
		mPose.EndPose3D = EP;
		mPose.ViaPose3D = VP;

		vecd vs(3);
		vs(0) = SP.x - VP.x;
		vs(1) = SP.y - VP.y;
		vs(2) = SP.z - VP.z;

		vecd ve(3);
		ve(0) = EP.x - VP.x;
		ve(1) = EP.y - VP.y;
		ve(2) = EP.z - VP.z;

		double p, q;
		p = ve.squaredNorm() * (vs.squaredNorm() - vs.dot(ve) ) / (2.0 * (vs.squaredNorm() * ve.squaredNorm() - vs.dot(ve) * vs.dot(ve)) );
		q = vs.squaredNorm() * (vs.dot(ve) - ve.squaredNorm() ) / (2.0 * (vs.dot(ve) * vs.dot(ve) - vs.squaredNorm() * ve.squaredNorm()) );

		vecd vc(3);
		vc = p*vs + q*ve;

		Position3D Center;
		Center.x = vc(0) + VP.x;
		Center.y = vc(1) + VP.y;
		Center.z = vc(2) + VP.z;

		mPose.CenterPosition = Center;

		PoseList.push_back(mPose);

		*angleforplanning = EndPose;

		timeprofile mtime;

		vecd cs, cv, ce;
		cs(0) = SP.x - Center.x;
		cs(1) = SP.y - Center.y;
		cs(2) = SP.z - Center.z;

		cv(0) = VP.x - Center.x;
		cv(1) = VP.y - Center.y;
		cv(2) = VP.z - Center.z;	

		ce(0) = EP.x - Center.x;
		ce(1) = EP.y - Center.y;
		ce(2) = EP.z - Center.z;

		mtime.distance1 = acos(cs.dot(cv)/ (cs.norm() * cv.norm()));
		mtime.distance = mtime.distance1 + acos(cv.dot(ce)/ (cv.norm() * ce.norm()));
		
		double V = mtime.distance/(TotalTime-AccelTime);
		double a = V/AccelTime;
		mtime.Method = Circular;
		mtime.ta = AccelTime;
		mtime.tc = TotalTime - 2 * AccelTime;
		mtime.td = AccelTime;
		mtime.totoaltime = TotalTime;

		mtime.a0[0] = 0.0;
		mtime.a1[0] = 0.0;
		mtime.a2[0] = 0.5*a;

		mtime.a0[1] = -0.5*a*AccelTime*AccelTime;
		mtime.a1[1] = V;
		mtime.a2[1] = 0.0;

		mtime.a0[2] = mtime.distance - 0.5*a*TotalTime*TotalTime;
		mtime.a1[2] = a*TotalTime;
		mtime.a2[2] = -0.5*a;

		mTF.push_back(mtime);
		mMF.push_back(mTF);
		mTF.clear();
	}

	void TrajectoryGenerator::Set_CIRC(Pose3D StartPose, Pose3D ViaPose, Pose3D EndPose, double TotalTime , double AccelTime )
	{
		MotionPose mPose;

		if(AccelTime >= TotalTime*0.5)
		{
			AccelTime = TotalTime*0.5;
		}

		int Er;
		mRobot->ComputeIK(StartPose, angleforplanning, *angleforplanning, &Er);
		mPose.StartPose = *angleforplanning;
		mRobot->ComputeIK(ViaPose, angleforplanning, *angleforplanning, &Er);
		mRobot->ComputeIK(EndPose, angleforplanning, *angleforplanning, &Er);
		mPose.EndPose = *angleforplanning;

		Pose3D SP, EP, VP;
		SP = StartPose;
		EP = EndPose;
		VP = ViaPose;

		mPose.StartPose3D = SP;
		mPose.EndPose3D = EP;
		mPose.ViaPose3D = VP;

		vecd vs(3);
		vs(0) = SP.x - VP.x;
		vs(1) = SP.y - VP.y;
		vs(2) = SP.z - VP.z;

		vecd ve(3);
		ve(0) = EP.x - VP.x;
		ve(1) = EP.y - VP.y;
		ve(2) = EP.z - VP.z;

		double p, q;
		p = ve.squaredNorm() * (vs.squaredNorm() - vs.dot(ve) ) / (2.0 * (vs.squaredNorm() * ve.squaredNorm() - vs.dot(ve) * vs.dot(ve) ));
		q = vs.squaredNorm() * (vs.dot(ve) - ve.squaredNorm() ) / (2.0 * (vs.dot(ve) * vs.dot(ve) - vs.squaredNorm() * ve.squaredNorm() ));

		vecd vc(3);
		vc = p*vs + q*ve;

		Position3D Center;
		Center.x = vc(0) + VP.x;
		Center.y = vc(1) + VP.y;
		Center.z = vc(2) + VP.z;

		mPose.CenterPosition = Center;

		PoseList.push_back(mPose);

		timeprofile mtime;

		vecd cs(3), cv(3), ce(3);
		cs(0) = SP.x - Center.x;
		cs(1) = SP.y - Center.y;
		cs(2) = SP.z - Center.z;

		cv(0) = VP.x - Center.x;
		cv(1) = VP.y - Center.y;
		cv(2) = VP.z - Center.z;	

		ce(0) = EP.x - Center.x;
		ce(1) = EP.y - Center.y;
		ce(2) = EP.z - Center.z;
		mtime.distance1 = acos(cs.dot(cv) / ( cs.norm() * cv.norm() ) );
		mtime.distance = mtime.distance1 + acos(cv.dot(ce) / ( cv.norm() * ce.norm() ) );
		
		double V = mtime.distance/(TotalTime-AccelTime);
		double a = V/AccelTime;
		mtime.Method = Circular;
		mtime.ta = AccelTime;
		mtime.tc = TotalTime - 2 * AccelTime;
		mtime.td = AccelTime;
		mtime.totoaltime = TotalTime;

		mtime.a0[0] = 0.0;
		mtime.a1[0] = 0.0;
		mtime.a2[0] = 0.5*a;

		mtime.a0[1] = -0.5*a*AccelTime*AccelTime;
		mtime.a1[1] = V;
		mtime.a2[1] = 0.0;

		mtime.a0[2] = mtime.distance - 0.5*a*TotalTime*TotalTime;
		mtime.a1[2] = a*TotalTime;
		mtime.a2[2] = -0.5*a;

		mTF.push_back(mtime);
		mMF.push_back(mTF);
		mTF.clear();
	}

	void TrajectoryGenerator::Set_P2PwithHand(vecd StartPose, vecd EndPose, veci Hand1, veci Hand2, double TotalTime, double AccelTime)
	{
		Set_P2P(StartPose, EndPose, TotalTime, AccelTime);

		veci DPh = Hand2 - Hand1;
		timeprofile mtime;
		for(unsigned int i = 0; i < 3 ;i++)
		{
			mtime.ta = AccelTime;
			mtime.tc = TotalTime - 2 * AccelTime;
			mtime.td = AccelTime;
			mtime.totoaltime = TotalTime;

			if(abs(DPh(i)) > 1)
			{
				double V = (double)DPh(i)/(TotalTime-AccelTime);
				double a = V/AccelTime;

				mtime.distance = DPh(i);
				mtime.a0[0] = Hand1(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.5*a;

				mtime.a0[1] = -0.5*a*AccelTime*AccelTime+Hand1(i);
				mtime.a1[1] = V;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = DPh(i) - 0.5*a*TotalTime*TotalTime+Hand1(i);
				mtime.a1[2] = a*TotalTime;
				mtime.a2[2] = -0.5*a;
				mTF.push_back(mtime);
			}
			else
			{
				mtime.distance = 0.0;
				mtime.a0[0] = Hand2(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.0;

				mtime.a0[1] = Hand2(i);
				mtime.a1[1] = 0.0;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = Hand2(i);
				mtime.a1[2] = 0.0;
				mtime.a2[2] = 0.0;
				mTF.push_back(mtime);
			}
		}
		mHandMF.push_back(mTF);
		mTF.clear();
	}

	void TrajectoryGenerator::Set_P2PwithHand(Pose3D StartPose, Pose3D EndPose, veci Hand1, veci Hand2, double TotalTime, double AccelTime)
	{
		Set_P2P(StartPose, EndPose, TotalTime, AccelTime);

		veci DPh = Hand2 - Hand1;
		timeprofile mtime;
		for(unsigned int i = 0; i < 3 ;i++)
		{
			mtime.ta = AccelTime;
			mtime.tc = TotalTime - 2 * AccelTime;
			mtime.td = AccelTime;
			mtime.totoaltime = TotalTime;

			if(abs(DPh(i)) > 1)
			{
				double V = (double)DPh(i)/(TotalTime-AccelTime);
				double a = V/AccelTime;

				mtime.distance = DPh(i);
				mtime.a0[0] = Hand1(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.5*a;

				mtime.a0[1] = -0.5*a*AccelTime*AccelTime+Hand1(i);
				mtime.a1[1] = V;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = DPh(i) - 0.5*a*TotalTime*TotalTime+Hand1(i);
				mtime.a1[2] = a*TotalTime;
				mtime.a2[2] = -0.5*a;
				mTF.push_back(mtime);
			}
			else
			{
				mtime.distance = 0.0;
				mtime.a0[0] = Hand2(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.0;

				mtime.a0[1] = Hand2(i);
				mtime.a1[1] = 0.0;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = Hand2(i);
				mtime.a1[2] = 0.0;
				mtime.a2[2] = 0.0;
				mTF.push_back(mtime);
			}
		}
		mHandMF.push_back(mTF);
		mTF.clear();

	}
					
	void TrajectoryGenerator::Set_LINwithHand(vecd StartPose, vecd EndPose, veci Hand1, veci Hand2, double TotalTime, double AccelTime)
	{
		Set_LIN(StartPose, EndPose, TotalTime, AccelTime);

		veci DPh = Hand2 - Hand1;
		timeprofile mtime;
		for(unsigned int i = 0; i < 3 ;i++)
		{
			mtime.ta = AccelTime;
			mtime.tc = TotalTime - 2 * AccelTime;
			mtime.td = AccelTime;
			mtime.totoaltime = TotalTime;

			if(abs(DPh(i)) > 1)
			{
				double V = (double)DPh(i)/(TotalTime-AccelTime);
				double a = V/AccelTime;

				mtime.distance = DPh(i);
				mtime.a0[0] = Hand1(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.5*a;

				mtime.a0[1] = -0.5*a*AccelTime*AccelTime+Hand1(i);
				mtime.a1[1] = V;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = DPh(i) - 0.5*a*TotalTime*TotalTime+Hand1(i);
				mtime.a1[2] = a*TotalTime;
				mtime.a2[2] = -0.5*a;
				mTF.push_back(mtime);
			}
			else
			{
				mtime.distance = 0.0;
				mtime.a0[0] = Hand2(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.0;

				mtime.a0[1] = Hand2(i);
				mtime.a1[1] = 0.0;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = Hand2(i);
				mtime.a1[2] = 0.0;
				mtime.a2[2] = 0.0;
				mTF.push_back(mtime);
			}
		}
		mHandMF.push_back(mTF);
		mTF.clear();

	}

	void TrajectoryGenerator::Set_LINwithHand(Pose3D StartPose, Pose3D EndPose, veci Hand1, veci Hand2, double TotalTime, double AccelTime)
	{
		Set_LIN(StartPose, EndPose, TotalTime, AccelTime);

		veci DPh = Hand2 - Hand1;
		timeprofile mtime;
		for(unsigned int i = 0; i < 3 ;i++)
		{
			mtime.ta = AccelTime;
			mtime.tc = TotalTime - 2 * AccelTime;
			mtime.td = AccelTime;
			mtime.totoaltime = TotalTime;

			if(abs(DPh(i)) > 1)
			{
				double V = (double)DPh(i)/(TotalTime-AccelTime);
				double a = V/AccelTime;

				mtime.distance = DPh(i);
				mtime.a0[0] = Hand1(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.5*a;

				mtime.a0[1] = -0.5*a*AccelTime*AccelTime+Hand1(i);
				mtime.a1[1] = V;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = DPh(i) - 0.5*a*TotalTime*TotalTime+Hand1(i);
				mtime.a1[2] = a*TotalTime;
				mtime.a2[2] = -0.5*a;
				mTF.push_back(mtime);
			}
			else
			{
				mtime.distance = 0.0;
				mtime.a0[0] = Hand2(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.0;

				mtime.a0[1] = Hand2(i);
				mtime.a1[1] = 0.0;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = Hand2(i);
				mtime.a1[2] = 0.0;
				mtime.a2[2] = 0.0;
				mTF.push_back(mtime);
			}
		}
		mHandMF.push_back(mTF);
		mTF.clear();

	}

	void TrajectoryGenerator::Set_CIRCwithHand(vecd StartPose, vecd ViaPose, vecd EndPose, veci Hand1, veci Hand2, double TotalTime, double AccelTime)
	{
		Set_CIRC(StartPose, ViaPose, EndPose, TotalTime, AccelTime);

		veci DPh = Hand2 - Hand1;
		timeprofile mtime;
		for(unsigned int i = 0; i < 3 ;i++)
		{
			mtime.ta = AccelTime;
			mtime.tc = TotalTime - 2 * AccelTime;
			mtime.td = AccelTime;
			mtime.totoaltime = TotalTime;

			if(abs(DPh(i)) > 1)
			{
				double V = (double)DPh(i)/(TotalTime-AccelTime);
				double a = V/AccelTime;

				mtime.distance = DPh(i);
				mtime.a0[0] = Hand1(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.5*a;

				mtime.a0[1] = -0.5*a*AccelTime*AccelTime+Hand1(i);
				mtime.a1[1] = V;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = DPh(i) - 0.5*a*TotalTime*TotalTime+Hand1(i);
				mtime.a1[2] = a*TotalTime;
				mtime.a2[2] = -0.5*a;
				mTF.push_back(mtime);
			}
			else
			{
				mtime.distance = 0.0;
				mtime.a0[0] = Hand2(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.0;

				mtime.a0[1] = Hand2(i);
				mtime.a1[1] = 0.0;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = Hand2(i);
				mtime.a1[2] = 0.0;
				mtime.a2[2] = 0.0;
				mTF.push_back(mtime);
			}
		}
		mHandMF.push_back(mTF);
		mTF.clear();
	}

	void TrajectoryGenerator::Set_CIRCwithHand(Pose3D StartPose, Pose3D ViaPose, Pose3D EndPose, veci Hand1, veci Hand2, double TotalTime, double AccelTime)
	{
		Set_CIRC(StartPose, ViaPose, EndPose, TotalTime, AccelTime);

		veci DPh = Hand2 - Hand1;
		timeprofile mtime;
		for(unsigned int i = 0; i < 3 ;i++)
		{
			mtime.ta = AccelTime;
			mtime.tc = TotalTime - 2 * AccelTime;
			mtime.td = AccelTime;
			mtime.totoaltime = TotalTime;

			if(abs(DPh(i)) > 1)
			{
				double V = (double)DPh(i)/(TotalTime-AccelTime);
				double a = V/AccelTime;

				mtime.distance = DPh(i);
				mtime.a0[0] = Hand1(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.5*a;

				mtime.a0[1] = -0.5*a*AccelTime*AccelTime+Hand1(i);
				mtime.a1[1] = V;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = DPh(i) - 0.5*a*TotalTime*TotalTime+Hand1(i);
				mtime.a1[2] = a*TotalTime;
				mtime.a2[2] = -0.5*a;
				mTF.push_back(mtime);
			}
			else
			{
				mtime.distance = 0.0;
				mtime.a0[0] = Hand2(i);
				mtime.a1[0] = 0.0;
				mtime.a2[0] = 0.0;

				mtime.a0[1] = Hand2(i);
				mtime.a1[1] = 0.0;
				mtime.a2[1] = 0.0;

				mtime.a0[2] = Hand2(i);
				mtime.a1[2] = 0.0;
				mtime.a2[2] = 0.0;
				mTF.push_back(mtime);
			}
		}
		mHandMF.push_back(mTF);
		mTF.clear();

	}

	void TrajectoryGenerator::ClearMF(void)
	{
		mMF.clear();
		mTF.clear();
		PoseList.clear();
		mHandMF.clear();
		angleforplanning = mRobot->GetCurrentAngle();
		angleforstart = *angleforplanning;
	}

	vecd TrajectoryGenerator::GetAngleforStart(void)
	{
		return angleforstart;
	}

	MotionProfile* TrajectoryGenerator::GetMotionProfile(void)
	{
		return &mMF;
	}

	MotionProfile* TrajectoryGenerator::GetHandMotionProfile(void)
	{
		return &mHandMF;
	}
	
	MotionPoseList* TrajectoryGenerator::GetMotionPoseList(void)
	{
		return &PoseList;
	}

	Kinematics* TrajectoryGenerator::GetKinematics(void)
	{
		return mRobot;
	}
}