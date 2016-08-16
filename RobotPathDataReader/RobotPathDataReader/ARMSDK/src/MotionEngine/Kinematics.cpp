#include "MotionEngine\Kinematics.h"

namespace armsdk
{

	Kinematics::Kinematics(RobotInfo *_mRobot)
	{ 
		mRobotInfo = _mRobot->GetRobotInfo();
		DOF = mRobotInfo->size();

		currentangle.resize(mRobotInfo->size());

		for(unsigned int i = 0; i < mRobotInfo->size() ; i++)
		{
			currentangle(i) = (*mRobotInfo)[i].GetJointAngle();
		}
		mRobot = _mRobot;
	}


	Kinematics::~Kinematics() {   }

	void Kinematics::RobotInfoReload(void)
	{
		mRobotInfo = mRobot->GetRobotInfo();
		DOF = mRobotInfo->size();

		currentangle.resize(mRobotInfo->size());

		for(unsigned int i = 0; i < mRobotInfo->size() ; i++)
		{
			currentangle(i) = (*mRobotInfo)[i].GetJointAngle();
		}
	}

	matd Kinematics::Forward(vecd angle)
	{
		matd T;
		T = T.Identity(4,4);
		mRobotMat.clear();
		mRobotMat.push_back(T);
		for(unsigned int i=0; i < mRobotInfo->size() ; i++)
		{
			(*mRobotInfo)[i].SetJointAngle(angle(i));
			(*mRobotInfo)[i].SetJointAxis(T(0,2),T(1,2),T(2,2));
			(*mRobotInfo)[i].SetJointPosition(T(0,3),T(1,3),T(2,3));
			T = T * ((*mRobotInfo)[i].GetTransformMatirx());
			mRobotMat.push_back(T);
		}

		currentpose.x = T(0,3);
		currentpose.y = T(1,3);
		currentpose.z = T(2,3);
		currentpose.Roll  = atan2(T(2,1), T(2,2)); 
		currentpose.Pitch = atan2(-T(2,0), sqrt(T(2,1)*T(2,1) + T(2,2)*T(2,2)) );
		currentpose.Yaw   = atan2(T(1,0), T(0,0));

		//currentpose_Quaternion.x = currentpose.x;
		//currentpose_Quaternion.y = currentpose.y;
		//currentpose_Quaternion.z = currentpose.z;
		//currentpose_Quaternion.Quaternion = armsdk::Algebra::Euler2Quaternion(currentpose.Roll, currentpose.Pitch, currentpose.Yaw);

		currentangle = angle;

		return T;
	}

	matd Kinematics::Forward(vecd angle, Pose3D *pose)
	{
		matd T = matd::Identity(4,4);
		mRobotMat.clear();
		mRobotMat.push_back(T);
		for(unsigned int i=0; i < mRobotInfo->size() ; i++)
		{
			(*mRobotInfo)[i].SetJointAngle(angle[i]);
			(*mRobotInfo)[i].SetJointAxis(T(0,2),T(1,2),T(2,2));
			(*mRobotInfo)[i].SetJointPosition(T(0,3),T(1,3),T(2,3));
			T=T*((*mRobotInfo)[i].GetTransformMatirx());
			mRobotMat.push_back(T);
		}

		//ÇÑ´Ü ´õ Ãß°¡
		/*vector<JointData>* temp;
		RobotInfo tempJoint;
		tempJoint.AddJoint(  0.0,      0.0,    39.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 0);
		temp = tempJoint.GetRobotInfo();
		(*temp)[0].SetJointAngle(0.0);
		(*temp)[0].SetJointAxis(T(0,2),T(1,2),T(2,2));
		(*temp)[0].SetJointPosition(T(0,3),T(1,3),T(2,3));
		T=T*((*temp)[0].GetTransformMatirx());
		mRobotMat.push_back(T);

		for(int k = 0; k < mRobotMat.size(); k++){
			printf("[%d] x : %f, y : %f, z : %f\n", k, mRobotMat.at(k)(0,3), mRobotMat.at(k)(1,3), mRobotMat.at(k)(2,3));
		}*/

		currentpose.x = T(0,3);
		currentpose.y = T(1,3);
		currentpose.z = T(2,3);
		currentpose.Roll  = atan2(T(2,1), T(2,2)); 
		currentpose.Pitch = atan2(-T(2,0), sqrt(T(2,1)*T(2,1) + T(2,2)*T(2,2)) );
		currentpose.Yaw   = atan2(T(1,0), T(0,0));

		//currentpose_Quaternion.x = currentpose.x;
		//currentpose_Quaternion.y = currentpose.y;
		//currentpose_Quaternion.z = currentpose.z;
		//currentpose_Quaternion.Quaternion = armsdk::Algebra::Euler2Quaternion(currentpose.Roll, currentpose.Pitch, currentpose.Yaw);

		*pose = currentpose;
		currentangle = angle;
		return T;
	}

	void Kinematics::EndAxis(vecd angle, Pose3D *pose, Pose3D *xaxis, Pose3D *yaxis, Pose3D *zaxis){
		matd T = matd::Identity(4,4);
		mRobotMat.clear();
		mRobotMat.push_back(T);
		for(unsigned int i=0; i < mRobotInfo->size() ; i++)
		{
			(*mRobotInfo)[i].SetJointAngle(angle[i]);
			(*mRobotInfo)[i].SetJointAxis(T(0,2),T(1,2),T(2,2));
			(*mRobotInfo)[i].SetJointPosition(T(0,3),T(1,3),T(2,3));
			T=T*((*mRobotInfo)[i].GetTransformMatirx());
			mRobotMat.push_back(T);
		}

		//¹Ø ¼Õ¸ñ end effector
		Pose3D wrist;
		wrist.x = T(0,3);
		wrist.y = T(1,3);
		wrist.z = T(2,3);

		//¼Õ¸ñ end effector
		vector<JointData>* temp;
		RobotInfo tempJoint;
		matd tempT = matd::Identity(4,4);
		tempJoint.AddJoint(  0.0,     -ML_PI_2,    39.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 0);
		temp = tempJoint.GetRobotInfo();
		(*temp)[0].SetJointAngle(0.0);
		(*temp)[0].SetJointAxis(T(0,2),T(1,2),T(2,2));
		(*temp)[0].SetJointPosition(T(0,3),T(1,3),T(2,3));
		tempT=T*((*temp)[0].GetTransformMatirx());

		pose->x = tempT(0,3);
		pose->y = tempT(1,3);
		pose->z = tempT(2,3);
		pose->Roll  = atan2(tempT(2,1), tempT(2,2)); 
		pose->Pitch = atan2(-tempT(2,0), sqrt(tempT(2,1)*tempT(2,1) + tempT(2,2)*T(2,2)) );
		pose->Yaw   = atan2(tempT(1,0), tempT(0,0));

		//z-axis	(¼Õ¸ñ ÁÂÇ¥ - ¹Ø¼Õ¸ñ ÁÂÇ¥)
		zaxis->x = (pose->x - T(0,3)) / 39.f;
		zaxis->y = (pose->y - T(1,3)) / 39.f;
		zaxis->z = (pose->z - T(2,3)) / 39.f;

		//y-axis
		tempJoint.AddJoint(  0.0,      0.0,    39.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 0);
		temp = tempJoint.GetRobotInfo();
		(*temp)[0].SetJointAngle(0.0);
		(*temp)[0].SetJointAxis(tempT(0,2),tempT(1,2),tempT(2,2));
		(*temp)[0].SetJointPosition(tempT(0,3),tempT(1,3),tempT(2,3));
		tempT=tempT*((*temp)[0].GetTransformMatirx());

		printf("%f %f %f\n", tempT(0,3), tempT(1,3), tempT(2,3));

		yaxis->x = (tempT(0,3) - pose->x) / 39.f;
		yaxis->y = (tempT(1,3) - pose->y) / 39.f;
		yaxis->z = (tempT(2,3) - pose->z) / 39.f;

		//y-axis ( cross product : y*z )
		xaxis->x = yaxis->y*zaxis->z - yaxis->z*zaxis->y;
		xaxis->y = yaxis->z*zaxis->x - yaxis->x*zaxis->z;
		xaxis->z = yaxis->x*zaxis->y - yaxis->y*zaxis->x;
	}

	matd Kinematics::Jacobian(void)
	{
		matd J(6, DOF);
		vecd Jaxis(3);
		vecd Jpos(3);
		vecd jaco123(3);
		vecd currentpos(3);
		Position3D _Jpos;
		currentpos(0) = currentpose.x;
		currentpos(1) = currentpose.y;
		currentpos(2) = currentpose.z;

		for(unsigned int i = 0; i< DOF ;i++)
		{
			Jaxis = (*mRobotInfo)[i].GetJointAxis();
			_Jpos = (*mRobotInfo)[i].GetJointPosition();
			Jpos(0) = _Jpos.x;
			Jpos(1) = _Jpos.y;
			Jpos(2) = _Jpos.z;
			jaco123 = Algebra::Cross(Jaxis, currentpos-Jpos);
			J(0,i) = jaco123(0);
			J(1,i) = jaco123(1);
			J(2,i) = jaco123(2);
			J(3,i) = Jaxis(0);
			J(4,i) = Jaxis(1);
			J(5,i) = Jaxis(2);
		}
		return J;
	}

	vecd Kinematics::CalcError(Pose3D desired, matd current)
	{
		vecd de(6);
		de(0) = desired.x - current(0,3);
		de(1) = desired.y - current(1,3);
		de(2) = desired.z - current(2,3);

		vecd Oerror(3);
		Oerror = Algebra::rot2omega(((current.topLeftCorner(3,3)).transpose())*Algebra::GetOrientationMatrix(desired.Roll,desired.Pitch,desired.Yaw));
		Oerror = (current.topLeftCorner(3,3))*Oerror;
		de(3) = Oerror(0);
		de(4) = Oerror(1);
		de(5) = Oerror(2);

		return de;
	}

	void Kinematics::ComputeIK(Pose3D _desired, vecd *q, vecd Initangle, int *ErrorStatus)
	{
		unsigned int n = 0;
		vecd dq(DOF);
		double noei=0.0;
		double noef=0.0;
		vecd error;

		error = CalcError(_desired, Forward(Initangle));
		noei = error.norm();

		*ErrorStatus = 0x00;

		if(noei<0.000001)
		{
			*q = Initangle;
			*ErrorStatus |= ARMSDK_NO_ERROR;
		}
		else
		{
			matd I;
			I=I.Identity(6,6);
			matd J;
			matd Jt;
			matd invJ;
			const double Lambda0 = 0.5;
			const double w0=0.5;
			double Lambda;
			double w;
			int i = 0;

			while(1)
			{
				i += 1;
				J = Jacobian();
				Jt = J.transpose();

				w = sqrt(abs((J*Jt).determinant()));
				if( w < w0 )
					Lambda = Lambda0*(1-w/w0)*(1-w/w0);
				else
					Lambda = 0.2;


				invJ = J*Jt + Lambda*Lambda*I;
				invJ = invJ.inverse();
				invJ = Jt*invJ;
				dq = invJ * error;

				Initangle = Initangle + dq;

				error = CalcError(_desired, Forward(Initangle));
				noef=error.norm();

				if(abs(noei-noef) < 0.0005 || noef < 0.001 || i > 50)
				{
					for(unsigned int k=0; k < DOF ; ++k)
					{
						Initangle(k) = Initangle(k) - ML_2PI*floor(Initangle(k)/ML_2PI);

						if(Initangle(k)> ML_PI) 
							Initangle(k) -= ML_2PI;
						else if(Initangle(k)< -ML_PI )
							Initangle(k) +=ML_2PI;
					}

					if( i < 50)
					{
						if(noef < 0.001)
						{
							*ErrorStatus |= ARMSDK_NO_ERROR;
							for(int num = 0; num < DOF ; num++)
							{
								if(Initangle(num) > (*mRobotInfo)[num].GetMaxAngleLimitInRad() 
									|| Initangle(num) < (*mRobotInfo)[num].GetMinAngleLimitInRad())
								{	
									*ErrorStatus |= ARMSDK_OUT_OF_JOINT_RANGE;
									break;
								}
							}
						}
						else
						{
							*ErrorStatus |= ARMSDK_NO_IK_SOLUTION;
							if(noef < 5.0)
								*ErrorStatus |= ARMSDK_ACCEPTABLE_ERROR;

							for(int num = 0; num < DOF ; num++)
							{
								if(Initangle(num) > (*mRobotInfo)[num].GetMaxAngleLimitInRad() 
									|| Initangle(num) < (*mRobotInfo)[num].GetMinAngleLimitInRad())
								{	
									*ErrorStatus |= ARMSDK_OUT_OF_JOINT_RANGE;
									*ErrorStatus |= !ARMSDK_ACCEPTABLE_ERROR;
									break;
								}
							}
						}						
					}
					else
					{
						*ErrorStatus |= ARMSDK_NO_IK_SOLUTION;

						if(noef < 5.0)
							*ErrorStatus |= ARMSDK_ACCEPTABLE_ERROR;

						for(int num = 0; num < DOF ; num++)
						{
							if(Initangle(num) > (*mRobotInfo)[num].GetMaxAngleLimitInRad() 
								|| Initangle(num) < (*mRobotInfo)[num].GetMinAngleLimitInRad())
							{	
								*ErrorStatus |= ARMSDK_OUT_OF_JOINT_RANGE;
								*ErrorStatus |= !ARMSDK_ACCEPTABLE_ERROR;
								break;
							}
						}
					}

					break;
				}

				noei = noef;
			}
		}

		*q = Initangle;
		currentangle = Initangle;
		//Forward(currentangle);
	}


	unsigned int Kinematics::GetNumberofJoint(void)
	{
		return (*mRobotInfo).size();
	}

	vecd* Kinematics::GetCurrentAngle(void)
	{
		return &currentangle;
	}

	Pose3D* Kinematics::GetCurrentPose(void)
	{
		Forward(currentangle, &currentpose);
		return &currentpose;
	}

	RobotInfo* Kinematics::GetRobotInfo(void)
	{
		return mRobot;
	}

	veci Kinematics::Rad2Value(vecd q)
	{
		double m;
		double d;
		double r2;
		double r1;
		double v2, v1;

		veci value;
		value.resize(mRobotInfo->size());
		for(unsigned int i = 0 ; i<mRobotInfo->size() ; i++)
		{
			r2 = (*mRobotInfo)[i].GetMaxAngleInRad();
			r1 = (*mRobotInfo)[i].GetMinAngleInRad();

			v2 = (*mRobotInfo)[i].GetMaxAngleInValue();
			v1 = (*mRobotInfo)[i].GetMinAngleInValue();

			m = (v2 - v1)/(r2 - r1);
			d = v1 - m*r1;

			value[i] = (int)(m*q[i] + d);
		}
		return value;
	}

	veci Kinematics::Get_IDList(void)
	{
		veci ID;
		ID.resize(mRobotInfo->size());

		for(unsigned int i = 0; i < mRobotInfo->size() ; i++)
		{
			ID(i) = (*mRobotInfo)[i].GetID();
		}

		return ID;
	}

	vecd Kinematics::Value2Rad(veci q)
	{
		double m;
		double d;
		vecd rad;

		double r2;
		double r1;
		double v2, v1;

		rad.resize(mRobotInfo->size());
		for(unsigned int i = 0 ; i<mRobotInfo->size() ; i++)
		{
			r2 = (*mRobotInfo)[i].GetMaxAngleInRad();
			r1 = (*mRobotInfo)[i].GetMinAngleInRad();

			v2 = (*mRobotInfo)[i].GetMaxAngleInValue();
			v1 = (*mRobotInfo)[i].GetMinAngleInValue();

			m = (r2 - r1)/(v2 - v1);
			d = r1 - m*v1;

			rad[i] = (m*q[i] + d);
		}
		return rad;
	}

}