#include "MotionEngine\JointData.h"

namespace armsdk
{
	JointData::JointData()
	{
		Link_Length    = 0.0;
		Link_Twist     = 0.0;
		Joint_Offset   = 0.0;
		Joint_Angle    = 0.0;

		MinAngleRad      = -ML_PI;
		MaxAngleRad      =  ML_PI;
		MinAngleValue  =  0;
		MaxAngleValue  =  4095;
		DXL_ID         =  1;

		MinAngleLimitRad = -ML_PI;
		MaxAngleLimitRad = ML_PI;
		MinAngleLimitValue = 0;
		MaxAngleLimitValue = 4095;


		JointAxis.resize(3);
		JointAxis.fill(0.0);
		JointPosition.x = 0.0;
		JointPosition.y = 0.0;
		JointPosition.z = 0.0;
		TransformMatrix.fill(0.0);
	}

	JointData::~JointData() {   }

	void JointData::SetJointID(unsigned int ID)
	{
		DXL_ID = ID;
	}

	void JointData::SetJointAngle(double JointAngle)
	{
		Joint_Angle = JointAngle;

		double ca = cos(Joint_Angle);
		double sa = sin(Joint_Angle);
		double ct = cos(Link_Twist);
		double st = sin(Link_Twist);

		TransformMatrix.fill(0.0);
		TransformMatrix.resize(4,4);
		TransformMatrix <<  ca, -sa*ct,  sa*st, Link_Length*ca,
			sa,  ca*ct, -ca*st, Link_Length*sa,
			0.0,     st,     ct,   Joint_Offset,
			0.0,    0.0,    0.0,            1.0;
	}

	void JointData::SetMinAngleInRad(double MinAngleInRad)
	{
		MinAngleRad = MinAngleInRad;
	}

	void JointData::SetMaxAngleInRad(double MaxAngleInRad)
	{
		MaxAngleRad = MaxAngleInRad;
	}

	void JointData::SetMinAngleInValue(int Min_AngleValue)
	{
		MinAngleValue = Min_AngleValue;
	}

	void JointData::SetMaxAngleInValue(int Max_AngleValue)
	{
		MaxAngleValue = Max_AngleValue;
	}

	void JointData::SetMinAngleLimitInRad(double MinAngleLimitInRad)
	{
		MinAngleLimitRad = MinAngleLimitInRad;
		//Limit Value 계산식 추가
		double m, d;
		double r2,r1,v2,v1;
		
		r2 = MaxAngleRad;
		r1 = MinAngleRad;

		v2 = MaxAngleValue;
		v1 = MinAngleValue;

		m = (v2 - v1)/(r2 - r1);
		d = v1 - m*r1;
		MinAngleLimitValue = (int)(m*MinAngleLimitInRad + d);
	}

	void JointData::SetMaxAngleLimitInRad(double MaxAngleLimitInRad)
	{
		MaxAngleLimitRad = MaxAngleLimitInRad;

		double m, d;
		double r2,r1,v2,v1;
		
		r2 = MaxAngleRad;
		r1 = MinAngleRad;

		v2 = MaxAngleValue;
		v1 = MinAngleValue;

		m = (v2 - v1)/(r2 - r1);
		d = v1 - m*r1;
		MaxAngleLimitValue = (int)(m*MaxAngleLimitInRad + d);
	}

	//void JointData::SetMinAngleLimitInValue(int MinAngleLimitInValue)
	//{
	//	MinAngleLimitValue = MinAngleLimitInValue; 
	//}

	//void JointData::SetMaxAngleLimitInValue(int MaxAngleLimitInValue)
	//{
	//	MaxAngleLimitValue = MaxAngleLimitInValue;
	//}

	void JointData::SetJointAxis(double J_Axis_x, double J_Axis_y, double J_Axis_z)
	{
		JointAxis(0) = J_Axis_x;
		JointAxis(1) = J_Axis_y;
		JointAxis(2) = J_Axis_z;
	}

	void JointData::SetJointPosition(double JPosx, double JPosy, double JPosz)
	{
		JointPosition.x = JPosx;
		JointPosition.y = JPosy;
		JointPosition.z = JPosz;
	}

	void JointData::SetJointDataDH(double LinkLenth, double LinkTwist, double JointOffset, double JointAngle)
	{
		Link_Length = LinkLenth;
		Link_Twist = LinkTwist;
		Joint_Offset = JointOffset;
		SetJointAngle(JointAngle);

	}

	double JointData::GetJointAngle(void)
	{
		return Joint_Angle;
	}

	double JointData::GetMinAngleInRad(void)
	{
		return MinAngleRad;
	}

	double JointData::GetMaxAngleInRad(void)
	{
		return MaxAngleRad;
	}

	int JointData::GetMinAngleInValue(void)
	{
		return MinAngleValue;
	}

	int JointData::GetMaxAngleInValue(void)
	{
		return MaxAngleValue;
	}

	double JointData::GetMinAngleLimitInRad(void)
	{
		return MinAngleLimitRad;
	}

	double JointData::GetMaxAngleLimitInRad(void)
	{
		return MaxAngleLimitRad;
	}

	int JointData::GetMinAngleLimitInValue(void)
	{
		return MinAngleLimitValue;
	}

	int JointData::GetMaxAngleLimitInValue(void)
	{
		return MaxAngleLimitValue;
	}


	unsigned int JointData::GetID(void)
	{
		return DXL_ID;
	}

	vecd JointData::GetJointAxis(void)
	{
		return JointAxis;
	}

	Position3D JointData::GetJointPosition(void)
	{
		return JointPosition;
	}

	matd JointData::GetTransformMatirx(void)
	{
		return TransformMatrix;
	}
}