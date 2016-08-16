#ifndef __JOINTINFO_H_
#define __JOINTINFO_H_

//Joint Data based on D-H Notation


#include <cmath>
#include "../ARMSDK_Define.h"
#include "../ARMSDK_Math.h"

namespace armsdk
{
	class JointData
	{
	private:
		double Link_Length;
		double Link_Twist;
		double Joint_Offset;
		double Joint_Angle;

		double MinAngleRad;
		double MaxAngleRad;
		int MinAngleValue;
		int MaxAngleValue;

		double MinAngleLimitRad;
		double MaxAngleLimitRad;
		int MinAngleLimitValue;
		int MaxAngleLimitValue;
		
		unsigned int DXL_ID;
		vecd JointAxis;
		Position3D JointPosition;
		matd TransformMatrix;

	public:
		JointData();
		~JointData();

		void SetJointID(unsigned int ID);
		void SetJointAngle(double JointAngle);

		void SetMinAngleInRad(double MinAngleInRad);
		void SetMaxAngleInRad(double MaxAngleInRad);

		void SetMinAngleInValue(int Min_AngleValue);
		void SetMaxAngleInValue(int Max_AngleValue);

		void SetMinAngleLimitInRad(double MinAngleLimitInRad);
		void SetMaxAngleLimitInRad(double MaxAngleLimitInRad);

		//void SetMinAngleLimitInValue(int MinAngleLimitInValue);
		//void SetMaxAngleLimitInValue(int MaxAngleLimitInValue);
		
		void SetJointAxis(double J_Axis_x, double J_Axis_y, double J_Axis_z);
		void SetJointPosition(double J_Pos_x, double J_Pos_y, double J_Pos_z);


		unsigned int GetID(void);
		void SetJointDataDH(double LinkLenth, double LinkTwist, double JointOffset, double JointAngle);
		double GetJointAngle(void);

		double GetMinAngleInRad(void);
		double GetMaxAngleInRad(void);

		int GetMinAngleInValue(void);
		int GetMaxAngleInValue(void);

		double GetMinAngleLimitInRad(void);
		double GetMaxAngleLimitInRad(void);

		int GetMinAngleLimitInValue(void);
		int GetMaxAngleLimitInValue(void);


		vecd GetJointAxis(void);
		Position3D GetJointPosition(void);
		matd GetTransformMatirx(void);
	};
}
#endif
