#pragma once
#include <stdio.h>
#include <Eigen\Dense>
#include <Windows.h>

#include "dynamixel.h"
#include "MX_controller.h"
#include "DynamixelPro.h"

#define MAX_MOVING_DEGREE					10
#define USING_FINGER

class RobotArm
{
public:
	RobotArm(void);
	~RobotArm(void);

	int Init(int PortNum, int BaudRateNum, int *ID_list);
	int DeInit();
	int SetID(int *ID_list);

	int TorqueOn();
	int TorqueOff();

	int GetTemperature(int *Temperature = NULL);						//온도 받아오기
	int GetPresPosition(int *PresentPosition = NULL);					//현재 위치 받아오기
	int GetGoalPosition(int *GoalPosition = NULL);						//목표 위치 받아오기
	int isMoving(bool *ismove = NULL);								//움직임 유무 받아오기
	int GetPresVelocity(int *PresentVelocity = NULL);					//현재 속도 받아오기
	int GetGoalVelocity(int *GoalVelocity = NULL);						//목표 속도 받아오기
	int GetFingerLoad(int *load = NULL);

	int SetGoalVelocity(int *GoalVelocity);
	int SetGoalPosition(int *GoalPosition);
	int SetLED(bool onoff);
	int SetFingerPosition(int *GoalPosition);

	void safeReleasePose();
	void safeMovePose(int *goal);

	//ARMSDK function
	int Arm_Get_JointValue(Eigen::VectorXi *value);
	SerialPort* DXL_Get_Port(void);

private:
	int fingerID_[NUM_FINGER];
	int jointID_[NUM_JOINT];
	int Com_port_num_;
	int Baud_rate_num_;

	SerialPort sp_;
	SerialPort *Port_;

	DynamixelPro jointcontroller_;
#ifdef USING_FINGER
	MX_controller fingercontroller_;
#endif

	bool waitMove();
};

