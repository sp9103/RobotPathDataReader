#pragma once
#include <stdio.h>
#include <stdlib.h>

#include "dynamixel.h"

#define NUM_JOINT							6

#define PRO_OPERATING_MODE			11

#define PRO_TORQUE_ENABLE			562 //Address of Torque Enable in Control Table
#define PRO_LED_RED					563
#define PRO_LED_GREEN				564
#define PRO_LED_BLUE				565

//Not Used - Ignore this address
#define PRO_VELOCITY_I_GAIN			586
#define PRO_VELOCITY_P_GAIN			588
#define	PRO_POSITION_P_GAIN			594

#define PRO_GOAL_POSITION			596 //Address of Goal Position in Control Table
#define PRO_GOAL_VELOCITY			600
#define PRO_GOAL_TORQUE				604
#define PRO_GOAL_ACCELERATION		606

//Read ONLY
#define PRO_MOVING					610
#define PRO_PRESENT_POSITION		611 //Address of Goal Position in Control Table
#define PRO_PRESENT_VELOCITY		615
#define PRO_PRESENT_LOAD			619
#define PRO_PRESENT_CURRENT			621 //Address of Present Current in Control Table
#define PRO_PRESENT_INPUT_VOLTAGE	623
#define PRO_PRESENT_TEMPERATURE		625 //Address of Present Temperature in Control Table

//Mode
#define DYNAMIXELPRO_JOINTMODE		3
#define DYNAMIXELPRO_WHEELMODE		1
#define DYNAMIXELPRO_TORQUEMODE		0

class DynamixelPro
{
public:
	DynamixelPro(void);
	~DynamixelPro(void);
		
	void Init(SerialPort port, int *id);

	int TorqueOn();
	int TorqueOff();

	int GetTemperature(int *Temperature = NULL);						//온도 받아오기
	int GetPresPosition(int *PresentPosition = NULL);					//현재 위치 받아오기
	int GetGoalPosition(int *GoalPosition = NULL);						//목표 위치 받아오기
	int isMoving(bool *ismove = NULL);
	int GetPresVelocity(int *PresentVelocity = NULL);					//현재 속도 받아오기
	int GetGoalVelocity(int *GoalVelocity = NULL);						//목표 속도 받아오기
	int GetPresLoad(int *PresentLoad = NULL);
	int GetPresCurrent(int *PresentCurrent = NULL);

	int SetGoalVelocity(int *GoalVelocity);
	int SetGoalPosition(int *GoalPosition);
	int SetLED(unsigned char *LED);										//Red만

private:
	int ID_list_[NUM_JOINT];
	unsigned char mode_;

	BulkData bd[256];
	BulkData *pbd[256];

	SerialPort sp_;
	SerialPort *Port_;
};

