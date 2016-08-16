#ifndef __ARM_COMM_WIN32_H_
#define __ARM_COMM_WIN32_H_

#include <afx.h>
#include <iostream>
#include "dynamixel.h"
#include "../ARMSDK_Math.h"
#include "NX.h"
#include <conio.h>

#pragma comment (lib, "dynamixel.lib")

namespace armsdk
{
	class ARM_Comm_Win32_NX
	{
		veci ARM_IDLIST;
		int mPortnum;
		int mBaudnum;
		BulkData bd[256];
		BulkData *pbd[256];
		SerialPort sp, *Port;

	public:
		ARM_Comm_Win32_NX();
		ARM_Comm_Win32_NX(int portnum, int baudnum);
		~ARM_Comm_Win32_NX(){  }

		void DXL_Set_Init_Param(int portnum, int baudnum);
		int DXL_Open();	
		SerialPort* DXL_Get_Port(void);
		void DXL_Close(void);
		

		//Robot Arm
		void Arm_ID_Setup(veci ID_LIST);
		int Arm_Set_JointValue(veci value);
		int Arm_Torque_On(void);
		int Arm_Torque_Off(void);
		int Arm_Get_JointValue(veci *value);
		int Arm_Set_Position_PID_Gain(int P, int I, int D);
		int Arm_Set_Position_PID_Gain(int P, int I, int D, int id);
		int Arm_LED_On(void);
		int Arm_LED_Off(void);

		int Arm_Write_Parameter(int Address, int nByte);
		int Arm_Read_Parameter(void);
	};
}
#endif