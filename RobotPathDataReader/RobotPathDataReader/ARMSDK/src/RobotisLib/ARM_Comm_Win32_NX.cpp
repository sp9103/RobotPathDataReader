#include "RobotisLib\ARM_Comm_Win32_NX.h"

namespace armsdk
{
	ARM_Comm_Win32_NX::ARM_Comm_Win32_NX()
	{
		sp.dPacketStartTime = 0;
		sp.fByteTransferTime = 0;
		sp.fPacketWaitTime = 0;
		sp.hComm = 0;
		sp.iBusUsing = 0;

		Port = &sp;

		for(unsigned int i = 0 ; i <256 ; i++)
		{
			pbd[i] = &bd[i];
		}
	}

	ARM_Comm_Win32_NX::ARM_Comm_Win32_NX(int portnum, int baudnum)
	{
		sp.dPacketStartTime = 0;
		sp.fByteTransferTime = 0;
		sp.fPacketWaitTime = 0;
		sp.hComm = 0;
		sp.iBusUsing = 0;

		Port = &sp;

		for(unsigned int i = 0 ; i <256 ; i++)
			pbd[i] = &bd[i];

		mPortnum = portnum; 	mBaudnum = baudnum;
	}

	void ARM_Comm_Win32_NX::DXL_Set_Init_Param(int portnum, int baudnum)
	{
		mPortnum = portnum;
		mBaudnum = baudnum;
	}

	SerialPort* ARM_Comm_Win32_NX::DXL_Get_Port(void)
	{
		return Port;
	}

	int ARM_Comm_Win32_NX::DXL_Open()
	{
		if( dxl_initialize(Port, mPortnum, mBaudnum) == 0 )
		{
			printf( "Failed to open USB2Dynamixel!\n" );
			return 0;
		}
		else
		{	
			printf("Succeed to open USB2Dynamixel!\n");
			return 1;
		}
	}

	void ARM_Comm_Win32_NX::DXL_Close(void)
	{
		dxl_terminate(Port);
		std::cout<<"DXL is terminated"<<std::endl;
	}


	void ARM_Comm_Win32_NX::Arm_ID_Setup(veci Arm_ID_List)
	{
		ARM_IDLIST.resize(Arm_ID_List.size());
		ARM_IDLIST = Arm_ID_List;
	}

	int ARM_Comm_Win32_NX::Arm_Set_JointValue(veci value)
	{
		if(value.size() != ARM_IDLIST.size())
			return 0;

		unsigned char* param = new unsigned char[5*value.size()];
		for(int i = 0; i < value.size(); i++)
		{
			param[i*5] = (unsigned char)ARM_IDLIST[i];
			param[i*5+1] = DXL_LOBYTE(DXL_LOWORD(value[i]));
			param[i*5+2] = DXL_HIBYTE(DXL_LOWORD(value[i]));
			param[i*5+3] = DXL_LOBYTE(DXL_HIWORD(value[i]));
			param[i*5+4] = DXL_HIBYTE(DXL_HIWORD(value[i]));
		}
		int b = dxl_sync_write(Port, NX::P_GOAL_POSITION_LL, 4, param, 5*value.size());
		delete []param;
		return b;
	}

	int ARM_Comm_Win32_NX::Arm_Torque_On(void)
	{
		unsigned char* param = new unsigned char[2*ARM_IDLIST.size()];
		for(int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i*2] = ARM_IDLIST[i];
			param[i*2+1] = 1;
		}
		int b = dxl_sync_write(Port, NX::P_TORQUE_ENABLE, 1, param, 2*ARM_IDLIST.size());
		delete []param;
		return b;
	}

	int ARM_Comm_Win32_NX::Arm_Torque_Off(void)
	{
		unsigned char* param = new unsigned char[2*ARM_IDLIST.size()];
		for(int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i*2] = ARM_IDLIST[i];
			param[i*2+1] = 0;
		}
		int b = dxl_sync_write(Port, NX::P_TORQUE_ENABLE, 1, param, 2*ARM_IDLIST.size());
		delete []param;
		return b;
	}

	int ARM_Comm_Win32_NX::Arm_Get_JointValue(veci *value)
	{
		value->resize(ARM_IDLIST.size());

		int i = 0;
		int result = 0;
		int temp_val;

		for(i = 0; i < ARM_IDLIST.size() ; i++)
		{
			result = dxl_read_dword(Port, ARM_IDLIST[i], NX::P_PRESENT_POSITION_LL, (unsigned int*)&temp_val, 0);

			if(result != COMM_RXSUCCESS)
				return result;

			(*value)[i] = temp_val;
		}

		return result;
	}

	int ARM_Comm_Win32_NX::Arm_Set_Position_PID_Gain(int P, int I, int D)
	{
		unsigned char* param = new unsigned char[7*ARM_IDLIST.size()];
		for(int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i*7] = (unsigned char)ARM_IDLIST[i];
			param[i*7+1] = DXL_LOBYTE(D);
			param[i*7+2] = DXL_HIBYTE(D);
			param[i*7+3] = DXL_LOBYTE(I);
			param[i*7+4] = DXL_HIBYTE(I);
			param[i*7+5] = DXL_LOBYTE(P);
			param[i*7+6] = DXL_HIBYTE(P);
		}
		int b = dxl_sync_write(Port, NX::P_POSITION_D_GAIN_L, 6, param, 7*ARM_IDLIST.size());
		delete []param;
		return b;
	}

	int ARM_Comm_Win32_NX::Arm_Set_Position_PID_Gain(int P, int I, int D, int id)
	{
		unsigned char* param = new unsigned char[7];

		param[0] = id;
		param[1] = DXL_LOBYTE(D);
		param[2] = DXL_HIBYTE(D);
		param[3] = DXL_LOBYTE(I);
		param[4] = DXL_HIBYTE(I);
		param[5] = DXL_LOBYTE(P);
		param[6] = DXL_HIBYTE(P);

		int b = dxl_sync_write(Port, NX::P_POSITION_D_GAIN_L, 6, param, 7);
		delete []param;
		return b;
	}

	int ARM_Comm_Win32_NX::Arm_LED_On(void)
	{
		unsigned char* param = new unsigned char[2*ARM_IDLIST.size()];
		for(int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i*2] = ARM_IDLIST[i];
			param[i*2+1] = 255;
		}
		int b = dxl_sync_write(Port, NX::P_LED_RED, 1, param, 2*ARM_IDLIST.size());
		delete []param;
		return b;
	}

	int ARM_Comm_Win32_NX::Arm_LED_Off(void)
	{
		unsigned char* param = new unsigned char[2*ARM_IDLIST.size()];
		for(int i = 0; i < ARM_IDLIST.size(); i++)
		{
			param[i*2] = ARM_IDLIST[i];
			param[i*2+1] = 0;
		}
		int b = dxl_sync_write(Port, NX::P_LED_RED, 1, param, 2*ARM_IDLIST.size());
		delete []param;
		return b;
	}
}