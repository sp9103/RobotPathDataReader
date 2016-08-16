#include "DynamixelPro.h"


DynamixelPro::DynamixelPro(void)
{
	sp_.dPacketStartTime = 0;
	sp_.fByteTransferTime = 0;
	sp_.fPacketWaitTime = 0;
	sp_.hComm = 0;
	sp_.iBusUsing = 0;

	mode_ = -1;

	Port_ = &sp_;

	for(unsigned int i = 0 ; i <256 ; i++)
	{
		pbd[i] = &bd[i];
	}
}


DynamixelPro::~DynamixelPro(void)
{
}

void DynamixelPro::Init(SerialPort port, int *id){
	for(int i = 0; i < NUM_JOINT; i++)
		ID_list_[i] = id[i];

	sp_ = port;
}

int DynamixelPro::TorqueOn(){
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(char)*(NUM_JOINT*2));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*2] = (unsigned char)ID_list_[i];
		t_param[i*2+1] = 1;
	}

	int Result = dxl_sync_write(Port_, PRO_TORQUE_ENABLE, 1, t_param, NUM_JOINT*2);
	if( Result != COMM_RXSUCCESS )
	{
		printf( "Failed DynamixelPro Enable!\n" );
		free(t_param);
		return -1;
	}

	free(t_param);

	return 1;
}

int DynamixelPro::TorqueOff(){
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(char)*(NUM_JOINT*2));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*2] = (unsigned char)ID_list_[i];
		t_param[i*2+1] = 0;
	}

	int Result = dxl_sync_write(Port_, PRO_TORQUE_ENABLE, 1, t_param, NUM_JOINT*2);
	if( Result != COMM_RXSUCCESS )
	{
		printf( "Failed DynamixelPro Disable!\n" );
		free(t_param);
		return -1;
	}

	free(t_param);

	return 1;
}

int DynamixelPro::GetTemperature(int *Temperature){
	int tP_temperature[NUM_JOINT], temperature_length = 1;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_JOINT*5));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(PRO_PRESENT_TEMPERATURE);
		t_param[i*5+2] = DXL_HIBYTE(PRO_PRESENT_TEMPERATURE);
		t_param[i*5+3] = DXL_LOBYTE(temperature_length);
		t_param[i*5+4] = DXL_HIBYTE(temperature_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_JOINT*5, pbd);

	for(int i = 0; i < NUM_JOINT; i++){
		dxl_get_bulk_byte(pbd, ID_list_[i], PRO_PRESENT_TEMPERATURE, &tP_temperature[i]);
		printf("ID %d : %d\n", ID_list_[i], tP_temperature[i]);

		if(Temperature != NULL){
			Temperature[i] = tP_temperature[i];
		}
	}

	free(t_param);

	return 1;
}

int DynamixelPro::isMoving(bool *ismove){
	int tP_position[NUM_JOINT],tIsMovingLength = 1;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_JOINT*5));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(PRO_MOVING);
		t_param[i*5+2] = DXL_HIBYTE(PRO_MOVING);
		t_param[i*5+3] = DXL_LOBYTE(tIsMovingLength);
		t_param[i*5+4] = DXL_HIBYTE(tIsMovingLength);
	}

	dxl_bulk_read(Port_, t_param, NUM_JOINT*5, pbd);

	for(int i = 0; i < NUM_JOINT; i++){
		dxl_get_bulk_byte(pbd, ID_list_[i], PRO_MOVING, &tP_position[i]);
		printf("ID %d : %d\n", ID_list_[i], tP_position[i]);

		if(ismove != NULL){
			ismove[i] = tP_position[i] == 1 ? true : false;
		}
	}

	free(t_param);

	return 1;
}

int DynamixelPro::GetPresPosition(int *PresentPosition){
	int tP_position[NUM_JOINT],tPosition_length = 4;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_JOINT*5));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(PRO_PRESENT_POSITION);
		t_param[i*5+2] = DXL_HIBYTE(PRO_PRESENT_POSITION);
		t_param[i*5+3] = DXL_LOBYTE(tPosition_length);
		t_param[i*5+4] = DXL_HIBYTE(tPosition_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_JOINT*5, pbd);

	for(int i = 0; i < NUM_JOINT; i++){
		dxl_get_bulk_dword(pbd, ID_list_[i], PRO_PRESENT_POSITION, (unsigned*)&tP_position[i]);

		if(PresentPosition != NULL){
			PresentPosition[i] = tP_position[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tP_position[i]);
	}

	free(t_param);

	return 1;
}

int DynamixelPro::GetGoalPosition(int *GoalPosition){
	int tG_position[NUM_JOINT],tPosition_length = 4;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_JOINT*5));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(PRO_GOAL_POSITION);
		t_param[i*5+2] = DXL_HIBYTE(PRO_GOAL_POSITION);
		t_param[i*5+3] = DXL_LOBYTE(tPosition_length);
		t_param[i*5+4] = DXL_HIBYTE(tPosition_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_JOINT*5, pbd);

	for(int i = 0; i < NUM_JOINT; i++){
		dxl_get_bulk_dword(pbd, ID_list_[i], PRO_GOAL_POSITION, (unsigned*)&tG_position[i]);

		if(GoalPosition != NULL){
			GoalPosition[i] = tG_position[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tG_position[i]);

	}

	free(t_param);

	return 1;
}

int DynamixelPro::GetPresVelocity(int *PresentVelocity){
	int tP_velocity[NUM_JOINT],tVelocity_length = 4;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_JOINT*5));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(PRO_PRESENT_VELOCITY);
		t_param[i*5+2] = DXL_HIBYTE(PRO_PRESENT_VELOCITY);
		t_param[i*5+3] = DXL_LOBYTE(tVelocity_length);
		t_param[i*5+4] = DXL_HIBYTE(tVelocity_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_JOINT*5, pbd);

	for(int i = 0; i < NUM_JOINT; i++){
		dxl_get_bulk_dword(pbd, ID_list_[i], PRO_PRESENT_VELOCITY, (unsigned*)&tP_velocity[i]);

		if(PresentVelocity != NULL){
			PresentVelocity[i] = tP_velocity[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tP_velocity[i]);

	}

	free(t_param);

	return 1;
}

int DynamixelPro::GetGoalVelocity(int *GoalVelocity){
	int tG_velocity[NUM_JOINT],tVelocity_length = 4;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_JOINT*5));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(PRO_GOAL_VELOCITY);
		t_param[i*5+2] = DXL_HIBYTE(PRO_GOAL_VELOCITY);
		t_param[i*5+3] = DXL_LOBYTE(tVelocity_length);
		t_param[i*5+4] = DXL_HIBYTE(tVelocity_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_JOINT*5, pbd);

	for(int i = 0; i < NUM_JOINT; i++){
		dxl_get_bulk_dword(pbd, ID_list_[i], PRO_GOAL_VELOCITY, (unsigned*)&tG_velocity[i]);

		if(GoalVelocity != NULL){
			GoalVelocity[i] = tG_velocity[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tG_velocity[i]);

	}

	free(t_param);

	return 1;
}

int DynamixelPro::SetGoalPosition(int *GoalPosition){
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_JOINT*5));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(DXL_LOWORD(GoalPosition[i]));
		t_param[i*5+2] = DXL_HIBYTE(DXL_LOWORD(GoalPosition[i]));
		t_param[i*5+3] = DXL_LOBYTE(DXL_HIWORD(GoalPosition[i]));
		t_param[i*5+4] = DXL_HIBYTE(DXL_HIWORD(GoalPosition[i]));
	}

	int Result = dxl_sync_write(Port_, PRO_GOAL_POSITION, 4, t_param, NUM_JOINT*5);
	if( Result != COMM_RXSUCCESS )
	{
		printf( "Failed to set goal position!\n" );
		free(t_param);
		return -1;
	}

	free(t_param);
	return 1;
}

int DynamixelPro::SetGoalVelocity(int *GoalVelocity){
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_JOINT*5));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(DXL_LOWORD(GoalVelocity[i]));
		t_param[i*5+2] = DXL_HIBYTE(DXL_LOWORD(GoalVelocity[i]));
		t_param[i*5+3] = DXL_LOBYTE(DXL_HIWORD(GoalVelocity[i]));
		t_param[i*5+4] = DXL_HIBYTE(DXL_HIWORD(GoalVelocity[i]));
	}

	int Result = dxl_sync_write(Port_, PRO_GOAL_VELOCITY, 4, t_param, NUM_JOINT*5);
	if( Result != COMM_RXSUCCESS )
	{
		printf( "Failed to set goal position!\n" );
		free(t_param);
		return -1;
	}

	free(t_param);
	return 1;
}

//RED LED
int DynamixelPro::SetLED(unsigned char *LED){
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_JOINT*2));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*2] = ID_list_[i];
		t_param[i*2+1] = LED[i];
	}

	int Result = dxl_sync_write(Port_, PRO_LED_RED, 1, t_param, NUM_JOINT*2);
	if( Result != COMM_RXSUCCESS )
	{
		printf( "Failed to change red LED!\n" );
		free(t_param);
		return -1;
	}

	free(t_param);
	return 1;
}

int DynamixelPro::GetPresLoad(int *PresentLoad){
	int tP_load[NUM_JOINT], tLoad_length = 2;
	
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_JOINT*5));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(PRO_PRESENT_LOAD);
		t_param[i*5+2] = DXL_HIBYTE(PRO_PRESENT_LOAD);
		t_param[i*5+3] = DXL_LOBYTE(tLoad_length);
		t_param[i*5+4] = DXL_HIBYTE(tLoad_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_JOINT*5, pbd);

	for(int i = 0; i < NUM_JOINT; i++){
		dxl_get_bulk_word(pbd, ID_list_[i], PRO_PRESENT_LOAD, &tP_load[i]);

		if(PresentLoad != NULL){
			PresentLoad[i] = tP_load[i];
		}else{
			printf("ID %d : %d\n", ID_list_[i], (short)tP_load[i]);
		}
	}

	free(t_param);

	return 1;
}

int DynamixelPro::GetPresCurrent(int *PresentCurrent){
	int tP_current[NUM_JOINT], tCurrent_length = 2;
	
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_JOINT*5));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(PRO_PRESENT_CURRENT);
		t_param[i*5+2] = DXL_HIBYTE(PRO_PRESENT_CURRENT);
		t_param[i*5+3] = DXL_LOBYTE(tCurrent_length);
		t_param[i*5+4] = DXL_HIBYTE(tCurrent_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_JOINT*5, pbd);

	for(int i = 0; i < NUM_JOINT; i++){
		dxl_get_bulk_word(pbd, ID_list_[i], PRO_PRESENT_CURRENT, &tP_current[i]);

		if(PresentCurrent != NULL){
			PresentCurrent[i] = tP_current[i];
		}else{
			printf("%d\t", (short)tP_current[i]);
		}
	}
	printf("\n");

	free(t_param);

	return 1;
}