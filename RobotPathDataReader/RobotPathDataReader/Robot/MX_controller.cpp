#include "MX_controller.h"


MX_controller::MX_controller(void)
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


MX_controller::~MX_controller(void)
{
}

void MX_controller::Init(SerialPort port, int *id){
	for(int i = 0; i < NUM_FINGER; i++)
		ID_list_[i] = id[i];

	sp_ = port;
}

int MX_controller::TorqueOn(){
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(char)*(NUM_FINGER*2));

	for(int i = 0; i < NUM_FINGER; i++){
		t_param[i*2] = (unsigned char)ID_list_[i];
		t_param[i*2+1] = 1;
	}

	int Result = dxl_sync_write(Port_, MX_TORQUE_ENABLE, 1, t_param, NUM_FINGER*2);
	if( Result != COMM_RXSUCCESS )
	{
		printf( "Failed MX Enable!\n" );
		free(t_param);
		return -1;
	}

	free(t_param);
	return 1;
}

int MX_controller::TorqueOff(){
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(char)*(NUM_FINGER*2));

	for(int i = 0; i < NUM_FINGER; i++){
		t_param[i*2] = (unsigned char)ID_list_[i];
		t_param[i*2+1] = 0;
	}

	int Result = dxl_sync_write(Port_, MX_TORQUE_ENABLE, 1, t_param, NUM_FINGER*2);
	if( Result != COMM_RXSUCCESS )
	{
		printf( "Failed MX Disable!\n" );
		free(t_param);
		return -1;
	}

	free(t_param);
	return 1;
}

int MX_controller::GetTemperature(int *Temperature){
	int tP_temperature[NUM_FINGER], temperature_length = 1;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_FINGER*5));

	for(int i = 0; i < NUM_FINGER; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(MX_PRESENT_TEMPERATURE);
		t_param[i*5+2] = DXL_HIBYTE(MX_PRESENT_TEMPERATURE);
		t_param[i*5+3] = DXL_LOBYTE(temperature_length);
		t_param[i*5+4] = DXL_HIBYTE(temperature_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_FINGER*5, pbd);

	for(int i = 0; i < NUM_FINGER; i++){
		dxl_get_bulk_byte(pbd, ID_list_[i], MX_PRESENT_TEMPERATURE, &tP_temperature[i]);

		if(Temperature != NULL){
			Temperature[i] = tP_temperature[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tP_temperature[i]);
	}

	free(t_param);

	return 1;
}

int MX_controller::isMoving(bool *ismove){
	int tP_position[NUM_FINGER],tIsMovingLength = 1;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_FINGER*5));

	for(int i = 0; i < NUM_FINGER; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(MX_MOVING);
		t_param[i*5+2] = DXL_HIBYTE(MX_MOVING);
		t_param[i*5+3] = DXL_LOBYTE(tIsMovingLength);
		t_param[i*5+4] = DXL_HIBYTE(tIsMovingLength);
	}

	dxl_bulk_read(Port_, t_param, NUM_FINGER*5, pbd);

	for(int i = 0; i < NUM_FINGER; i++){
		dxl_get_bulk_byte(pbd, ID_list_[i], MX_MOVING, &tP_position[i]);

		if(ismove != NULL){
			ismove[i] = tP_position[i] == 1 ? true : false;
		}else
			printf("ID %d : %d\n", ID_list_[i], tP_position[i]);

	}

	free(t_param);

	return 1;
}

int MX_controller::GetPresPosition(int *PresentPosition){
	int tP_position[NUM_FINGER],tPosition_length = 2;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_FINGER*5));

	for(int i = 0; i < NUM_FINGER; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(MX_PRESENT_POSITION);
		t_param[i*5+2] = DXL_HIBYTE(MX_PRESENT_POSITION);
		t_param[i*5+3] = DXL_LOBYTE(tPosition_length);
		t_param[i*5+4] = DXL_HIBYTE(tPosition_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_FINGER*5, pbd);

	for(int i = 0; i < NUM_FINGER; i++){
		dxl_get_bulk_word(pbd, ID_list_[i], MX_PRESENT_POSITION, &tP_position[i]);

		if(PresentPosition != NULL){
			PresentPosition[i] = tP_position[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tP_position[i]);

	}

	free(t_param);

	return 1;
}

int MX_controller::GetGoalPosition(int *GoalPosition){
	int tG_position[NUM_FINGER],tPosition_length = 2;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_FINGER*5));

	for(int i = 0; i < NUM_FINGER; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(MX_GOAL_POSITION);
		t_param[i*5+2] = DXL_HIBYTE(MX_GOAL_POSITION);
		t_param[i*5+3] = DXL_LOBYTE(tPosition_length);
		t_param[i*5+4] = DXL_HIBYTE(tPosition_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_FINGER*5, pbd);

	for(int i = 0; i < NUM_FINGER; i++){
		dxl_get_bulk_word(pbd, ID_list_[i], MX_GOAL_POSITION, &tG_position[i]);

		if(GoalPosition != NULL){
			GoalPosition[i] = tG_position[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tG_position[i]);
	}

	free(t_param);

	return 1;
}

int MX_controller::GetPresVelocity(int *PresentVelocity){
	int tP_velocity[NUM_FINGER],tVelocity_length = 2;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_FINGER*5));

	for(int i = 0; i < NUM_FINGER; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(MX_PRESENT_SPEED);
		t_param[i*5+2] = DXL_HIBYTE(MX_PRESENT_SPEED);
		t_param[i*5+3] = DXL_LOBYTE(tVelocity_length);
		t_param[i*5+4] = DXL_HIBYTE(tVelocity_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_FINGER*5, pbd);

	for(int i = 0; i < NUM_FINGER; i++){
		dxl_get_bulk_word(pbd, ID_list_[i], MX_PRESENT_SPEED, &tP_velocity[i]);

		if(PresentVelocity != NULL){
			PresentVelocity[i] = tP_velocity[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tP_velocity[i]);

	}

	free(t_param);

	return 1;
}

int MX_controller::GetGoalVelocity(int *GoalVelocity){
	int tG_velocity[NUM_FINGER],tVelocity_length = 2;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_FINGER*5));

	for(int i = 0; i < NUM_FINGER; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(MX_MOVING_SPEED);
		t_param[i*5+2] = DXL_HIBYTE(MX_MOVING_SPEED);
		t_param[i*5+3] = DXL_LOBYTE(tVelocity_length);
		t_param[i*5+4] = DXL_HIBYTE(tVelocity_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_FINGER*5, pbd);

	for(int i = 0; i < NUM_FINGER; i++){
		dxl_get_bulk_word(pbd, ID_list_[i], MX_MOVING_SPEED, &tG_velocity[i]);

		if(GoalVelocity != NULL){
			GoalVelocity[i] = tG_velocity[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tG_velocity[i]);
	}

	free(t_param);

	return 1;
}

int MX_controller::SetGoalVelocity(int *GoalVelocity){
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_FINGER*3));

	for(int i = 0; i < NUM_FINGER; i++){
		t_param[i*3] = (unsigned char)ID_list_[i];
		t_param[i*3+1] = DXL_LOBYTE(DXL_LOWORD(GoalVelocity[i]));
		t_param[i*3+2] = DXL_HIBYTE(DXL_LOWORD(GoalVelocity[i]));
		/*t_param[i*5+3] = DXL_LOBYTE(DXL_HIWORD(GoalVelocity[i]));
		t_param[i*5+4] = DXL_HIBYTE(DXL_HIWORD(GoalVelocity[i]));*/
	}

	int Result = dxl_sync_write(Port_, MX_MOVING_SPEED, 2, t_param, NUM_FINGER*3);
	if( Result != COMM_RXSUCCESS )
	{
		printf( "Failed to set goal position!\n" );
		free(t_param);
		return -1;
	}

	free(t_param);
	return 1;
}

int MX_controller::SetGoalPosition(int *GoalPosition){
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_FINGER*3));

	for(int i = 0; i < NUM_FINGER; i++){
		t_param[i*3] = (unsigned char)ID_list_[i];
		t_param[i*3+1] = DXL_LOBYTE(DXL_LOWORD(GoalPosition[i]));
		t_param[i*3+2] = DXL_HIBYTE(DXL_LOWORD(GoalPosition[i]));
		/*t_param[i*5+3] = DXL_LOBYTE(DXL_HIWORD(GoalPosition[i]));
		t_param[i*5+4] = DXL_HIBYTE(DXL_HIWORD(GoalPosition[i]));*/
	}

	int Result = dxl_sync_write(Port_, MX_GOAL_POSITION, 2, t_param, NUM_FINGER*3);
	if( Result != COMM_RXSUCCESS )
	{
		printf( "Failed to set goal position!\n" );
		free(t_param);
		return -1;
	}

	free(t_param);
	return 1;
}

int MX_controller::SetLED(unsigned char *LED){
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(char)*(NUM_FINGER*2));

	for(int i = 0; i < NUM_FINGER; i++){
		t_param[i*2] = (unsigned char)ID_list_[i];
		t_param[i*2+1] = LED[i] > 0 ? 1:0;
	}

	int Result = dxl_sync_write(Port_, MX_LED_ONOFF, 1, t_param, NUM_FINGER*2);
	if( Result != COMM_RXSUCCESS )
	{
		printf( "Failed MX LED ON!\n" );
		free(t_param);
		return -1;
	}

	free(t_param);

	return 1;
}

int MX_controller::GetPresLoad(int *PresentLoad){
	int tP_load[NUM_FINGER],tLoad_length = 2;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_FINGER*5));

	for(int i = 0; i < NUM_FINGER; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(MX_PRESENT_LOAD);
		t_param[i*5+2] = DXL_HIBYTE(MX_PRESENT_LOAD);
		t_param[i*5+3] = DXL_LOBYTE(tLoad_length);
		t_param[i*5+4] = DXL_HIBYTE(tLoad_length);
	}
	while(1){
		dxl_bulk_read(Port_, t_param, NUM_FINGER*5, pbd);

		for(int i = 0; i < NUM_FINGER; i++){
			dxl_get_bulk_word(pbd, ID_list_[i], MX_PRESENT_LOAD, &tP_load[i]);

			if(PresentLoad != NULL){
				PresentLoad[i] = tP_load[i];
			}else
				printf("ID %d : %d\n", ID_list_[i], tP_load[i]);
		}

		if(DataValidCheck(tP_load, 2048, 0) == 1)
			break;
	}

	free(t_param);


	return 1;
}

int MX_controller::DataValidCheck(int *src, int Max, int Min){

	for(int i = 0; i < NUM_FINGER; i++){
		if(Max < src[i] || src[i] < Min)
			return -1;
	}

	return 1;
}