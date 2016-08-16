#include "RobotArm.h"


RobotArm::RobotArm(void)
{
	sp_.dPacketStartTime = 0;
	sp_.fByteTransferTime = 0;
	sp_.fPacketWaitTime = 0;
	sp_.hComm = 0;
	sp_.iBusUsing = 0;

	Port_ = &sp_;
}


RobotArm::~RobotArm(void)
{
}

int RobotArm::Init(int PortNum, int BaudRateNum, int *ID_list){
	SetID(ID_list);

	Com_port_num_ = PortNum;
	Baud_rate_num_ = BaudRateNum;

	if(dxl_initialize(Port_, Com_port_num_, Baud_rate_num_) == COMM_RXSUCCESS )
		printf("Succeed to open USB2Dynamixel!\n");
	else
	{
		printf( "Failed to open USB2Dynamixel!\n" );
		return -1;
	}
#ifdef USING_FINGER
	fingercontroller_.Init(sp_, fingerID_);
#endif
	jointcontroller_.Init(sp_, jointID_);

	return 1;
}

int RobotArm::DeInit(){

	dxl_terminate(Port_);

	return 1;
}

int RobotArm::SetID(int *ID_list){
	//각각 아이디 부여
	int i;
	for(i = 0; i < NUM_JOINT; i++)
		jointID_[i] = ID_list[i];
	for(int j = 0; j < NUM_FINGER; j++)
		fingerID_[j] = ID_list[i++];

	return 1;
}

int RobotArm::TorqueOn(){
#ifdef USING_FINGER
	fingercontroller_.TorqueOn();
#endif
	jointcontroller_.TorqueOn();

	printf("=================Torque ON====================\n");
	return 1;
}

int RobotArm::TorqueOff(){
#ifdef USING_FINGER
	fingercontroller_.TorqueOff();
#endif
	jointcontroller_.TorqueOff();

	printf("=================Torque OFF====================\n");
	return 1;
}

int RobotArm::GetTemperature(int *Temperature){
	if(!Temperature){
		printf("Present Temperature\n");
		jointcontroller_.GetTemperature();
#ifdef USING_FINGER
		fingercontroller_.GetTemperature();
#endif
	}else{
		jointcontroller_.GetTemperature(Temperature);
#ifdef USING_FINGER
		fingercontroller_.GetTemperature(&Temperature[NUM_JOINT]);
#endif
	}

	return 1;
}

int RobotArm::isMoving(bool *ismove){
	if(!ismove){
		printf("Moving State\n");
		jointcontroller_.isMoving();
#ifdef USING_FINGER
		fingercontroller_.isMoving();
#endif
	}else{
		jointcontroller_.isMoving(ismove);
#ifdef USING_FINGER
		fingercontroller_.isMoving(&ismove[NUM_JOINT]);
#endif
	}

	return 1;
}

int RobotArm::GetPresPosition(int *PresentPosition){
	if(!PresentPosition){
		printf("Present position\n");
		jointcontroller_.GetPresPosition();
#ifdef USING_FINGER
		fingercontroller_.GetPresPosition();
#endif
	}else{
		jointcontroller_.GetPresPosition(PresentPosition);
#ifdef USING_FINGER
		fingercontroller_.GetPresPosition(&PresentPosition[NUM_JOINT]);
#endif
	}

	return 1;
}

int RobotArm::GetGoalPosition(int *GoalPosition){
	if(!GoalPosition){
		printf("Goal position\n");
		jointcontroller_.GetGoalPosition();
#ifdef USING_FINGER
		fingercontroller_.GetGoalPosition();
#endif
	}else{
		jointcontroller_.GetGoalPosition(GoalPosition);
#ifdef USING_FINGER
		fingercontroller_.GetGoalPosition(&GoalPosition[NUM_JOINT]);
#endif
	}

	return 1;
}

int RobotArm::GetPresVelocity(int *PresentVelocity){
	if(!PresentVelocity){
		printf("Present velocity\n");
		jointcontroller_.GetPresVelocity();
#ifdef USING_FINGER
		fingercontroller_.GetPresVelocity();
#endif
	}else{
		jointcontroller_.GetPresVelocity(PresentVelocity);
#ifdef USING_FINGER
		fingercontroller_.GetPresVelocity(&PresentVelocity[NUM_JOINT]);
#endif
	}

	return 1;
}
int RobotArm::GetGoalVelocity(int *GoalVelocity){
	if(!GoalVelocity){
		printf("Goal velocity\n");
		jointcontroller_.GetGoalVelocity();
#ifdef USING_FINGER
		fingercontroller_.GetGoalVelocity();
#endif
	}else{
		jointcontroller_.GetGoalVelocity(GoalVelocity);
#ifdef USING_FINGER
		fingercontroller_.GetGoalVelocity(&GoalVelocity[NUM_JOINT]);
#endif
	}

	return 1;
}

int RobotArm::SetGoalVelocity(int *GoalVelocity){
	if(!jointcontroller_.SetGoalVelocity(GoalVelocity))					return -1;
#ifdef USING_FINGER
	if(!fingercontroller_.SetGoalVelocity(&GoalVelocity[NUM_JOINT]))	return -1;
#endif

	return 1;
}

int RobotArm::SetGoalPosition(int *GoalPosition){
	if(!jointcontroller_.SetGoalPosition(GoalPosition))					return -1;
#ifdef USING_FINGER
	if(!fingercontroller_.SetGoalPosition(&GoalPosition[NUM_JOINT]))	return -1;
#endif

	return 1;
}

int RobotArm::SetLED(bool onoff){
	unsigned char LEDPRO[NUM_JOINT];
	unsigned char LEDMX[NUM_FINGER];

	unsigned char onoffval = onoff == true ? 255 : 0;

	for(int i = 0; i < NUM_JOINT; i++)
		LEDPRO[i] = onoffval;
	for(int i = 0; i < NUM_FINGER; i++)
		LEDMX[i] = onoffval;

	if(!jointcontroller_.SetLED(LEDPRO))					return -1;
#ifdef USING_FINGER
	if(!fingercontroller_.SetLED(LEDMX))					return -1;
#endif

	return 1;
}

int RobotArm::GetFingerLoad(int *load){
#ifdef USING_FINGER
	if(!load){
		printf("present Load\n");
		fingercontroller_.GetPresLoad();
	}else{
		//jointcontroller_.GetPresCurrent();
		fingercontroller_.GetPresLoad(load);
	}
#endif

	return 1;
}

int RobotArm::SetFingerPosition(int *GoalPosition){
#ifdef USING_FINGER
	if(!fingercontroller_.SetGoalPosition(GoalPosition))					return -1;
#endif
	return 1;
}

int RobotArm::Arm_Get_JointValue(Eigen::VectorXi *value)
{
	value->resize(NUM_JOINT);

	int temp_val[9];
	GetPresPosition(temp_val);

	for(int i = 0; i < NUM_JOINT; i++){
		(*value)[i] = temp_val[i];
	}

	return 1;
}

SerialPort* RobotArm::DXL_Get_Port(void)
{
	return Port_;
}
void RobotArm::safeReleasePose(){
	int ReleasePose[] = {0,0,0,0,0,0};
	int presPose[6];
	int goalPose[6];

	jointcontroller_.GetPresPosition(presPose);
	memcpy(goalPose, presPose, sizeof(int) * 6);
	goalPose[1] = ReleasePose[1];
	jointcontroller_.SetGoalPosition(goalPose);
	waitMove();
	goalPose[3] = ReleasePose[3];
	jointcontroller_.SetGoalPosition(goalPose);
	waitMove();
	jointcontroller_.SetGoalPosition(ReleasePose);
	waitMove();
}

bool RobotArm::waitMove(){
	int presPose[6], goalPose[6];

	while(1){
		jointcontroller_.GetPresPosition(presPose);
		jointcontroller_.GetGoalPosition(goalPose);
		int max = -9999;
		for(int i = 0; i < 6; i++){
			int sub = abs(presPose[i] - goalPose[i]);
			if(sub > max)	max = sub;
		}
		if(max < 30)	break;
		Sleep(10);
	}

	return true;
}

void RobotArm::safeMovePose(int *goal){
	int presPose[6];

	jointcontroller_.GetPresPosition(presPose);
	presPose[0] = goal[0];
	presPose[2] = goal[2];
	presPose[4] = goal[4];
	presPose[5] = goal[5];
	jointcontroller_.SetGoalPosition(presPose);
	waitMove();
	presPose[3] = goal[3];
	jointcontroller_.SetGoalPosition(presPose);
	waitMove();
	jointcontroller_.SetGoalPosition(goal);
	waitMove();
}