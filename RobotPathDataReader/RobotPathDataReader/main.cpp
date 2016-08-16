#include <stdio.h>
#include <afx.h>
#include <stdlib.h>
#include <conio.h>
#include <strsafe.h>
#include <Windows.h>

#include "ColorBasedTracker.h"
#include "Robot\RobotArm.h"
#include "ARMSDK\include\ARMSDK.h"
#include "KinectMangerThread.h"

#ifdef _DEBUG
#pragma comment(lib, "ARMSDKd.lib")
#endif
#ifdef NDEBUG
#pragma comment(lib, "ARMSDK.lib") 
#endif

#define DEFAULT_WRITE_PATH "data"
#define DEFAULT_READ_PATH "D:\\Users\\sp\\Documents\\RobotPathDataRecorder\\RobotPathDataRecorder\\RobotPathDataRecorder\\data"

void ControllerInit(RobotArm *robot);
bool robotConnectCheck(RobotArm *robot, armsdk::RobotInfo *robotinfo, armsdk::Kinematics *kin);

int main(){
	RobotArm arm;
	ColorBasedTracker tracker;
	KinectMangerThread kinectManager;
	armsdk::RobotInfo robot;
	armsdk::Kinematics kin;

	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	TCHAR szDir[MAX_PATH] = { 0, };

	//variable
	cv::Rect RobotROI((KINECT_DEPTH_WIDTH - 160) / 2 + 40, (KINECT_DEPTH_HEIGHT- 160) / 2, 160, 160);

	//initialize
	kinectManager.Initialize(RobotROI);
	ControllerInit(&arm);
	if(robotConnectCheck(&arm, &robot, &kin))
		return -1;
	arm.TorqueOn();

	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, DEFAULT_READ_PATH, strlen(DEFAULT_READ_PATH), szDir, MAX_PATH);
	StringCchCat(szDir, MAX_PATH, TEXT("\\*"));
	hFind = FindFirstFile(szDir, &ffd);
	while (FindNextFile(hFind, &ffd) != 0){
		TCHAR subDir[MAX_PATH] = { 0, };
		memcpy(subDir, szDir, sizeof(TCHAR)*MAX_PATH);
		size_t len;
		StringCchLength(subDir, MAX_PATH, &len);
		subDir[len - 1] = '\0';
		StringCchCat(subDir, MAX_PATH, ffd.cFileName);
		char tBuf[MAX_PATH];
		WideCharToMultiByte(CP_ACP, 0, subDir, MAX_PATH, tBuf, MAX_PATH, NULL, NULL);

		//Tchar to char
		char ccFileName[256];
		WideCharToMultiByte(CP_ACP, 0, ffd.cFileName, len, ccFileName, 256, NULL, NULL);
		printf("directory : %s load.\n", ccFileName);
		if (ccFileName[0] != '.'){
			//시작부
			//background store
			arm.safeReleasePose();
			cv::Mat backRGB = kinectManager.getImg();
			cv::Mat backDepth = kinectManager.getDepth();
			if(backRGB.channels() == 4)	cv::cvtColor(backRGB, backRGB, CV_BGRA2BGR);
			cv::imshow("background", backRGB);
			cv::waitKey(1);
		}
	}

	kinectManager.Deinitialize();

	return 0;
}

void ControllerInit(RobotArm *robot){
	int robotid[] = {1,3,5,7,9,11,13,15,17};
	int vel[] = {2000, 2000, 2000, 2000, 2000, 2000, 50, 50, 50};
	//Upper Left, UpperRight, Thumb

	robot->Init(6,3, robotid);

	robot->SetGoalVelocity(vel);
}

bool robotConnectCheck(RobotArm *robot, armsdk::RobotInfo *robotinfo, armsdk::Kinematics *kin){
	veci angi(6);
	robot->Arm_Get_JointValue(&angi);

#ifdef RIGHT_ARM_USE
	//RightArm
	robotinfo->AddJoint(  0.0,  ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 1);
	robotinfo->AddJoint(  0.0, -ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 3);
	robotinfo->AddJoint( 30.0, -ML_PI_2,  246.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 5);
	robotinfo->AddJoint(-30.0,  ML_PI_2,    0.0,  ML_PI_2, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 7);
	robotinfo->AddJoint(  0.0, -ML_PI_2,  216.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 9);
	robotinfo->AddJoint(  0.0,  ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 11);
#elif defined LEFT_ARM_USE
	//Leftarm
	robotinfo->AddJoint(  0.0, -ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 2);
	robotinfo->AddJoint(  0.0,  ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 4);
	robotinfo->AddJoint( 30.0,  ML_PI_2,  246.0,      0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 6);
	robotinfo->AddJoint(-30.0, -ML_PI_2,    0.0, -ML_PI_2, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 8);
	robotinfo->AddJoint(  0.0,  ML_PI_2,  216.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 10);
	robotinfo->AddJoint(  0.0, -ML_PI_2,    0.0,      0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 12);
#endif
	kin->InitRobot(robotinfo);

	//맥시멈 앵글 체크 - 쓰레기값 걸러내기
	for(int JointNum = 0; JointNum < 6; JointNum++)
	{
		if(abs(angi[JointNum]) > robotinfo->GetJointInfo(JointNum)->GetMaxAngleInValue() + 10)
		{
			cout<<"read fail"<<endl;
			printf("Data Fail %d\n", angi[JointNum]);
			return false;
		}
	}
}