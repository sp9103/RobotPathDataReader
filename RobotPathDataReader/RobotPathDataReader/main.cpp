#include <stdio.h>
#include <afx.h>
#include <stdlib.h>

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

#define DEFAULT_PATH "data"

int main(){
	RobotArm arm;
	ColorBasedTracker tracker;
	KinectMangerThread kinectManager;

	//variable
	cv::Rect RobotROI((KINECT_DEPTH_WIDTH - 160) / 2 + 40, (KINECT_DEPTH_HEIGHT- 160) / 2, 160, 160);

	//initialize
	kinectManager.Initialize(RobotROI);

	kinectManager.Deinitialize();

	return 0;
}