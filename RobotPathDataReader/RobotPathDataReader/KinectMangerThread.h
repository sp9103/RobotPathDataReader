#pragma once
#include <process.h>

#include "KinectConnecter.h"
#include "stdafx.h"

class KinectMangerThread
{
public:
	KinectMangerThread(void);
	~KinectMangerThread(void);

	void Initialize(cv::Rect srcROI);
	void Deinitialize();
	cv::Mat getImg();
	cv::Mat getDepth();
	cv::Mat getPointCloud();
	bool isThreadDead();

private:
	bool endCheck;
	bool loopClose;
	CRITICAL_SECTION cs;
	cv::Mat frame_, depth_, pointCloud_;
	cv::Rect imgROI;

	static UINT WINAPI KinectThread(LPVOID param); // 쓰레드 함수.
};

