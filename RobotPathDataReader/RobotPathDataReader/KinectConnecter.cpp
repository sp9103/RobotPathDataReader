#include "KinectConnecter.h"

KinectConnecter::KinectConnecter(void)
{
	m_pColorRGBX = new RGBQUAD[KINECT_COLOR_HEIGHT * KINECT_COLOR_WIDTH];
	m_pDepthRGBX = new RGBQUAD[KINECT_DEPTH_HEIGHT * KINECT_DEPTH_WIDTH];
	pDepthCoordinate = new DepthSpacePoint[KINECT_COLOR_HEIGHT * KINECT_COLOR_WIDTH];
	pColorCoordinate = new ColorSpacePoint[KINECT_COLOR_HEIGHT * KINECT_COLOR_WIDTH];

	MapDepthToByte = 8000 / 256;

	pDepthBuffer = NULL;

	colorRAWFrameMat.create(KINECT_COLOR_HEIGHT, KINECT_COLOR_WIDTH, CV_8UC4);
}


KinectConnecter::~KinectConnecter(void)
{
	if(m_pColorRGBX){
		delete [] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}
	if(m_pDepthRGBX){
		delete [] m_pDepthRGBX;
		m_pDepthRGBX = NULL;
	}

	SafeRelease(m_pBodyFrameReader);
	SafeRelease(m_pCoordinateMapper);
	//SafeRelease(m_pBodyIndexFrameReader);

	Kmat.release();
	Rmat.release();
	Tmat.release();
}

HRESULT KinectConnecter::KinectInitialize(){
	printf("Start Kinect Initialize...\n");
	HRESULT hrc = GetDefaultKinectSensor(&m_pKinectSensor);				//Kinect Color Frame
	HRESULT hrd = NULL;													//Kinect Depth Frame
	HRESULT hrs = NULL;

	if(FAILED(hrc)){
		printf("KinectSeonsor Open fail. Check Kinect Sensor connected.\n");
		return hrc;
	}

	if(m_pKinectSensor){
		// Initialize the Kinect and get the color reader
		IColorFrameSource*	pColorFrameSource = NULL;
		IDepthFrameSource*	pDepthFrameSource = NULL;
		IBodyFrameSource*	pBodyFrameSource = NULL;
		//IBodyIndexFrameSource*	pBodyIndexFrameSource = NULL;

		hrc = m_pKinectSensor->Open();
		printf("Kinect Sensor open Complte!\n");

		if(SUCCEEDED(hrc)){
			hrc = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
			hrd = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);

			m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
			hrs = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
			//m_pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);				//body index Frame source open
		}

		if(SUCCEEDED(hrc) && SUCCEEDED(hrd) && SUCCEEDED(hrs)){
			hrc = pColorFrameSource->OpenReader(&m_pColorFrameReader);
			hrd = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
			hrs = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
			//pBodyIndexFrameSource->OpenReader(&m_pBodyIndexFrameReader);					//body index Frame reader open
		}

		if(SUCCEEDED(hrc)){
			//WCHAR temp[256] = L"";
			hrc = m_pKinectSensor->get_UniqueKinectId(_countof(UniqueID), UniqueID);
			printf("Kinect ID : %s\n", UniqueID);
		}

		SafeRelease(pColorFrameSource);
		SafeRelease(pDepthFrameSource);
		SafeRelease(pBodyFrameSource);
		//SafeRelease(pBodyIndexFrameSource);
	}

	if(!m_pKinectSensor || FAILED(hrc) || FAILED(hrd) || FAILED(hrs)){
		printf("No ready Kinect found!\n");
		return E_FAIL;
	}

	printf("Kinect initialzie Complete\n");

	return hrc;
}

void KinectConnecter::KinectDestroy(){
	printf("Start Kinect Destroy...\n");
	if(m_pKinectSensor){
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
	printf("Kinect Destroy Complete\n");
}

void KinectConnecter::GetColorImage(Mat *src){

	if(!m_pColorFrameReader){
		return;
	}

	IColorFrame* pColorFrame = NULL;

	HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);

	if(SUCCEEDED(hr)){
		INT64 nTime = 0;
		IFrameDescription* pFrameDescription = NULL;
		int nWidth	= 0;
		int nHeight = 0;
		ColorImageFormat ImageFormat = ColorImageFormat_None;
		UINT nBUfferSize = 0;
		RGBQUAD *pBuffer = NULL;

		hr = pColorFrame->get_RelativeTime(&nTime);

		if(SUCCEEDED(hr)){
			hr = pColorFrame->get_FrameDescription(&pFrameDescription);
		}

		//Check Color Image Width & Height
		if(SUCCEEDED(hr)){
			pFrameDescription->get_Width(&nWidth);
			pFrameDescription->get_Height(&nHeight);
			hr = pColorFrame->get_RawColorImageFormat(&ImageFormat);
		}

		if(SUCCEEDED(hr)){
			if(ImageFormat == ColorImageFormat_Bgra){
				//if Image format is BGRA -> copy image direct.
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBUfferSize, reinterpret_cast<BYTE**>(&pBuffer));
			}
			else if(m_pColorRGBX){
				//Default Image format Yuy2
				pBuffer = m_pColorRGBX;
				nBUfferSize = KINECT_COLOR_HEIGHT * KINECT_COLOR_WIDTH * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBUfferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
			}
			else
				hr = E_FAIL;
		}

		//Image Format Convert
		if(SUCCEEDED(hr)){
			ConvertOpencvColorImage(src, pBuffer, nBUfferSize);
			colorRAWFrameMat = src->clone();
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pColorFrame);
}

void KinectConnecter::GetBodyIndexImage(Mat *src){
	if( !m_pBodyIndexFrameReader){
		return;
	}

	IBodyIndexFrame* pBodyIndexFrame = NULL;

	HRESULT hr = m_pBodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);

	if(SUCCEEDED(hr)){
		INT64 nTime = 0;
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		UINT nBufferSize = 0;
		BYTE *pBuffer	= NULL;

		hr = pBodyIndexFrame->get_RelativeTime(&nTime);

		if(SUCCEEDED(hr)){
			hr = pBodyIndexFrame->get_FrameDescription(&pFrameDescription);
		}

		if(SUCCEEDED(hr)){
			pFrameDescription->get_Width(&nWidth);
			pFrameDescription->get_Height(&nHeight);
			hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}

		if(SUCCEEDED(hr)){
			ConvertOpencvBodyIdxImage(src, pBuffer, nHeight, nWidth);
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pBodyIndexFrame);
}

void KinectConnecter::GetDepthImage(Mat *src){
	if( !m_pDepthFrameReader){
		return;
	}

	IDepthFrame* pDepthFrame = NULL;

	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	if(SUCCEEDED(hr)){
		INT64 nTime = 0;
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance	= 0;
		USHORT nDepthMaxDistance			= 0;
		UINT nBufferSize = 0;

		hr = pDepthFrame->get_RelativeTime(&nTime);

		if(SUCCEEDED(hr)){
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		}

		if(SUCCEEDED(hr)){
			pFrameDescription->get_Width(&nWidth);
			pFrameDescription->get_Height(&nHeight);
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);

			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nDepthMaxDistance = USHRT_MAX;

			//// 각 어플리케이션에서 최장거리 제한이 필요한 경우 아래 코드 수정..
			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
			hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
			nDepthMaxDistance = 1200;
		}

		if(SUCCEEDED(hr)){
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pDepthBuffer);
		}

		if(SUCCEEDED(hr)){
			ConvertOpencvGrayImage(src, pDepthBuffer, nHeight, nWidth, nDepthMinReliableDistance, nDepthMaxDistance);
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pDepthFrame);
}

void KinectConnecter::ConvertOpencvColorImage(cv::Mat *src, RGBQUAD* pBuffer, int nBufferSize){
	memcpy(src->data, pBuffer, nBufferSize);
}

void KinectConnecter::ConvertOpencvGrayImage(cv::Mat *src, UINT16* pBuffer, int nHeight, int nWidth, int nMinDepth, int nMaxDepth){
	src->setTo(Scalar::all(0));
	int Rangle = (nMaxDepth/* - nMinDepth*/)/256;
	if(m_pDepthRGBX && pBuffer){
		//RGBQUAD* pRGBX = m_pDepthRGBX;

		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		for(int i = 0; i < nWidth*nHeight; i++){
			USHORT depth = *pBuffer;

			for(int j = 0; j < 4; j++){
				src->data[4*i+j] = (byte)(depth >= nMinDepth && depth <= nMaxDepth ? ((depth - nMinDepth) * 255 / (nMaxDepth - nMinDepth)) : 0);
				//src->data[4*i+j] = (byte)(depth >= nMinDepth && depth <= 1500 ? (depth / MapDepthToByte) : 0);


				/*if(depth > nMaxDepth)
				src->data[4*i+j] = 255;
				else if(depth < nMinDepth)
				src->data[4*i+j] = 0;
				else
				src->data[4*i+j] = depth / Rangle;*/

			}
			pBuffer++;
		}
	}
}

void KinectConnecter::ConvertOpencvBodyIdxImage(cv::Mat *src, BYTE* pBuffer, int nHeight, int nWidth){
	//src->setTo(Scalar::all(0));
	cv::Mat tempImage;
	tempImage.create(nHeight, nWidth, CV_8UC1);
	memcpy(tempImage.data, pBuffer, nHeight*nWidth);
	cv::cvtColor(tempImage, *src, CV_GRAY2BGRA);

	tempImage.release();
}

void KinectConnecter::GetSkeletonPos(SkeletonInfo* m_SkeletonInfo, Mat *src, int mode){
	m_SkeletonInfo->Count = -1;

	if( !m_pBodyFrameReader){
		return;
	}

	IBodyFrame* pBodyFrame = NULL;

	HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

	if(SUCCEEDED(hr)){
		INT64 nTime = 0;

		hr = pBodyFrame->get_RelativeTime(&nTime);

		IBody* ppBodies[BODY_COUNT] = {0};

		if(SUCCEEDED(hr)){
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		}

		//Process raw skeleton data
		if(SUCCEEDED(hr)){
			Mat temp = src->clone();
			ProcessSkel(m_SkeletonInfo, BODY_COUNT, ppBodies, src, mode);												//process & draw body

			/*///////////////////////////////////////////////////
			//double tx = 0.f * PI/180.f;
			//double ty = 0.f * PI/180.f;
			//double tz = 0.f * PI/180.f;

			//double c1 = cos(tx), s1 = sin(tx);
			//double c2 = cos(ty), s2 = sin(ty);
			//double c3 = cos(tz), s3 = sin(tz);

			////Rmat = (Mat_<double>(3,3) << c2*c3, c2*s3, -s2, s1*s2*c3 - c1*s3, s1*s2*s3+c1*c3, s1*c2, c1*s2*c3+s1*s3, c1*s2*s3-s1*c3, c1*c2);

			////tx = atan2(Rmat.at<double>(1,2), Rmat.at<double>(2,2)) *180.f / PI;
			////ty = atan2(-Rmat.at<double>(0,2), sqrt(Rmat.at<double>(0,0)*Rmat.at<double>(0,0) + Rmat.at<double>(0,1)*Rmat.at<double>(0,1))) *180.f / PI;
			////tz = atan2(Rmat.at<double>(0,1), Rmat.at<double>(0,0)) *180.f / PI;


			//double heading = tx;
			//double bank = ty;
			//double attitude = tz;

			//double sa = sin(attitude);
			//double ca = cos(attitude);
			//double sb = sin(bank);
			//double cb = cos(bank);
			//double sh = sin(heading);
			//double ch = cos(heading);

			//Rmat = (Mat_<double>(3,3) << ch*ca, (-ch)*sa*cb+sh*sb, ch*sa*sb+sh*cb, sa, ca*cb, (-ca)*sb, (-sh)*ca, sh*sa*cb+ch*sb, -sh*sa*sb+ch*cb);
			//Rmat = Rmat.t();
			//double x_angle = atan2(Rmat.at<double>(2,1), Rmat.at<double>(2,2)) * 180.f / PI;
			//double y_angle = atan2(-Rmat.at<double>(2,0), sqrt(pow(Rmat.at<double>(2,1), 2.0)+pow(Rmat.at<double>(2,2), 2.0))) * 180.f / PI;
			//double z_angle = atan2(Rmat.at<double>(1,0), Rmat.at<double>(0,0)) * 180.f / PI;
			///////////////////////////////////////////////
			////double heading, bank, attitude;
			//if(Rmat.at<double>(1,0) > 0.998){
			//	heading = atan2(-Rmat.at<double>(0,2), Rmat.at<double>(2,2)) * 180.f / PI;
			//	bank = 0;
			//	attitude = PI / 2.0f * 180.f / PI;
			//}
			//else if(Rmat.at<double>(1,0) < -0.998){
			//	heading = atan2(Rmat.at<double>(0,2), Rmat.at<double>(2,2)) * 180.f / PI;
			//	bank = 0;
			//	attitude = -PI/2.0 * 180.f / PI;
			//}else{
			//	heading = atan2(Rmat.at<double>(2,0), Rmat.at<double>(0,0)) * 180.f / PI;
			//	bank = atan2(Rmat.at<double>(1,2), Rmat.at<double>(1,1)) * 180.f / PI;
			//	attitude = asin(Rmat.at<double>(1,0)) * 180.f / PI;
			//}

			//Tmat = (Mat_<double>(3,1) << 0.f, 0.f, 2.f);*/

			/*Rmat = (Mat_<double>(3,3) << 0.658508f, 0.004087f, 0.752563f, 0.038497f, 0.998493f, -0.039108f, -0.751589f, 0.054724f, 0.657358f);
			double x_angle = atan2(Rmat.at<double>(2,1), Rmat.at<double>(2,2)) * 180.f / PI;
			double y_angle = atan2(-Rmat.at<double>(2,0), sqrt(pow(Rmat.at<double>(2,1), 2.0)+pow(Rmat.at<double>(2,2), 2.0))) * 180.f / PI;
			double z_angle = atan2(Rmat.at<double>(1,0), Rmat.at<double>(0,0)) * 180.f / PI;*/

			//BasisCalibration(m_SkeletonInfo);																			//calibration

			//printf("%d\n", m_SkeletonInfo->Count);

			////////////////////////////////////////////////////////임시로///////////////////////////////////////////////////

			//Point2d tjoint[JointType_Count];

			//for(int i = 0; i < m_SkeletonInfo->Count; i++){
			//	for(int j = 0; j < JointType_Count; j++){
			//		//printf("before projection : %f %f %f\n", m_SkeletonInfo->InfoBody[i].JointPos[j].Position.X, m_SkeletonInfo->InfoBody[i].JointPos[j].Position.Y, m_SkeletonInfo->InfoBody[i].JointPos[j].Position.Z);
			//		tjoint[j] = BodyToScreen(m_SkeletonInfo->InfoBody[i].JointPos[j].Position, mode);
			//	}
			//	//printf("%f %f\n", tjoint[0].x, tjoint[0].y);

			//	DrawSkelToMat(&temp, tjoint, m_SkeletonInfo->InfoBody[i].JointPos, mode, m_SkeletonInfo->InfoBody[i].BodyID);
			//}

			//imshow("Calibration & Projection",temp);

		}

		for(int i = 0; i < _countof(ppBodies); i++){
			SafeRelease(ppBodies[i]);
		}
	}


	SafeRelease(pBodyFrame);
}

void KinectConnecter::DrawSkelToMat(cv::Mat *src, Point2d* JointPoints, Joint* pJoint, int mode, int t_id){
	Scalar t_Color = Scalar((t_id*37)%256, (t_id*113)%256, (t_id*71)%256); 

	// Torso
	DrawSkelBone(src, pJoint, JointPoints, JointType_Head, JointType_Neck, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_Neck, JointType_SpineShoulder, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_SpineShoulder, JointType_SpineMid, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_SpineMid, JointType_SpineBase, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_SpineShoulder, JointType_ShoulderRight, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_SpineShoulder, JointType_ShoulderLeft, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_SpineBase, JointType_HipRight, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_SpineBase, JointType_HipLeft, t_Color);

	// Right Arm
	DrawSkelBone(src, pJoint, JointPoints, JointType_ShoulderRight, JointType_ElbowRight, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_ElbowRight, JointType_WristRight, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_WristRight, JointType_HandRight, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_HandRight, JointType_HandTipRight, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_WristRight, JointType_ThumbRight, t_Color);

	// Left Arm
	DrawSkelBone(src, pJoint, JointPoints, JointType_ShoulderLeft, JointType_ElbowLeft, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_ElbowLeft, JointType_WristLeft, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_WristLeft, JointType_HandLeft, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_HandLeft, JointType_HandTipLeft, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_WristLeft, JointType_ThumbLeft, t_Color);

	// Right Leg
	DrawSkelBone(src, pJoint, JointPoints, JointType_HipRight, JointType_KneeRight, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_KneeRight, JointType_AnkleRight, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_AnkleRight, JointType_FootRight, t_Color);

	// Left Leg
	DrawSkelBone(src, pJoint, JointPoints, JointType_HipLeft, JointType_KneeLeft, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_KneeLeft, JointType_AnkleLeft, t_Color);
	DrawSkelBone(src, pJoint, JointPoints, JointType_AnkleLeft, JointType_FootLeft, t_Color);

	//Draw the Joints	- not implemented.
}

void KinectConnecter::DrawSkelBone(Mat *src, Joint* pJoints, Point2d* pJointPoints, JointType joint0, JointType joint1, Scalar t_Color){
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}

	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		line(*src, pJointPoints[joint0], pJointPoints[joint1], t_Color, c_TrackedBoneThickness);
	}
	else
	{
		line(*src, pJointPoints[joint0], pJointPoints[joint1], t_Color, c_InferredBoneThickness);
	}
}

//Not Implemented. - not work in SDK 2.0 preview version.. (나중에 사용해야할듯 handState가 항상 unknown.)
void KinectConnecter::DrawHand(Mat *src, HandState handState, Point2d& handposition){
	switch(handState){
	case HandState_Closed:
		break;
	case HandState_Open:
		break;
	case HandState_Lasso:
		break;
	}
}

Point2d KinectConnecter::BodyToScreen(const CameraSpacePoint& bodyPoint, int mode){
	Point2d Return_val;

	if(mode == 0){
		ColorSpacePoint colorPoint = {0};
		m_pCoordinateMapper->MapCameraPointToColorSpace(bodyPoint, &colorPoint);

		Return_val.x = static_cast<double>(colorPoint.X);
		Return_val.y = static_cast<double>(colorPoint.Y);
	}
	if(mode == 1){
		DepthSpacePoint depthPoint = {0};
		m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

		Return_val.x = static_cast<double>(depthPoint.X);
		Return_val.y = static_cast<double>(depthPoint.Y);
	}

	return Return_val;
}

void KinectConnecter::ProcessSkel(SkeletonInfo* m_SkeletonInfo, int nBodyCount, IBody** ppBodies, Mat *src, int mode){
	SkeletonCount = 0;
	HRESULT hr;

	m_SkeletonInfo->Count = SkeletonCount;

	if(m_pCoordinateMapper){
		for(int i = 0; i < nBodyCount; i++){
			IBody* pBody = ppBodies[i];

			if(pBody){
				BOOLEAN bTracked = false;
				hr = pBody->get_IsTracked(&bTracked);

				if(SUCCEEDED(hr) && bTracked){
					Joint joints[JointType_Count];
					Point2d jointPoints[JointType_Count];
					HandState leftHandState = HandState_Unknown;
					HandState rightHandState = HandState_Unknown;

					pBody->get_HandLeftState(&leftHandState);
					pBody->get_HandRightState(&rightHandState);

					hr = pBody->GetJoints(_countof(joints), joints);

					//Get Tracking Body ID;
					UINT64 t_ID;
					hr = pBody->get_TrackingId(&t_ID);
					m_SkeletonInfo->InfoBody[SkeletonCount].BodyID = t_ID;

					if(SUCCEEDED(hr)){

						for(int j = 0; j < _countof(joints); j++){
							m_SkeletonInfo->InfoBody[SkeletonCount].JointPos[j] = joints[j];
							jointPoints[j] = BodyToScreen(joints[j].Position, mode);
						}

						//printf("%f %f\n", jointPoints[0]);

						DrawSkelToMat(src, jointPoints, joints, mode, t_ID);

						DrawHand(src, leftHandState, jointPoints[JointType_HandLeft]);
						DrawHand(src, rightHandState, jointPoints[JointType_HandRight]); 

						SkeletonCount++;
					}
				}
			}
		}

		m_SkeletonInfo->Count = SkeletonCount;
	}
}

void KinectConnecter::GetKinectID(WCHAR *KinectID){
	memcpy(KinectID, UniqueID, _countof(UniqueID)*sizeof(WCHAR));
}

void KinectConnecter::SetKmat(Mat src){
	Kmat = src.clone();
}

void KinectConnecter::SetRmat(Mat src){
	//Rmat = src.clone();
	Rmat = (Mat_<double>(3,3) << 1., 0., 0., 0., 1.,0., 0., 0., 1.);
}

void KinectConnecter::SetTmat(Mat src){
	//Tmat = src.clone();
	Tmat = (Mat_<double>(3,1) << 0., 0., 0.);
}

void KinectConnecter::BasisCalibration(SkeletonInfo* m_SkeletonInfo)
{
	BodyInfo tempBody;
	double scale = 0.001;

	//좌표축 변환 메트릭스
	//Mat GLMat = (Mat_<double>(3,3) << -1., 0., 0., 0., 1., 0., 0., 0., -1.);

	Mat tempR = Rmat.clone();
	Mat tempT = Tmat.clone();

	tempR = /*GLMat**/tempR/**GLMat.t()*/;
	tempT = /*GLMat**/tempT;
	//추후 로테이션 추가

	//printf("%d\n", m_SkeletonInfo->Count);

	for(int i = 0; i < m_SkeletonInfo->Count; i++){
		tempBody = m_SkeletonInfo->InfoBody[i];

		//Calibration - 각 조인트에 대해서
		for(int j = 0; j < JointType_Count; j++){
			Joint tempJoint = tempBody.JointPos[j];

			Mat camData = (Mat_<double>(3,1) << tempJoint.Position.X, tempJoint.Position.Y, tempJoint.Position.Z);
			//printf("%f %f %f\n", camData.at<double>(0,0), camData.at<double>(1,0), camData.at<double>(2,0));

			camData = tempR.t()*(camData - tempT*scale);

			//printf("calib : %f %f %f\n", camData.at<double>(0,0), camData.at<double>(1,0), camData.at<double>(2,0));

			tempJoint.Position.X = camData.at<double>(0,0);
			tempJoint.Position.Y = camData.at<double>(1,0);
			tempJoint.Position.Z = camData.at<double>(2,0);

			m_SkeletonInfo->InfoBody[i].JointPos[j] = tempJoint;	
		}
	}
}

void KinectConnecter::GetDepthMappingImage(Mat *src){
	if( !m_pDepthFrameReader){
		return;
	}

	src->setTo(Scalar(0));

	IDepthFrame* pDepthFrame = NULL;

	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	if(SUCCEEDED(hr)){
		INT64 nTime = 0;
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance	= 0;
		USHORT nDepthMaxDistance			= 0;
		UINT nBufferSize = 0;
		UINT16 *pBuffer	= NULL;
		Mat tImage;

		hr = pDepthFrame->get_RelativeTime(&nTime);

		if(SUCCEEDED(hr)){
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		}

		if(SUCCEEDED(hr)){
			pFrameDescription->get_Width(&nWidth);
			pFrameDescription->get_Height(&nHeight);
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);

			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nDepthMaxDistance = USHRT_MAX;

			//// 각 어플리케이션에서 최장거리 제한이 필요한 경우 아래 코드 수정..
			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
			//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}

		if(SUCCEEDED(hr)){
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}

		if(SUCCEEDED(hr)){
			tImage.create(KINECT_DEPTH_HEIGHT, KINECT_DEPTH_WIDTH, CV_8UC4);			//Kinect Depth Image format BGRA 4 Channel image
			ConvertOpencvGrayImage(&tImage, pBuffer, nHeight, nWidth, nDepthMinReliableDistance, nDepthMaxDistance);
			//imshow("Test", tImage);
			//waitKey(0);
		}

		//mapping
		if(SUCCEEDED(hr)){
			//m_pCoordinateMapper->MapDepthFrameToColorSpace(nWidth*nHeight, pBuffer, KINECT_COLOR_WIDTH*KINECT_COLOR_HEIGHT, );
			hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, pBuffer, KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT, pDepthCoordinate);


			for(int h = 0; h < KINECT_COLOR_HEIGHT; h++){
				for(int w = 0; w < KINECT_COLOR_WIDTH; w++){
					DepthSpacePoint dsp = pDepthCoordinate[h*KINECT_COLOR_WIDTH + w];
					int depthX = (int)floor(dsp.X + 0.5);
					int depthY = (int)floor(dsp.Y + 0.5);
					//printf("%d %d\n", h, w);
					if (depthX >= 0 && depthX < KINECT_DEPTH_WIDTH && depthY >= 0 && depthY < KINECT_DEPTH_HEIGHT)
					{
						src->at<uchar>(h,w) = (uchar)tImage.at<cv::Vec3b>(depthY,depthX)[0];
					}
				}
			}
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pDepthFrame);
}

cv::Mat KinectConnecter::calculateMappedFrame(int mode)
{
	cv::Mat Mapping_img;
	HRESULT hr = E_FAIL;
	UINT16 *pBuffer	= NULL;

	// Depth coordinate mapping
	if (m_pCoordinateMapper != nullptr)
	{

		IDepthFrame* pDepthFrame = NULL;
		UINT nBufferSize = 0;

		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
		if(SUCCEEDED(hr)){
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
			// Color 2 Depth Mapping
			if (mode == 0)
			{
				Mapping_img = cv::Mat::zeros(KINECT_DEPTH_HEIGHT, KINECT_DEPTH_WIDTH, CV_8UC4);
				hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, (UINT16*)pBuffer, KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, pColorCoordinate);
				if (SUCCEEDED(hr))
				{
					Vec4b* p = Mapping_img.ptr< Vec4b >(0);
					for (int idx = 0; idx < KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT; idx++)				//Depth의 정보를 Color를 입힘
					{
						ColorSpacePoint csp = pColorCoordinate[idx];
						int colorX = (int)floor(csp.X + 0.5);
						int colorY = (int)floor(csp.Y + 0.5);
						if (colorX >= 0 && colorX < KINECT_COLOR_WIDTH && colorY >= 0 && colorY < KINECT_COLOR_HEIGHT)
						{
							p[idx] = colorRAWFrameMat.at< Vec4b >(colorY, colorX);
						}
					}
				}
			}
			// Depth 2 Color Mapping
			else if (mode == 1)
			{
				hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, (UINT16*)pBuffer, KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT, pDepthCoordinate);
				if (SUCCEEDED(hr))
				{
					Mapping_img = Mat::zeros(Size(KINECT_COLOR_WIDTH, KINECT_COLOR_HEIGHT), CV_8UC4);
					Vec4b* pMappedFrame = Mapping_img.ptr< Vec4b >(0);
#pragma omp parallel for

					for (int idx = 0; idx < KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT; idx++)			//Color의 점을 Depth 로 옮김
					{
						DepthSpacePoint dsp = pDepthCoordinate[idx];
						int depthX = (int)floor(dsp.X + 0.5);
						int depthY = (int)floor(dsp.Y + 0.5);
						if (depthX >= 0 && depthX < KINECT_DEPTH_WIDTH && depthY >= 0 && depthY < KINECT_DEPTH_HEIGHT)
						{
							ushort val = pBuffer[depthY * KINECT_DEPTH_WIDTH + depthX];
							if (val)
								pMappedFrame[idx] = /*((Vec4b*)colorRAWFrameMat)[idx]*/colorRAWFrameMat.at<Vec4b>(idx);
							else
								pMappedFrame[idx] = Vec4b(0, 0, 0, 0);
						}
					}
					/*flip(Mapping_img, Mapping_img, 1);*/
				}
			}
		}
		SafeRelease(pDepthFrame);
	}

	return Mapping_img;
}

//Color -> Depth
void KinectConnecter::GetMappingPos(std::vector<std::pair<cv::Point2f, cv::Point2f>> *src)
{
	HRESULT hr = E_FAIL;
	UINT16 *pBuffer	= NULL;

	// Depth coordinate mapping
	if (m_pCoordinateMapper != nullptr)
	{

		IDepthFrame* pDepthFrame = NULL;
		UINT nBufferSize = 0;

		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
		if(SUCCEEDED(hr)){
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
			// Color 2 Depth Mapping

			hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, (UINT16*)pBuffer, KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT, pDepthCoordinate);
			if (SUCCEEDED(hr))
			{
#pragma omp parallel for

				for (int idx = 0; idx < KINECT_COLOR_WIDTH * KINECT_COLOR_HEIGHT; idx++)			//Color의 점을 Depth 로 옮김
				{
					DepthSpacePoint dsp = pDepthCoordinate[idx];
					int depthX = (int)floor(dsp.X + 0.5);
					int depthY = (int)floor(dsp.Y + 0.5);
					if (depthX >= 0 && depthX < KINECT_DEPTH_WIDTH && depthY >= 0 && depthY < KINECT_DEPTH_HEIGHT)
					{
						ushort val = pBuffer[depthY * KINECT_DEPTH_WIDTH + depthX];
						if (val){
							cv::Point2f depthPos, colorPos;
							depthPos.x = depthX;
							depthPos.y = depthY;
							colorPos.x = idx % KINECT_COLOR_WIDTH;
							colorPos.y = idx / KINECT_COLOR_WIDTH;

							std::pair<cv::Point2f, cv::Point2f> tempPair;
							tempPair.first = depthPos;
							tempPair.second = colorPos;

							src->push_back(tempPair);
						}
					}
				}
				/*flip(Mapping_img, Mapping_img, 1);*/
			}
		}

		SafeRelease(pDepthFrame);
	}
}

void KinectConnecter::GetUVD_XYZ(std::vector<std::pair<cv::Point3f, cv::Point3f>> *src){
	HRESULT hr = E_FAIL;
	UINT16 *pBuffer	= NULL;
	CameraSpacePoint cameraSpacePoints[KINECT_COLOR_HEIGHT * KINECT_COLOR_WIDTH];

	static float maxX = -999999;
	static float minX = 999999;
	static float maxY = -999999;
	static float minY = 999999;

	// Depth coordinate mapping
	if (m_pCoordinateMapper != nullptr)
	{
		IDepthFrame* pDepthFrame = NULL;
		UINT nBufferSize = 0;

		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
		if(SUCCEEDED(hr)){
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
			// Color 2 Depth Mapping
			hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, (UINT16*)pBuffer, KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, pColorCoordinate);
			if (SUCCEEDED(hr))
			{
				hr = m_pCoordinateMapper->MapColorFrameToCameraSpace(KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, (UINT16*)pBuffer, KINECT_COLOR_HEIGHT * KINECT_COLOR_WIDTH, cameraSpacePoints);
				CameraIntrinsics tempIntrinsic;
				m_pCoordinateMapper->GetDepthCameraIntrinsics(&tempIntrinsic);

				for (int idx = 0; idx < KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT; idx++)				//Depth의 정보를 Color를 입힘
				{
					ColorSpacePoint csp = pColorCoordinate[idx];
					int colorX = (int)floor(csp.X + 0.5);
					int colorY = (int)floor(csp.Y + 0.5);
					if (colorX >= 0 && colorX < KINECT_COLOR_WIDTH && colorY >= 0 && colorY < KINECT_COLOR_HEIGHT)
					{
						int colorIdx = colorX + colorY * KINECT_COLOR_WIDTH;
						cv::Point3f XYZ = cv::Point3f(cameraSpacePoints[colorIdx].X, cameraSpacePoints[colorIdx].Y, cameraSpacePoints[colorIdx].Z);
						cv::Point3f UVD = cv::Point3f(colorX, colorY, (float)pBuffer[idx]);

						if(cvIsInf(XYZ.x) || cvIsInf(XYZ.y) || cvIsInf(XYZ.z)){
							//printf("XYZ ERROR!\n");
							continue;
						}

						std::pair<cv::Point3f, cv::Point3f> tempPair;
						tempPair.first = UVD;
						tempPair.second = XYZ;

						if(UVD.x > maxX) maxX = UVD.x;
						if(UVD.x <= minX) minX = UVD.x;
						if(UVD.y > maxY) maxY = UVD.y;
						if(UVD.y <= minY) minY = UVD.y;

						/*int src_x = colorX;
						int src_y = colorY;
						float src_depth = pBuffer[idx] / 1000.f;
						float ux = (src_x - tempIntrinsic.PrincipalPointX) / tempIntrinsic.FocalLengthX;
						float uy = (tempIntrinsic.PrincipalPointY-src_y) / tempIntrinsic.FocalLengthY;

						float r = sqrt(ux*ux+uy*uy);
						float cof_xy = 1.0f + tempIntrinsic.RadialDistortionSecondOrder*pow(r, 2) + tempIntrinsic.RadialDistortionFourthOrder*pow(r, 4) + tempIntrinsic.RadialDistortionSixthOrder*pow(r, 6);

						float dest_x = ux*cof_xy*src_depth;
						float dest_y = uy*cof_xy*src_depth;*/
						if(UVD.x < KINECT_COLOR_WIDTH/4 || UVD.x > KINECT_COLOR_WIDTH*3/4)
							continue;
						if(UVD.y < KINECT_COLOR_HEIGHT/4 || UVD.y > KINECT_COLOR_HEIGHT*3/4)
							continue;

						src->push_back(tempPair);
					}
				}
			}
		}

		SafeRelease(pDepthFrame);
	}
}

void KinectConnecter::GetRGBUVDMat(cv::Mat *uvdMat, cv::Mat *xyzMat){
	uvdMat->create(KINECT_DEPTH_HEIGHT, KINECT_DEPTH_WIDTH, CV_64FC1);
	xyzMat->create(KINECT_DEPTH_HEIGHT, KINECT_DEPTH_WIDTH, CV_64FC3);

	for(int i = 0; i < KINECT_DEPTH_HEIGHT * KINECT_DEPTH_WIDTH; i++){
		uvdMat->at<double>(i) = 0.0f;
		xyzMat->at<cv::Vec3d>(i) = cv::Vec3d(0.0f, 0.0f, 0.0f);
	}

	HRESULT hr = E_FAIL;
	UINT16 *pBuffer	= NULL;
	CameraSpacePoint* cameraSpacePoints = new CameraSpacePoint[KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT];

	// Depth coordinate mapping
	if (m_pCoordinateMapper != nullptr)
	{
		IDepthFrame* pDepthFrame = NULL;
		UINT nBufferSize = 0;

		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
		if(SUCCEEDED(hr)){
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
			// Color 2 Depth Mapping
			hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, (UINT16*)pBuffer, KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, pColorCoordinate);
			if (SUCCEEDED(hr))
			{
				hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, (UINT16*)pBuffer, KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, cameraSpacePoints);

				for (int idx = 0; idx < KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT; idx++)				//Depth의 정보를 Color를 입힘
				{
					ColorSpacePoint csp = pColorCoordinate[idx];
					int colorX = (int)floor(csp.X + 0.5);
					int colorY = (int)floor(csp.Y + 0.5);

					if (colorX >= 0 && colorX < KINECT_COLOR_WIDTH && colorY >= 0 && colorY < KINECT_COLOR_HEIGHT)
					{
						int colorIdx = colorX + colorY * KINECT_COLOR_WIDTH;
						uvdMat->at<double>(idx) = (double)pBuffer[idx];
						xyzMat->at<cv::Vec3d>(idx) = cv::Vec3d(cameraSpacePoints[idx].X, cameraSpacePoints[idx].Y, cameraSpacePoints[idx].Z);
					}
				}
			}
		}

		SafeRelease(pDepthFrame);
	}
}

void KinectConnecter::GetDepthUVD_XYZ(std::vector<std::pair<cv::Point3f, cv::Point3f>> *src){
	HRESULT hr = E_FAIL;
	UINT16 *pBuffer	= NULL;
	CameraSpacePoint* cameraSpacePoints = new CameraSpacePoint[KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT];
	float totalError = 0;
	float TotalMax = -99999;
	float TotalMin = 99999;

	static float maxX = -999999;
	static float minX = 999999;
	static float maxY = -999999;
	static float minY = 999999;

	// Depth coordinate mapping
	if (m_pCoordinateMapper != nullptr)
	{
		IDepthFrame* pDepthFrame = NULL;
		UINT nBufferSize = 0;

		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
		if(SUCCEEDED(hr)){
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
			// Color 2 Depth Mapping
			hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, (UINT16*)pBuffer, KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, cameraSpacePoints);
			CameraIntrinsics tempIntrinsic;
			m_pCoordinateMapper->GetDepthCameraIntrinsics(&tempIntrinsic);
			if (SUCCEEDED(hr))
			{

				for (int idx = 0; idx < KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT; idx++)				//Depth의 정보를 Color를 입힘
				{
					int src_x = idx % KINECT_DEPTH_WIDTH;
					int src_y = idx / KINECT_DEPTH_WIDTH;
					cv::Point3f XYZ = cv::Point3f(cameraSpacePoints[idx].X, cameraSpacePoints[idx].Y, cameraSpacePoints[idx].Z);
					cv::Point3f UVD = cv::Point3f(idx % KINECT_DEPTH_WIDTH, idx / KINECT_DEPTH_WIDTH, (float)pBuffer[idx]);

					UVD.x = KINECT_DEPTH_WIDTH - UVD.x;

					DepthSpacePoint depthpoint;
					CameraSpacePoint camerapoint;
					depthpoint.X = src_x;
					depthpoint.Y = src_y;
					hr = m_pCoordinateMapper->MapDepthPointsToCameraSpace(1, &depthpoint, 1, &pBuffer[idx], 1, &camerapoint);

					if(cvIsInf(XYZ.x) || cvIsInf(XYZ.y) || cvIsInf(XYZ.z)){
						//printf("XYZ ERROR!\n");
						continue;
					}

					std::pair<cv::Point3f, cv::Point3f> tempPair;
					tempPair.first = UVD;
					tempPair.second = XYZ;

					DepthSpacePoint input;
					CameraSpacePoint output;

					input.X = UVD.x;
					input.Y = UVD.y;

					if(UVD.x < KINECT_DEPTH_WIDTH / 4 || UVD.x > KINECT_DEPTH_WIDTH * 3 / 4)
						continue;
					if(UVD.y < KINECT_DEPTH_HEIGHT / 4 || UVD.y > KINECT_DEPTH_HEIGHT * 3 / 4)
						continue;

					float src_depth = pBuffer[idx] / 1000.f;
					float ux = (src_x - tempIntrinsic.PrincipalPointX) / tempIntrinsic.FocalLengthX;
					float uy = (tempIntrinsic.PrincipalPointY-src_y) / tempIntrinsic.FocalLengthY;

					float r = sqrt(ux*ux+uy*uy);
					float cof_xy = 1.0f + tempIntrinsic.RadialDistortionSecondOrder*pow(r, 2) + tempIntrinsic.RadialDistortionFourthOrder*pow(r, 4) + tempIntrinsic.RadialDistortionSixthOrder*pow(r, 6);

					float dest_x = ux*cof_xy*src_depth;
					float dest_y = uy*cof_xy*src_depth;

					///////////////////////////////////////////////////////////////////////////////////////////////////

					float x = (src_x - tempIntrinsic.PrincipalPointX) / tempIntrinsic.FocalLengthX;
					float y = (tempIntrinsic.PrincipalPointY - src_y) / tempIntrinsic.FocalLengthY;

					// undistort
					float r2 = x * x + y * y;
					float d = 1.0f - tempIntrinsic.RadialDistortionSecondOrder * r2 - tempIntrinsic.RadialDistortionFourthOrder * r2 * r2 - tempIntrinsic.RadialDistortionSixthOrder * r2 * r2 * r2;

					// Camera space reports coordinates in meters vs. millimeters like depth imag
					float depthF = pBuffer[idx] * 0.001f;
					x = x * d * depthF;
					y = y * d * depthF;

					float linear_x = (src_x - tempIntrinsic.PrincipalPointX) / tempIntrinsic.FocalLengthX * depthF;
					float linear_y = (tempIntrinsic.PrincipalPointY - src_y) / tempIntrinsic.FocalLengthY * depthF;
					float temp = sqrt(pow(linear_x-XYZ.x, 2) + pow(linear_y-XYZ.y, 2));
					////////////////////////////////////////////////////////////////////////////////////////////////////
					if(TotalMax < temp)	TotalMax = temp;
					if(TotalMin > temp) TotalMin = temp;
					totalError += temp;

					src->push_back(tempPair);

				}
			}
		}
		totalError /= src->size();

		SafeRelease(pDepthFrame);
	}
}

void KinectConnecter::GetRGBDnDepthnXYZ(cv::Mat *rgbd, cv::Mat *depthmap, cv::Mat *xyzmap){
	cv::Mat maprgbd, mapdepth, mapxyz;
	HRESULT hr = E_FAIL;
	UINT16 *pBuffer	= NULL;
	CameraSpacePoint *cameraSpacePoints = new CameraSpacePoint[KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT];

	// Depth coordinate mapping
	if (m_pCoordinateMapper != nullptr)
	{
		IDepthFrame* pDepthFrame = NULL;
		UINT nBufferSize = 0;

		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
		if(SUCCEEDED(hr)){
			//Image allocation
			maprgbd = cv::Mat::zeros(KINECT_DEPTH_HEIGHT, KINECT_DEPTH_WIDTH, CV_8UC4);
			mapdepth = cv::Mat::zeros(KINECT_DEPTH_HEIGHT, KINECT_DEPTH_WIDTH, CV_32FC1);
			mapxyz = cv::Mat::zeros(KINECT_DEPTH_HEIGHT, KINECT_DEPTH_WIDTH, CV_32FC3);

			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
			// Color 2 Depth Mapping
			hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, (UINT16*)pBuffer, KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, pColorCoordinate);
			if (SUCCEEDED(hr))
			{
				hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, (UINT16*)pBuffer, KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT, cameraSpacePoints);

				Vec4b* rgbdp = maprgbd.ptr< Vec4b >(0);
				float* depthp = mapdepth.ptr< float >(0);
				//Vec3f* xyzp = mapxyz.ptr < Vec3f > (0);
				for (int idx = 0; idx < KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT; idx++)				//Depth의 정보를 Color를 입힘
				{
					//rgbd image set
					ColorSpacePoint csp = pColorCoordinate[idx];
					int colorX = (int)floor(csp.X + 0.5);
					int colorY = (int)floor(csp.Y + 0.5);
					if (colorX >= 0 && colorX < KINECT_COLOR_WIDTH && colorY >= 0 && colorY < KINECT_COLOR_HEIGHT)
						rgbdp[idx] = colorRAWFrameMat.at< Vec4b >(colorY, colorX);

					//Depth image set
					depthp[idx] = (float)pBuffer[idx];

					//xyz matrix set
					//xyzp[idx] = Vec3f(cameraSpacePoints[idx].X, cameraSpacePoints[idx].Y, cameraSpacePoints[idx].Z);
					mapxyz.at<Vec3f>(idx) = Vec3f(cameraSpacePoints[idx].X, cameraSpacePoints[idx].Y, cameraSpacePoints[idx].Z);
				}
			}
		}
		SafeRelease(pDepthFrame);
	}

	*rgbd = maprgbd.clone();
	*depthmap = mapdepth.clone();
	*xyzmap = mapxyz.clone();

	delete[] cameraSpacePoints;
}