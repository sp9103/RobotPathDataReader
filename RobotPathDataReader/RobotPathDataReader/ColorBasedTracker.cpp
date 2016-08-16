#include "ColorBasedTracker.h"




ColorBasedTracker::ColorBasedTracker(void)
{
}


ColorBasedTracker::~ColorBasedTracker(void)
{
}

void ColorBasedTracker::InsertBackGround(cv::Mat Color, cv::Mat Depth){
	imgWidth = Color.cols;
	imgHeight = Color.rows;

	if(Color.channels() == 3)		ColorBackGround = Color.clone();
	else						cv::cvtColor(Color, ColorBackGround, CV_BGRA2BGR);
	DepthBackGround = Depth.clone();
}

cv::Mat ColorBasedTracker::subBackground(cv::Mat srcColor, cv::Mat srcDepth){
	const float threshold = 10;
	cv::Mat tMat(srcColor.rows, srcColor.cols, CV_8UC1);
	//DepthBase sub Image
	for (int i = 0; i < srcColor.rows * srcColor.cols; i++){
		float bVal = DepthBackGround.at<float>(i);
		float oVal = srcDepth.at<float>(i);

		if (bVal == 0 || oVal == 0){
			tMat.at<uchar>(i) = (uchar)0;
			continue;
		}
		if (abs(oVal - bVal) > threshold){
			tMat.at<uchar>(i) = (uchar)255;
		}
		else{
			tMat.at<uchar>(i) = (uchar)0;
		}
	}

	//ColorBase sub image
	for (int i = 0; i < srcColor.rows * srcColor.cols; i++){
		uchar binVal = tMat.at<uchar>(i);
		if (binVal > 0){
			cv::Vec3b bVal = ColorBackGround.at<cv::Vec3b>(i);
			cv::Vec3b oVal = srcColor.at<cv::Vec3b>(i);
			cv::Vec3b subVal;
			const int cThreshold = 40;
			for (int j = 0; j < 3; j++)		subVal[j] = abs((int)bVal[j] - (int)oVal[j]);
			if (subVal[0] < cThreshold && subVal[1] < cThreshold && subVal[2] < cThreshold)
				tMat.at<uchar>(i) = (uchar)0;
		}
	}

	/*cv::erode(tMat, tMat, cv::Mat(), cv::Point(-1, -1), 4);
	cv::dilate(tMat, tMat, cv::Mat(), cv::Point(-1, -1), 4);*/

	return tMat.clone();
}

cv::Mat ColorBasedTracker::calcImage(cv::Mat src, cv::Mat depth){
	if(src.channels() == 4)	cv::cvtColor(src, src, CV_BGRA2BGR);

	double duration;                                                         //변수 설정
	duration = static_cast<double>(cv::getTickCount());       //초기 시작 시간 설정
	cv::Mat output;
	cv::Mat backSub = subBackground(src, depth);
	cv::Mat YellowMap = DetectColorMap(src, backSub);
	cv::Mat MapSub = DeleteSub(YellowMap, backSub);
	_blobLabeling.Labeling(20, MapSub);

	//Calculate Object Image
	std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> BOI;		//blob of interest
	FindNMaxBlob(2, &BOI);
	cv::Mat src2 = drawBlobMap(src, BOI);
	for(int i = 0; i < BOI.size(); i++)
		cv::rectangle(src2, BOI.at(i).second, cv::Scalar(0,0,255));
	cv::Rect HandBox = FindHandBlob(BOI);
	if(HandBox.width > 0)
		cv::rectangle(src2, HandBox, cv::Scalar(0,255,0), 2);
	cv::imshow("src2", src2);

	//compose with background
	cv::Mat src_compose = src.clone();
	for(int i = 0; i < _blobLabeling.getBlobCount(); i++){
		std::vector<cv::Point2i> blobVec = _blobLabeling.getBlob(i);
		for(int j = 0; j < blobVec.size(); j++){
			cv::Point2i blobPoint = blobVec.at(j);
			src_compose.at<cv::Vec3b>(blobPoint.x, blobPoint.y) = ColorBackGround.at<cv::Vec3b>(blobPoint.x, blobPoint.y);
		}
	}
	duration = static_cast<double>(cv::getTickCount())-duration;		//함수 내용 지난담에 시간과 초기시간 빼기
	duration /= cv::getTickFrequency();									//ms 단위 경과 시간
	printf("%f ms\n",duration);

	//cv::imshow("backSub", backSub);
	//cv::imshow("YellowMap", YellowMap);
	//cv::imshow("MapSub", MapSub);
	//cv::imshow("src", src);
	//cv::imshow("back_compoes", src_compose);
	//cv::waitKey(10);

	//backSub.release();

	//이미지 생성부
	const int extra = 4;
	HandBox.x -= extra;
	HandBox.y -= extra;
	HandBox.width += extra;
	HandBox.height += extra;
	int x_prime = HandBox.x + HandBox.width;
	int y_prime = HandBox.y + HandBox.height;

	if(HandBox.x <= 0 || HandBox.y <= 0 || src.cols <= x_prime || src.rows <= y_prime)	output.create(0, 0, CV_8UC1);
	else{
		output.create(src.rows, src.cols, src.type());
		for(int h = 0; h < src.rows; h++)
			for(int w = 0; w < src.cols; w++){
				if(HandBox.x <= w && w <= x_prime && HandBox.y <= h && h <= y_prime)
					output.at<cv::Vec3b>(h,w) = src.at<cv::Vec3b>(h,w);
				else
					output.at<cv::Vec3b>(h,w) = ColorBackGround.at<cv::Vec3b>(h,w);
			}
	}

	return output;
}

cv::Mat ColorBasedTracker::DetectColorMap(cv::Mat rgb, cv::Mat subMap){
	cv::Mat output(rgb.rows, rgb.cols, CV_8UC1);
	cv::Mat HSV_Input(rgb.rows, rgb.cols, rgb.type());
	cv::cvtColor(rgb, HSV_Input, CV_BGR2HSV);

	for(int h = 0; h < rgb.rows; h++){
		for(int w = 0; w < rgb.cols; w++){
			uchar R = rgb.at<cv::Vec3b>(h,w)[2];
			uchar B = rgb.at<cv::Vec3b>(h,w)[0];
			uchar G = rgb.at<cv::Vec3b>(h,w)[1];

			uchar H = HSV_Input.at<cv::Vec3b>(h,w)[0];
			uchar S = HSV_Input.at<cv::Vec3b>(h,w)[1];
			uchar V = HSV_Input.at<cv::Vec3b>(h,w)[2];

			uchar subMapPixel = subMap.at<uchar>(h,w);

			if(subMapPixel > 0){
				if(30 > H || H > 120/* && V > 50*/){
					output.at<uchar>(h,w) = (uchar)255;
				}else
					output.at<uchar>(h,w) = (uchar)0;

			}else{
				output.at<uchar>(h,w) = 0;
			}
		}
	}

	cv::erode(output, output, cv::Mat(), cv::Point(-1, -1), 2);
	cv::dilate(output, output, cv::Mat(), cv::Point(-1, -1), 4);
	//cv::erode(output, output, cv::Mat(), cv::Point(-1, -1), 3);

	return output;
}

cv::Mat ColorBasedTracker::DeleteSub(cv::Mat map, cv::Mat src){
	cv::Mat output(map.rows, map.cols, src.type());

	for(int h = 0; h < map.rows; h++){
		for(int w = 0; w < map.cols; w++){
			uchar srcPixel = src.at<uchar>(h,w);
			uchar mapPixel = map.at<uchar>(h,w);

			if(srcPixel == 0 || mapPixel > 0)
				output.at<uchar>(h,w) = (uchar)0;
			else
				output.at<uchar>(h,w) = (uchar)255;
		}
	}

	return output;
}

bool sort_vector_pair(std::pair<int, int> i, std::pair<int, int> j) {	return (i.second > j.second);	}
void ColorBasedTracker::FindNMaxBlob(int N, std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> *dst){
	std::vector<std::pair<int, int>>	tempVec;		//idx, count
	int blobCount = _blobLabeling.getBlobCount();

	for(int i = 0; i < blobCount; i++){
		int NumBlobPixel = _blobLabeling.getBlob(i).size();
		tempVec.push_back(std::make_pair(i, NumBlobPixel));
	}
	if(blobCount > 2)		std::sort(tempVec.begin(), tempVec.end(), sort_vector_pair);

	for(int i = 0; i < MIN(N, tempVec.size()); i++){
		cv::Rect BoundingBox;
		cv::Point2i MinPoint = cv::Point2i(INT_MAX, INT_MAX);
		cv::Point2i MaxPoint = cv::Point2i(-1, -1);
		std::vector<cv::Point2i> blobVec = _blobLabeling.getBlob(tempVec.at(i).first);
		for(int j = 0; j < blobVec.size(); j++){
			cv::Point2i pixelPoint = blobVec.at(j);
			if(MinPoint.x > pixelPoint.y)		MinPoint.x = pixelPoint.y;
			if(MinPoint.y > pixelPoint.x)		MinPoint.y = pixelPoint.x;
			if(MaxPoint.x < pixelPoint.y)		MaxPoint.x = pixelPoint.y;
			if(MaxPoint.y < pixelPoint.x)		MaxPoint.y = pixelPoint.x;
		}
		BoundingBox = cv::Rect(MinPoint, MaxPoint);
		BoundingBox.width += 1;
		BoundingBox.height += 1;
		dst->push_back(std::make_pair(blobVec, BoundingBox));
	}
}

cv::Mat ColorBasedTracker::drawBlobMap(cv::Mat src, std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> blobInfo){
	cv::Mat retMap = src.clone();

	for(int i = 0; i < blobInfo.size(); i++){
		std::vector<cv::Point2i> blobPixelVec = blobInfo.at(i).first;

		for(int j = 0; j < blobPixelVec.size(); j++){
			cv::Point2i blobPoint = blobPixelVec.at(j);
			retMap.at<cv::Vec3b>(blobPoint.x, blobPoint.y) = cv::Vec3b(((i+1) * 137) % 255, ((i+1) * 79) % 255, ((i+1) * 101) % 255);
		}
	}

	return retMap;
}

cv::Rect ColorBasedTracker::FindHandBlob(std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> src){
	cv::Rect outRect = cv::Rect(-1,-1,-1,-1);
	std::vector<int> nonBoundBlobIdx;
	int maxIdx, minY = INT_MAX;

	if(src.size() == 0)		return outRect;
	if(src.size() == 1)		outRect = src.at(0).second;
	else{
		//경계에 닿는 블록수 체크
		int boundaryCount = 0;
		for(int i = 0; i < src.size(); i++){
			cv::Rect blobRect = src.at(i).second;

			if(blobRect.y <= 0)		return blobRect;
			if(blobRect.y <= 0 || blobRect.x <= 0 || (blobRect.x + blobRect.width) >= imgWidth-1 || (blobRect.y + blobRect.height) >= imgHeight-1)	boundaryCount++;
			else
				nonBoundBlobIdx.push_back(i);
			if(minY > blobRect.y){
				minY = blobRect.y;
				maxIdx = i;
			}
		}

		if(nonBoundBlobIdx.size() == 1)		return src.at(nonBoundBlobIdx.at(0)).second;
		else
			return src.at(maxIdx).second;
	}

	return outRect;
}