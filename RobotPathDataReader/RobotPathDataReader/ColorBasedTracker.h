#pragma once
#include <opencv2\opencv.hpp>
#include "BlobLabeling.h"

#define COLOR_THRESHOLD		10
#define DEPTH_THRESHOLD		10
#define PEDDING				4

class ColorBasedTracker
{
public:
	ColorBasedTracker(void);
	~ColorBasedTracker(void);

	void InsertBackGround(cv::Mat Color, cv::Mat Depth);

	cv::Mat calcImage(cv::Mat src, cv::Mat depth);

private:
	int imgWidth;
	int imgHeight;

	BlobLabeling _blobLabeling;

	cv::Mat ColorBackGround;
	cv::Mat DepthBackGround;
	
	cv::Mat subBackground(cv::Mat srcColor, cv::Mat srcDepth);
	cv::Mat DetectColorMap(cv::Mat rgb, cv::Mat subMap);
	cv::Mat DeleteSub(cv::Mat map, cv::Mat src);
	void FindNMaxBlob(int N, std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> *dst);
	cv::Rect FindHandBlob(std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> src);
	
	cv::Mat drawBlobMap(cv::Mat src, std::vector<std::pair<std::vector<cv::Point2i>, cv::Rect>> blobInfo);
};

