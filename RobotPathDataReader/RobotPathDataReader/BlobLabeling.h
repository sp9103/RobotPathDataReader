#pragma once
#include <opencv2\opencv.hpp>

class BlobLabeling
{
public:
	BlobLabeling(void);
	~BlobLabeling(void);

	void Labeling(int pixelCount, cv::Mat src);
	int getBlobCount();
	std::vector<cv::Point2i> getBlob(int i);

private:
	std::vector<cv::Point2i>	direction;
	std::map<int, std::vector<cv::Point2i>> _group;
	
	cv::Mat findChunk(cv::Mat src, std::map<int, std::vector<cv::Point2i>> *tGroup); 
	void visMap(cv::Mat src, const char *windowName);
	void ChunkMerge(cv::Mat src, std::map<int, std::vector<cv::Point2i>> tGroup);
	int findMinNeighborID(int h, int w, cv::Mat srcMap);
};

