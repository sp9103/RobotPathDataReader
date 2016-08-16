#include "BlobLabeling.h"


BlobLabeling::BlobLabeling(void)
{
	direction.push_back(cv::Point2i(-1,-1));
	direction.push_back(cv::Point2i(-1,0));
	direction.push_back(cv::Point2i(-1,1));
	direction.push_back(cv::Point2i(0,1));
	direction.push_back(cv::Point2i(1,1));
	direction.push_back(cv::Point2i(1,0));
	direction.push_back(cv::Point2i(1,-1));
	direction.push_back(cv::Point2i(0,-1));
}


BlobLabeling::~BlobLabeling(void)
{
}


void BlobLabeling::Labeling(int pixelCount, cv::Mat src){
	_group.clear();
	if(src.channels() != 1){
		printf("input image must 1 channels\n");
		return;
	}

	//find chunk
	std::map<int, std::vector<cv::Point2i>> TempGroup;
	cv::Mat chunkMap = findChunk(src, &TempGroup);
	//visMap(chunkMap, "before Merge");

	//Merge
	ChunkMerge(chunkMap, TempGroup);
	//visMap(chunkMap, "after Merge");

	//dump
	std::vector<int> eraseList;
	for(std::map<int, std::vector<cv::Point2i>>::iterator it = _group.begin(); it != _group.end(); it++){
		int pixelNum = it->second.size();
		if(pixelNum < pixelCount)
			eraseList.push_back(it->first);
	}
	for(int i = 0; i < eraseList.size(); i++)	_group.erase(eraseList.at(i));

}

cv::Mat BlobLabeling::findChunk(cv::Mat src, std::map<int, std::vector<cv::Point2i>> *tGroup){
	int count = 0;
	cv::Mat out = cv::Mat::zeros(src.rows, src.cols, CV_32FC1);

	for(int h = 0; h < src.rows; h++){
		for(int w = 0; w < src.cols; w++){
			uchar val = src.at<uchar>(h,w);

			if(val != 0){
				float tempVal = findMinNeighborID(h, w, out);

				if(tempVal == -1){
					tempVal = ++count;
					std::vector<cv::Point2i> tempVec;
					tempVec.push_back(cv::Point2i(h,w));
					tGroup->insert(std::pair<int, std::vector<cv::Point2i>>(count, tempVec));
				}else{
					tGroup->find((int)tempVal)->second.push_back(cv::Point2i(h,w));
				}

				out.at<float>(h,w) = tempVal;
			}
		}
	}

	return out;
}

void BlobLabeling::visMap(cv::Mat src, const char *windowName){
	cv::Mat vis(src.rows, src.cols, CV_8UC3);

	for(int i = 0; i < src.rows * src.cols; i++){
		float val = src.at<float>(i);
		cv::Vec3b tColor = cv::Vec3b(((int)val * 137) % 255, ((int)val * 79) % 255, ((int)val * 101) % 255);
		vis.at<cv::Vec3b>(i) = tColor;
	}

	cv::imshow(windowName, vis);
	cv::waitKey(0);
}

void BlobLabeling::ChunkMerge(cv::Mat src, std::map<int, std::vector<cv::Point2i>> tGroup){
	if(tGroup.size() <= 0) return;
	
	int count = 0;
	_group.insert(std::pair<int, std::vector<cv::Point2i>>(1, tGroup.find(1)->second));
	for(int i = 2; i <= tGroup.size(); i++){
		std::map<int, std::vector<cv::Point2i>>::iterator it = tGroup.find(i);

		int minNeighbor = -1;
		for(int j = 0; j < it->second.size(); j++){
			cv::Point2i target = it->second.at(j);

			int tempVal = findMinNeighborID(target.x, target.y, src);
			if(minNeighbor == -1)	minNeighbor = tempVal;
			else if(minNeighbor > tempVal && tempVal < it->first)	minNeighbor = tempVal;
		}

		if(minNeighbor == -1 || minNeighbor == it->first)	_group.insert(std::pair<int, std::vector<cv::Point2i>>(it->first, tGroup.find(it->first)->second));
		else{
			for(int j = 0; j < it->second.size(); j++){
				cv::Point2i target = it->second.at(j);
				src.at<float>(target.x, target.y) = minNeighbor;
				_group.find(minNeighbor)->second.push_back(target);
			}
		}
	}
}

int BlobLabeling::findMinNeighborID(int h, int w, cv::Mat srcMap){
	float tempVal = -1;
	for(int i = 0; i < direction.size(); i++){
		int newH = direction.at(i).y + h;
		int newW = direction.at(i).x + w;

		if(newH >= srcMap.rows || newH < 0)	continue;
		if(newW >= srcMap.cols || newW < 0)	continue;

		float neighbor = srcMap.at<float>(newH, newW);
		if(neighbor == 0)	continue;
		if(tempVal == -1)	tempVal = neighbor;
		else if(tempVal > neighbor)
			tempVal = neighbor;
	}

	return tempVal;
}

int BlobLabeling::getBlobCount(){
	return _group.size();
}

std::vector<cv::Point2i> BlobLabeling::getBlob(int i){
	std::map<int, std::vector<cv::Point2i>>::iterator it = _group.begin();
	for(int j = 0; j < i; j++)	it++;

	return it->second;
}