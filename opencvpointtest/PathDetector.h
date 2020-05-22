#pragma once

#include<opencv2/opencv.hpp>
#include<iostream>
#include"AStarCalc.h"
#include"JPSCalc.h"

using namespace cv;
using namespace std;

struct objrange
{
	int objbegin;
	int objend;
	objrange():objbegin(0),objend(0)
	{
	}
};

class PathDetector
{
public:
	//�����������귵�ص�����
	vector<Point> getPath(Mat src, vector<Point> inputPoint,int type=0);
};

