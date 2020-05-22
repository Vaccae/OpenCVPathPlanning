#include<opencv2/opencv.hpp>
#include<iostream>
#include <direct.h>
#include "PathDetector.h"

using namespace cv;
using namespace std;

Mat src;
Mat srccopy; //用于拷贝出的源图像
string showsrc = "源图";
//起始坐标
Point startpoint(-1, -1);
//结束坐标
Point endpoint(-1, -1);

void onMouse(int event, int x, int y, int flags, void* ustc); //鼠标回调函数

int main(int argc, char** argv) {
	//获取程序目录
	char filepath[256];
	_getcwd(filepath, sizeof(filepath));

	//定义模型文件
	string imgsrc = (string)filepath + "/pointtest/2.jpg";

	//读取图片
	src = imread(imgsrc);
	//窗体
	namedWindow(showsrc);
	//设置鼠标响影事件
	setMouseCallback(showsrc, onMouse);

	imshow(showsrc, src);

	waitKey(0);
	return 0;
}

void onMouse(int event, int x, int y, int flags, void* ustc)
{
	//鼠标左键按下
	if (event == EVENT_LBUTTONUP)
	{
		//超始坐标未指定时
		if (startpoint.x == -1 && startpoint.y == -1) {
			startpoint = Point(x, y);
			srccopy = src.clone();
			circle(srccopy, startpoint, 2, Scalar(0, 0, 255));
			imshow(showsrc, srccopy);
		}
		else if (endpoint.x == -1 && endpoint.y == -1) {
			//显示起点和终点位置画线
			endpoint = Point(x, y);
			circle(srccopy, endpoint, 2, Scalar(0, 0, 255));
			imshow(showsrc, srccopy);

			cout << "startpoint:" << startpoint.x << "," << startpoint.y << endl;
			cout << "endpoint:" << endpoint.x << "," << endpoint.y << endl;
			

			srccopy = src.clone();

			//调用方法进行检测
			PathDetector detector = PathDetector();
			vector<Point> rescontours;
			vector<Point> inputpoints;
			inputpoints.push_back(startpoint);
			inputpoints.push_back(endpoint);

			//用A*算法计算路径并源图上画检测的结果
			double Time = (double)getTickCount();
			rescontours = detector.getPath(src, inputpoints);
			Time = ((double)getTickCount() - Time) / getTickFrequency();
			cout << "astar time:" << Time << "ss" << endl;

			if (rescontours.size() > 0) {
				//所有的返回点显示出来
				for (int j = 0; j < rescontours.size() - 1; j++) {
					int k = j + 1;
					line(srccopy, rescontours[j], rescontours[k], Scalar(0, 0, 255), 2);
					imshow(showsrc, srccopy);
				}
			}
			else {
				cout << "无可到达路径" << endl;
			}


			//用JPS算法计算路径并在源图上画检测的结果
			double t = (double)getTickCount();
			rescontours = detector.getPath(src, inputpoints,1);
			t = ((double)getTickCount() - t) / getTickFrequency();
			cout << "jps time:" << t << "ss" << endl;

			if (rescontours.size() > 0) {
				//所有的返回点显示出来
				for (int j = 0; j < rescontours.size() - 1; j++) {
					int k = j + 1;
					line(srccopy, rescontours[j], rescontours[k], Scalar(255, 0, 0), 2);
					imshow(showsrc, srccopy);
				}
			}
			else {
				cout << "无可到达路径" << endl;
			}

			
		}
	}
	//当鼠标右键按下时取消选点
	else if (event == EVENT_RBUTTONUP)
	{
		srccopy = src.clone(); //复制源图像
		startpoint = Point(-1, -1);
		endpoint = Point(-1, -1);
		imshow(showsrc, srccopy);
	}
}
