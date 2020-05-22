#include<opencv2/opencv.hpp>
#include<iostream>
#include <direct.h>
#include "PathDetector.h"

using namespace cv;
using namespace std;

Mat src;
Mat srccopy; //���ڿ�������Դͼ��
string showsrc = "Դͼ";
//��ʼ����
Point startpoint(-1, -1);
//��������
Point endpoint(-1, -1);

void onMouse(int event, int x, int y, int flags, void* ustc); //���ص�����

int main(int argc, char** argv) {
	//��ȡ����Ŀ¼
	char filepath[256];
	_getcwd(filepath, sizeof(filepath));

	//����ģ���ļ�
	string imgsrc = (string)filepath + "/pointtest/2.jpg";

	//��ȡͼƬ
	src = imread(imgsrc);
	//����
	namedWindow(showsrc);
	//���������Ӱ�¼�
	setMouseCallback(showsrc, onMouse);

	imshow(showsrc, src);

	waitKey(0);
	return 0;
}

void onMouse(int event, int x, int y, int flags, void* ustc)
{
	//����������
	if (event == EVENT_LBUTTONUP)
	{
		//��ʼ����δָ��ʱ
		if (startpoint.x == -1 && startpoint.y == -1) {
			startpoint = Point(x, y);
			srccopy = src.clone();
			circle(srccopy, startpoint, 2, Scalar(0, 0, 255));
			imshow(showsrc, srccopy);
		}
		else if (endpoint.x == -1 && endpoint.y == -1) {
			//��ʾ�����յ�λ�û���
			endpoint = Point(x, y);
			circle(srccopy, endpoint, 2, Scalar(0, 0, 255));
			imshow(showsrc, srccopy);

			cout << "startpoint:" << startpoint.x << "," << startpoint.y << endl;
			cout << "endpoint:" << endpoint.x << "," << endpoint.y << endl;
			

			srccopy = src.clone();

			//���÷������м��
			PathDetector detector = PathDetector();
			vector<Point> rescontours;
			vector<Point> inputpoints;
			inputpoints.push_back(startpoint);
			inputpoints.push_back(endpoint);

			//��A*�㷨����·����Դͼ�ϻ����Ľ��
			double Time = (double)getTickCount();
			rescontours = detector.getPath(src, inputpoints);
			Time = ((double)getTickCount() - Time) / getTickFrequency();
			cout << "astar time:" << Time << "ss" << endl;

			if (rescontours.size() > 0) {
				//���еķ��ص���ʾ����
				for (int j = 0; j < rescontours.size() - 1; j++) {
					int k = j + 1;
					line(srccopy, rescontours[j], rescontours[k], Scalar(0, 0, 255), 2);
					imshow(showsrc, srccopy);
				}
			}
			else {
				cout << "�޿ɵ���·��" << endl;
			}


			//��JPS�㷨����·������Դͼ�ϻ����Ľ��
			double t = (double)getTickCount();
			rescontours = detector.getPath(src, inputpoints,1);
			t = ((double)getTickCount() - t) / getTickFrequency();
			cout << "jps time:" << t << "ss" << endl;

			if (rescontours.size() > 0) {
				//���еķ��ص���ʾ����
				for (int j = 0; j < rescontours.size() - 1; j++) {
					int k = j + 1;
					line(srccopy, rescontours[j], rescontours[k], Scalar(255, 0, 0), 2);
					imshow(showsrc, srccopy);
				}
			}
			else {
				cout << "�޿ɵ���·��" << endl;
			}

			
		}
	}
	//������Ҽ�����ʱȡ��ѡ��
	else if (event == EVENT_RBUTTONUP)
	{
		srccopy = src.clone(); //����Դͼ��
		startpoint = Point(-1, -1);
		endpoint = Point(-1, -1);
		imshow(showsrc, srccopy);
	}
}
