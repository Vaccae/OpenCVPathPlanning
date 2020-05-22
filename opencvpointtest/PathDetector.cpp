#include "PathDetector.h"
#include <future>



vector<Point> PathDetector::getPath(Mat src, vector<Point> inputPoint,int type)
{
	vector<Point> resPoints;

	if (inputPoint.size() < 2) return resPoints;
	try
	{
		Mat tmpsrc, dealsrc;
		src.copyTo(tmpsrc);
		tmpsrc.copyTo(dealsrc);

		//cout << "cols:" << src.cols << " rows:" << src.rows << endl;

		Mat gray, dst;
		//1.�Ҷ�ͼ
		cvtColor(dealsrc, dst, COLOR_BGR2GRAY);
		//2.��˹ģ��
		GaussianBlur(dst, gray, Size(5, 5), 0.5, 0.5, 4);
		//3.��ֵ��ͼ��
		threshold(gray, gray, 0, 255, THRESH_BINARY | THRESH_OTSU);

		//4.��̬ѧ�ݶȲ���
		Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(gray, gray, MORPH_GRADIENT, kernel);

		//5.�����ҵ�������µ�ͼ��
		//vector<vector<Point>> contours;
		//vector<Vec4i> hi;
		//findContours(gray, contours, hi, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		Mat tmp = Mat::zeros(tmpsrc.size(), CV_8UC1);
		//drawContours(tmp, contours, -1, Scalar(255, 255, 255));

		//��һ�ο�����
		gray.copyTo(tmp);
		morphologyEx(tmp, tmp, MORPH_CLOSE, kernel);

		//5.����ͼ�ж����ϰ����Ž�����
		vector<vector<int>> sites;
		for (size_t col = 0; col < tmp.cols; col++) {
			vector<int> rows;
			for (size_t row = 0; row < tmp.rows; row++) {
				int color = tmp.at<uchar>(row, col);
				rows.push_back(color == 255 ? 1 : 0);
			}
			sites.push_back(rows);
		}
		

		//6.����AStar·���������ɷ��ص�
		if (type == 0) {
			AStarCalc calc = AStarCalc();
			calc.InitSites(sites);//��ʼ���ϰ���

			////���ÿ�ʼ�ͽ�����
			CalcPt startpt = CalcPt(inputPoint[0]);
			CalcPt endpt = CalcPt(inputPoint[1]);

			list<CalcPt* > reslist = calc.GetPath(startpt, endpt);
			for (auto p : reslist) {
				resPoints.push_back(p->pt);
			}
		}
		else {
			//JPS���
			JPSCalc calcjps = JPSCalc();
			calcjps.InitSites(sites);//��ʼ���ϰ���

			list<CalcJPSPt* > reslist = calcjps.GetPath(inputPoint[0], inputPoint[1]);
			for (auto p : reslist) {
				resPoints.push_back(p->pt);
			}
		}
	}
	catch (const std::exception & ex)
	{
		cout << ex.what() << endl;
		resPoints.clear();
	}


	return resPoints;
}
