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
		//1.灰度图
		cvtColor(dealsrc, dst, COLOR_BGR2GRAY);
		//2.高斯模糊
		GaussianBlur(dst, gray, Size(5, 5), 0.5, 0.5, 4);
		//3.二值化图像
		threshold(gray, gray, 0, 255, THRESH_BINARY | THRESH_OTSU);

		//4.形态学梯度操作
		Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(gray, gray, MORPH_GRADIENT, kernel);

		//5.轮廓找到后加入新的图中
		//vector<vector<Point>> contours;
		//vector<Vec4i> hi;
		//findContours(gray, contours, hi, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		Mat tmp = Mat::zeros(tmpsrc.size(), CV_8UC1);
		//drawContours(tmp, contours, -1, Scalar(255, 255, 255));

		//做一次开操作
		gray.copyTo(tmp);
		morphologyEx(tmp, tmp, MORPH_CLOSE, kernel);

		//5.将地图行动加障碍点存放进数组
		vector<vector<int>> sites;
		for (size_t col = 0; col < tmp.cols; col++) {
			vector<int> rows;
			for (size_t row = 0; row < tmp.rows; row++) {
				int color = tmp.at<uchar>(row, col);
				rows.push_back(color == 255 ? 1 : 0);
			}
			sites.push_back(rows);
		}
		

		//6.进行AStar路径计算生成返回点
		if (type == 0) {
			AStarCalc calc = AStarCalc();
			calc.InitSites(sites);//初始化障碍点

			////设置开始和结束点
			CalcPt startpt = CalcPt(inputPoint[0]);
			CalcPt endpt = CalcPt(inputPoint[1]);

			list<CalcPt* > reslist = calc.GetPath(startpt, endpt);
			for (auto p : reslist) {
				resPoints.push_back(p->pt);
			}
		}
		else {
			//JPS检测
			JPSCalc calcjps = JPSCalc();
			calcjps.InitSites(sites);//初始化障碍点

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
