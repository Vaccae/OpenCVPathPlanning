
#include<opencv2/opencv.hpp>
#include<iostream>


using namespace cv;
using namespace std;


struct CalcJPSPt
{
	Point pt; //OpenCV中点坐标 
	int F, G, H; //F=G+H 
	CalcJPSPt* parent; //parent的坐标，这里没有用指针，从而简化代码 
	CalcJPSPt(Point _pt) :pt(_pt), F(0), G(0), H(0), parent(NULL)  //变量初始化 
	{
	}
	//获取行动方向 
	//{1,0}=右  {-1,0}=左
	//{0,1}=下  {0,-1}=上
	//{1,-1}=右上  {1,1}=右下
	//{-1,-1}=左上  {-1,1}=左下
	Point getDirection() {
		if (parent == NULL) return Point(0, 0);
		//计算从当前点到父节点的行动方向
		int x = (pt.x - parent->pt.x) / max(abs(pt.x - parent->pt.x), 1);
		int y = (pt.y - parent->pt.y) / max(abs(pt.y - parent->pt.y), 1);
		return Point(x, y);
	}
};

class JPSCalc
{
private:

	CalcJPSPt* findPath(CalcJPSPt& startPoint, CalcJPSPt& endPoint);
	//获取需要开始计算的起跳点
	vector<Point> getNeighbourPoints(CalcJPSPt* point) const;
	//寻找跳跃点
	Point checkJumpPoint(Point targetpt, Point prept);

	//判断开启/关闭列表中是否包含某点
	CalcJPSPt* isInList(const std::list<CalcJPSPt*>& list, const CalcJPSPt* point) const;
	//检测障碍点
	bool isInSites(const int x, int y) const;
	//从开启列表中返回最后节点值最小的节点 
	CalcJPSPt* getLeastFpoint();


	vector<vector<int>> sites;  //图片点 0-可通行，1-障碍点
	Point finalpoint;   //终点
	list<CalcJPSPt*> openList;  //开启列表 
	list<CalcJPSPt*> closeList; //关闭列表 

public:
public:
	void InitSites(vector<vector<int>> _sites); //初始化地图
	//获取到路径
	list<CalcJPSPt*> GetPath(Point startPoint, Point endPoint);
};

