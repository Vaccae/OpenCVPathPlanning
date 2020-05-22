#include "JPSCalc.h"

CalcJPSPt* JPSCalc::findPath(CalcJPSPt& startPoint, CalcJPSPt& endPoint)
{
	//首先写入起点,拷贝开启一个节点，内外隔离
	CalcJPSPt* firstpt = new CalcJPSPt(Point(startPoint.pt.x, startPoint.pt.y));
	openList.push_back(firstpt);
	while (!openList.empty()) {
		//cout << "openlistcount:" << openList.size() << endl;
		//找到F值最小的点
		auto curPoint = getLeastFpoint();

		//找到后从开启列表中删除
		openList.remove(curPoint);
	    //再把当前点存放到关闭列表中
		closeList.push_front(curPoint);

		//F=G+H 
		int d, f, g, h;
		//跳点坐标
		Point jp;
		//获取传入点的计算起始点
		vector<Point> nbps = getNeighbourPoints(curPoint);
		//遍历计算点寻找跳跃点
		for (int i = 0; i < nbps.size(); i++) {
			jp = checkJumpPoint(nbps[i], curPoint->pt);
			//cout << "jp:" << jp.x << "," << jp.y << endl;
			//坐标不等于-1说明找到了跳点
			if (jp.x != -1) {
				CalcJPSPt* now = new CalcJPSPt(jp);
				//如果在关闭列表里，则跳出
				if (isInList(closeList, now)) continue;

				//计算跳点到当前点的距离d
				d = abs(jp.x - curPoint->pt.x) + abs(jp.y - curPoint->pt.y);
				//计算G值
				g= curPoint->G + d;
				//计算H值
				h = abs(jp.x - finalpoint.x) + abs(jp.y - finalpoint.y);

				//判断当前跳点在不在openlist中,存在删除，后面重新加
				CalcJPSPt* hasnow = isInList(openList, now);
				if (hasnow) {
					hasnow->G = g;
					hasnow->H = h;
					hasnow->F = g + h;
					hasnow->parent = curPoint;
				}
				else {
					now->G = g;
					now->H = h;
					now->F = g + h;
					now->parent = curPoint;
					//写入开启列表
					openList.push_front(now);
				}

				CalcJPSPt* resPoint = isInList(openList, &endPoint);
				//返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝 
				if (resPoint)
					return resPoint;
			}
		}
	}
	return NULL;
}

//获取到计算跳跃点的起始点
vector<Point> JPSCalc::getNeighbourPoints(CalcJPSPt* point) const
{
	vector<Point> nbps;
	Point dir;
	//判断当前点是起点
	if (point->parent == NULL) {
		for (int x = -1; x <= 1; ++x) {
			for (int y = -1; y <= 1; ++y) {
				//当前点不再判断
				if (x == 0 && y == 0)continue;
				//如果不是障碍点加入计算
				int dx = point->pt.x + x;
				int dy = point->pt.y + y;
				if (!isInSites(dx, dy)) {
					nbps.push_back(Point(dx, dy));
				}
			}
		}
	}
	//当前点不是起点
	else {
		//1.获取父节点到当前点的行动轨迹
		dir = point->getDirection();
		//2.当行动轨迹为斜线时
		if (dir.x != 0 && dir.y != 0) {
			//2.1垂直移动点判断
			if (!isInSites(point->pt.x, point->pt.y + dir.y))
				nbps.push_back(Point(point->pt.x, point->pt.y + dir.y));
			//2.2水平移动点判断
			if (!isInSites(point->pt.x + dir.x, point->pt.y))
				nbps.push_back(Point(point->pt.x + dir.x, point->pt.y));
			//2.3斜点判断
			if (!isInSites(point->pt.x + dir.x, point->pt.y + dir.y))
				nbps.push_back(Point(point->pt.x + dir.x, point->pt.y + dir.y));
			//2.4如果X轴有强迫邻居，加入判断
			if (isInSites(point->pt.x - dir.x, point->pt.y)
				&& !isInSites(point->pt.x - dir.x, point->pt.y + dir.y))
				nbps.push_back(Point(point->pt.x - dir.x, point->pt.y + dir.y));
			//2.5如果Y轴有强迫邻居,加入判断
			if (isInSites(point->pt.x, point->pt.y - dir.y)
				&& !isInSites(point->pt.x + dir.x, point->pt.y - dir.y))
				nbps.push_back(Point(point->pt.x + dir.x, point->pt.y - dir.y));
		}
		//3.当行动轨迹为直线时
		else {
			//3.1当是X轴水平移动时
			if (dir.x != 0) {
				//3.1.1水平线处理
				if (!isInSites(point->pt.x + dir.x, point->pt.y))
					nbps.push_back(Point(point->pt.x + dir.x, point->pt.y));
				//3.1.2当上方有强迫邻居时，加入判断
				if (isInSites(point->pt.x, point->pt.y - 1)
					&& !isInSites(point->pt.x + dir.x, point->pt.y - 1))
					nbps.push_back(Point(point->pt.x + dir.x, point->pt.y - 1));
				//3.1.3当下方有强迫邻居时，加入判断
				if (isInSites(point->pt.x, point->pt.y + 1)
					&& !isInSites(point->pt.x + dir.x, point->pt.y + 1))
					nbps.push_back(Point(point->pt.x + dir.x, point->pt.y + 1));
			}
			//3.2当是Y轴垂直移动时
			else {
				//3.2.1垂直线移动
				if (!isInSites(point->pt.x, point->pt.y + dir.y))
					nbps.push_back(Point(point->pt.x, point->pt.y + dir.y));
				//3.2.2当左边有强迫邻居时，加入判断
				if (isInSites(point->pt.x - 1, point->pt.y)
					&& !isInSites(point->pt.x - 1, point->pt.y + dir.y))
					nbps.push_back(Point(point->pt.x - 1, point->pt.y + dir.y));
				//3.2.3当右边有强迫邻居时，加入判断
				if (isInSites(point->pt.x + 1, point->pt.y)
					&& !isInSites(point->pt.x + 1, point->pt.y + dir.y))
					nbps.push_back(Point(point->pt.x + 1, point->pt.y + dir.y));
			}
		}
	}
	//返回点集
	return nbps;
}

//计算跳跃点,参数一：当前点，参数二：前一个点
Point JPSCalc::checkJumpPoint(Point targetpt, Point prept)
{
	//计算前一点到当前点的行动路径，x和y都不等于0说明是斜线走的
	//x=0说明是纵向移动，y=0说明是横向移动
	Point dir = targetpt - prept;
	//设置临时变量，如果没有跳跃点返回-1，-1代码跳出地图了
	Point tmp = Point(-1, -1);

	//检测当前点在地图中是否允许行动,如果当前点是障碍点直接返回tmp跳出
	if (isInSites(targetpt.x, targetpt.y)) {
		return tmp;
	}
	//如果是终点，直接返回当前点
	if (targetpt.x == finalpoint.x && targetpt.y == finalpoint.y) {
		return targetpt;
	}

	//1.检测当前点是否有强迫邻居，如果存在就返回当前点为跳跃点
	//1.1判断是斜线移动
	if (dir.x != 0 && dir.y != 0) {
		if ((!isInSites(targetpt.x - dir.x, targetpt.y + dir.y)
			&& isInSites(targetpt.x - dir.x, targetpt.y))
			|| (!isInSites(targetpt.x + dir.x, targetpt.y - dir.y)
				&& isInSites(targetpt.x, targetpt.y - dir.y))) {
			//cout << "斜线强迫邻居:" << targetpt.x << "," << targetpt.y << endl;
			return targetpt;
		}

	}
	//1.2判断直线移动
	else {
		//1.2.1判断是横向移动
		if (dir.x != 0) {
			if ((!isInSites(targetpt.x + dir.x, targetpt.y + 1)
				&& isInSites(targetpt.x, targetpt.y + 1))
				|| (!isInSites(targetpt.x + dir.x, targetpt.y - 1)
					&& isInSites(targetpt.x, targetpt.y - 1)))
			{
				//cout << "横向直线强迫邻居:" << targetpt.x << "," << targetpt.y << endl;
				return targetpt;
			}

		}
		//1.2.2判断是纵向移动
		else {
			if ((!isInSites(targetpt.x - 1, targetpt.y + dir.y)
				&& isInSites(targetpt.x - 1, targetpt.y))
				|| (!isInSites(targetpt.x + 1, targetpt.y + dir.y)
					&& isInSites(targetpt.x + 1, targetpt.y))) {
				//cout << "纵向直线强迫邻居:" << targetpt.x << "," << targetpt.y << endl;
				return targetpt;
			}

		}
	}

	//2.不存在强迫邻居按行动路径继续寻找跳跃点
	//2.1 判断是斜线移动,先按水平方向查找
	if (dir.x != 0 && dir.y != 0) {
		//2.1.1按水平方向继续寻找跳跃点
		tmp = checkJumpPoint(Point(targetpt.x + dir.x, targetpt.y), targetpt);
		//2.2.2再增加一个垂直方向跟踪的跳跃点进行判断
		Point tmp2 = checkJumpPoint(Point(targetpt.x, targetpt.y + dir.y), targetpt);
		//只有返回的两个点判断x不是-1，说明能找到跳跃点，这样就返回当前点
		if (tmp.x != -1 || tmp2.x != -1) return targetpt;
	}
	//2.2 判断水平方向是否可移动，如果可以按原来轨迹方向继续寻找
	if (!isInSites(targetpt.x + dir.x, targetpt.y)
		|| !isInSites(targetpt.x, targetpt.y + dir.y))
	{
		tmp = checkJumpPoint(Point(targetpt.x + dir.x, targetpt.y + dir.y), targetpt);
		if (tmp.x != -1) return tmp;
	}

	//最后返回tmp
	return tmp;
}


//判断开启/关闭列表中是否包含某点
CalcJPSPt* JPSCalc::isInList(const std::list<CalcJPSPt*>& list, const CalcJPSPt* point) const
{
	//判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
	for (auto p : list)
		if (p->pt.x == point->pt.x && p->pt.y == point->pt.y)
			return p;
	return NULL;
}

//检测点是否是障碍点 如果为1则是障碍点
bool JPSCalc::isInSites(const int x, int y) const
{
	if (x < 0 || y < 0 || x >= sites.size() || y >= sites[0].size()) return true;
	return sites[x][y] == 1;
}

//从开启列表中返回F值最小的节点 
CalcJPSPt* JPSCalc::getLeastFpoint()
{
	if (!openList.empty())
	{
		auto resPoint = openList.front();
		for (auto& point : openList)
			if (point->F < resPoint->F)
				resPoint = point;
		return resPoint;
	}
	return NULL;
}

void JPSCalc::InitSites(vector<vector<int>> _sites)
{
	sites = _sites;
}

list<CalcJPSPt*> JPSCalc::GetPath(Point startPoint, Point endPoint)
{
	finalpoint = endPoint;
	CalcJPSPt startpt = CalcJPSPt(startPoint);
	CalcJPSPt endpt = CalcJPSPt(finalpoint);
	CalcJPSPt* result = findPath(startpt, endpt);
	list<CalcJPSPt*> path;
	//返回路径，如果没有找到路径，返回空链表
	while (result) {
		path.push_back(result);
		result = result->parent;
	}
	return path;
}
