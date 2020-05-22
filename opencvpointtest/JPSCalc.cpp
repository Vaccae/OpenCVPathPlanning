#include "JPSCalc.h"

CalcJPSPt* JPSCalc::findPath(CalcJPSPt& startPoint, CalcJPSPt& endPoint)
{
	//����д�����,��������һ���ڵ㣬�������
	CalcJPSPt* firstpt = new CalcJPSPt(Point(startPoint.pt.x, startPoint.pt.y));
	openList.push_back(firstpt);
	while (!openList.empty()) {
		//cout << "openlistcount:" << openList.size() << endl;
		//�ҵ�Fֵ��С�ĵ�
		auto curPoint = getLeastFpoint();

		//�ҵ���ӿ����б���ɾ��
		openList.remove(curPoint);
	    //�ٰѵ�ǰ���ŵ��ر��б���
		closeList.push_front(curPoint);

		//F=G+H 
		int d, f, g, h;
		//��������
		Point jp;
		//��ȡ�����ļ�����ʼ��
		vector<Point> nbps = getNeighbourPoints(curPoint);
		//���������Ѱ����Ծ��
		for (int i = 0; i < nbps.size(); i++) {
			jp = checkJumpPoint(nbps[i], curPoint->pt);
			//cout << "jp:" << jp.x << "," << jp.y << endl;
			//���겻����-1˵���ҵ�������
			if (jp.x != -1) {
				CalcJPSPt* now = new CalcJPSPt(jp);
				//����ڹر��б��������
				if (isInList(closeList, now)) continue;

				//�������㵽��ǰ��ľ���d
				d = abs(jp.x - curPoint->pt.x) + abs(jp.y - curPoint->pt.y);
				//����Gֵ
				g= curPoint->G + d;
				//����Hֵ
				h = abs(jp.x - finalpoint.x) + abs(jp.y - finalpoint.y);

				//�жϵ�ǰ�����ڲ���openlist��,����ɾ�����������¼�
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
					//д�뿪���б�
					openList.push_front(now);
				}

				CalcJPSPt* resPoint = isInList(openList, &endPoint);
				//�����б���Ľڵ�ָ�룬��Ҫ��ԭ�������endpointָ�룬��Ϊ��������� 
				if (resPoint)
					return resPoint;
			}
		}
	}
	return NULL;
}

//��ȡ��������Ծ�����ʼ��
vector<Point> JPSCalc::getNeighbourPoints(CalcJPSPt* point) const
{
	vector<Point> nbps;
	Point dir;
	//�жϵ�ǰ�������
	if (point->parent == NULL) {
		for (int x = -1; x <= 1; ++x) {
			for (int y = -1; y <= 1; ++y) {
				//��ǰ�㲻���ж�
				if (x == 0 && y == 0)continue;
				//��������ϰ���������
				int dx = point->pt.x + x;
				int dy = point->pt.y + y;
				if (!isInSites(dx, dy)) {
					nbps.push_back(Point(dx, dy));
				}
			}
		}
	}
	//��ǰ�㲻�����
	else {
		//1.��ȡ���ڵ㵽��ǰ����ж��켣
		dir = point->getDirection();
		//2.���ж��켣Ϊб��ʱ
		if (dir.x != 0 && dir.y != 0) {
			//2.1��ֱ�ƶ����ж�
			if (!isInSites(point->pt.x, point->pt.y + dir.y))
				nbps.push_back(Point(point->pt.x, point->pt.y + dir.y));
			//2.2ˮƽ�ƶ����ж�
			if (!isInSites(point->pt.x + dir.x, point->pt.y))
				nbps.push_back(Point(point->pt.x + dir.x, point->pt.y));
			//2.3б���ж�
			if (!isInSites(point->pt.x + dir.x, point->pt.y + dir.y))
				nbps.push_back(Point(point->pt.x + dir.x, point->pt.y + dir.y));
			//2.4���X����ǿ���ھӣ������ж�
			if (isInSites(point->pt.x - dir.x, point->pt.y)
				&& !isInSites(point->pt.x - dir.x, point->pt.y + dir.y))
				nbps.push_back(Point(point->pt.x - dir.x, point->pt.y + dir.y));
			//2.5���Y����ǿ���ھ�,�����ж�
			if (isInSites(point->pt.x, point->pt.y - dir.y)
				&& !isInSites(point->pt.x + dir.x, point->pt.y - dir.y))
				nbps.push_back(Point(point->pt.x + dir.x, point->pt.y - dir.y));
		}
		//3.���ж��켣Ϊֱ��ʱ
		else {
			//3.1����X��ˮƽ�ƶ�ʱ
			if (dir.x != 0) {
				//3.1.1ˮƽ�ߴ���
				if (!isInSites(point->pt.x + dir.x, point->pt.y))
					nbps.push_back(Point(point->pt.x + dir.x, point->pt.y));
				//3.1.2���Ϸ���ǿ���ھ�ʱ�������ж�
				if (isInSites(point->pt.x, point->pt.y - 1)
					&& !isInSites(point->pt.x + dir.x, point->pt.y - 1))
					nbps.push_back(Point(point->pt.x + dir.x, point->pt.y - 1));
				//3.1.3���·���ǿ���ھ�ʱ�������ж�
				if (isInSites(point->pt.x, point->pt.y + 1)
					&& !isInSites(point->pt.x + dir.x, point->pt.y + 1))
					nbps.push_back(Point(point->pt.x + dir.x, point->pt.y + 1));
			}
			//3.2����Y�ᴹֱ�ƶ�ʱ
			else {
				//3.2.1��ֱ���ƶ�
				if (!isInSites(point->pt.x, point->pt.y + dir.y))
					nbps.push_back(Point(point->pt.x, point->pt.y + dir.y));
				//3.2.2�������ǿ���ھ�ʱ�������ж�
				if (isInSites(point->pt.x - 1, point->pt.y)
					&& !isInSites(point->pt.x - 1, point->pt.y + dir.y))
					nbps.push_back(Point(point->pt.x - 1, point->pt.y + dir.y));
				//3.2.3���ұ���ǿ���ھ�ʱ�������ж�
				if (isInSites(point->pt.x + 1, point->pt.y)
					&& !isInSites(point->pt.x + 1, point->pt.y + dir.y))
					nbps.push_back(Point(point->pt.x + 1, point->pt.y + dir.y));
			}
		}
	}
	//���ص㼯
	return nbps;
}

//������Ծ��,����һ����ǰ�㣬��������ǰһ����
Point JPSCalc::checkJumpPoint(Point targetpt, Point prept)
{
	//����ǰһ�㵽��ǰ����ж�·����x��y��������0˵����б���ߵ�
	//x=0˵���������ƶ���y=0˵���Ǻ����ƶ�
	Point dir = targetpt - prept;
	//������ʱ���������û����Ծ�㷵��-1��-1����������ͼ��
	Point tmp = Point(-1, -1);

	//��⵱ǰ���ڵ�ͼ���Ƿ������ж�,�����ǰ�����ϰ���ֱ�ӷ���tmp����
	if (isInSites(targetpt.x, targetpt.y)) {
		return tmp;
	}
	//������յ㣬ֱ�ӷ��ص�ǰ��
	if (targetpt.x == finalpoint.x && targetpt.y == finalpoint.y) {
		return targetpt;
	}

	//1.��⵱ǰ���Ƿ���ǿ���ھӣ�������ھͷ��ص�ǰ��Ϊ��Ծ��
	//1.1�ж���б���ƶ�
	if (dir.x != 0 && dir.y != 0) {
		if ((!isInSites(targetpt.x - dir.x, targetpt.y + dir.y)
			&& isInSites(targetpt.x - dir.x, targetpt.y))
			|| (!isInSites(targetpt.x + dir.x, targetpt.y - dir.y)
				&& isInSites(targetpt.x, targetpt.y - dir.y))) {
			//cout << "б��ǿ���ھ�:" << targetpt.x << "," << targetpt.y << endl;
			return targetpt;
		}

	}
	//1.2�ж�ֱ���ƶ�
	else {
		//1.2.1�ж��Ǻ����ƶ�
		if (dir.x != 0) {
			if ((!isInSites(targetpt.x + dir.x, targetpt.y + 1)
				&& isInSites(targetpt.x, targetpt.y + 1))
				|| (!isInSites(targetpt.x + dir.x, targetpt.y - 1)
					&& isInSites(targetpt.x, targetpt.y - 1)))
			{
				//cout << "����ֱ��ǿ���ھ�:" << targetpt.x << "," << targetpt.y << endl;
				return targetpt;
			}

		}
		//1.2.2�ж��������ƶ�
		else {
			if ((!isInSites(targetpt.x - 1, targetpt.y + dir.y)
				&& isInSites(targetpt.x - 1, targetpt.y))
				|| (!isInSites(targetpt.x + 1, targetpt.y + dir.y)
					&& isInSites(targetpt.x + 1, targetpt.y))) {
				//cout << "����ֱ��ǿ���ھ�:" << targetpt.x << "," << targetpt.y << endl;
				return targetpt;
			}

		}
	}

	//2.������ǿ���ھӰ��ж�·������Ѱ����Ծ��
	//2.1 �ж���б���ƶ�,�Ȱ�ˮƽ�������
	if (dir.x != 0 && dir.y != 0) {
		//2.1.1��ˮƽ�������Ѱ����Ծ��
		tmp = checkJumpPoint(Point(targetpt.x + dir.x, targetpt.y), targetpt);
		//2.2.2������һ����ֱ������ٵ���Ծ������ж�
		Point tmp2 = checkJumpPoint(Point(targetpt.x, targetpt.y + dir.y), targetpt);
		//ֻ�з��ص��������ж�x����-1��˵�����ҵ���Ծ�㣬�����ͷ��ص�ǰ��
		if (tmp.x != -1 || tmp2.x != -1) return targetpt;
	}
	//2.2 �ж�ˮƽ�����Ƿ���ƶ���������԰�ԭ���켣�������Ѱ��
	if (!isInSites(targetpt.x + dir.x, targetpt.y)
		|| !isInSites(targetpt.x, targetpt.y + dir.y))
	{
		tmp = checkJumpPoint(Point(targetpt.x + dir.x, targetpt.y + dir.y), targetpt);
		if (tmp.x != -1) return tmp;
	}

	//��󷵻�tmp
	return tmp;
}


//�жϿ���/�ر��б����Ƿ����ĳ��
CalcJPSPt* JPSCalc::isInList(const std::list<CalcJPSPt*>& list, const CalcJPSPt* point) const
{
	//�ж�ĳ���ڵ��Ƿ����б��У����ﲻ�ܱȽ�ָ�룬��Ϊÿ�μ����б����¿��ٵĽڵ㣬ֻ�ܱȽ�����
	for (auto p : list)
		if (p->pt.x == point->pt.x && p->pt.y == point->pt.y)
			return p;
	return NULL;
}

//�����Ƿ����ϰ��� ���Ϊ1�����ϰ���
bool JPSCalc::isInSites(const int x, int y) const
{
	if (x < 0 || y < 0 || x >= sites.size() || y >= sites[0].size()) return true;
	return sites[x][y] == 1;
}

//�ӿ����б��з���Fֵ��С�Ľڵ� 
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
	//����·�������û���ҵ�·�������ؿ�����
	while (result) {
		path.push_back(result);
		result = result->parent;
	}
	return path;
}
