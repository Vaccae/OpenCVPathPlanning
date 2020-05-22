#include "AStarCalc.h"


CalcPt* AStarCalc::findPath(CalcPt& startPoint, CalcPt& endPoint, bool isIgnoreCorner)
{
	bool isNearEndPoint = false;
	//����д�����,��������һ���ڵ㣬�������
	CalcPt* firstpt = new CalcPt(Point(startPoint.pt.x, startPoint.pt.y));
	firstpt->H = calcH(firstpt, &endPoint);
	firstpt->F = calcF(firstpt);

	openList.push_front(firstpt);
	//��Ϊ�������
	//insertList(openList, firstpt);

	while (!openList.empty()) {
		//�ҵ�Fֵ��С�ĵ�
		auto curPoint = getLeastFpoint();

		//�ӿ����б���ɾ��
		openList.remove(curPoint);
		//��ŵ��ر��б���
		closeList.push_front(curPoint);


		//����С��100ʱ�����յ���,
		//�ô˷������Լ����ԭ��������800����
		if (curPoint->H < 100) isNearEndPoint = true;

		//1.�ҵ���ǰ����Χ�˸����п���ͨ���ĵ�
		auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);

		for (auto& target : surroundPoints) {
			//2.��ĳһ���㣬������ڿ����б��У����뵽�����б��У����õ�ǰ��Ϊ���ڵ㣬����F,G,H��ֵ
			CalcPt* targetpt = isInList(openList, target);
			if (!targetpt) {
				//����F,G,H��ֵ
				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);

				target->parent = curPoint;

				//���뵽�����б���
				openList.push_front(target);
				//insertList(openList, target);
			}
			//3.��ĳ�����ڿ����б��м���Gֵ�������ԭ���Ĵ󣬾�ʲô������
			//�����������ĸ��ڵ�Ϊ��ǰ�㣬������G��F
			else {
				int tempG = calcG(curPoint, targetpt);
				if (tempG < targetpt->G) {
					targetpt->parent = curPoint;

					targetpt->G = tempG;
					targetpt->F = calcF(targetpt);
				}
			}

			//�жϿ쵽�յ��ſ�ʼ����
			if (isNearEndPoint) {
				CalcPt* resPoint = isInList(openList, &endPoint);
				//�����б���Ľڵ�ָ�룬��Ҫ��ԭ�������endpointָ�룬��Ϊ��������� 
				if (resPoint)
					return resPoint;
			}
		}
	}
	return NULL;
}


//���㵱ǰ����Χ�˸���
vector<CalcPt*> AStarCalc::getSurroundPoints(const CalcPt* point, bool isIgnoreCorner) const
{
	vector<CalcPt*> surroundPoints;

	for (int x = point->pt.x - 1; x <= point->pt.x + 1; ++x) {
		for (int y = point->pt.y - 1; y <= point->pt.y + 1; ++y) {
			if (isCanreach(point, new CalcPt(Point(x, y)), isIgnoreCorner)) {
				surroundPoints.push_back(new CalcPt(Point(x, y)));
			}
		}
	}

	return surroundPoints;
}


//�ж��Ƿ���ܽ��й滮
bool AStarCalc::isCanreach(const CalcPt* point, const CalcPt* target, bool isIgnoreCorner) const
{
	//����С��0ֱ�Ӳ�������
	if (target->pt.x < 0 || target->pt.y < 0
		//����ĵ��뵱ǰһ��Ҳ������
		|| target->pt.x == point->pt.x && target->pt.y == point->pt.y
		//�жϵ����ϰ����в�����
		|| isInSites(&target->pt)
		//������ڹر��б���Ҳ������
		|| isInList(closeList, target))
		return false;
	else {
		//�����ֱ�߿��Լ���
		if (abs(point->pt.x - target->pt.x) + abs(point->pt.y - target->pt.y) == 1)
			return true;
		else {
			//б��Ҫ�жϵ�ǰֱ�߻�񱻰�ס��Ҳ����˵ֱ�����Ƿ����ϰ�����
			Point tmppt1 = Point(point->pt.x, target->pt.y);
			Point tmppt2 = Point(target->pt.x, point->pt.y);
			if (!isInSites(&tmppt1) && !isInSites(&tmppt2))
				return true;
			else
				return isIgnoreCorner;
		}
	}
}

//�жϿ���/�ر��б����Ƿ����ĳ��
CalcPt* AStarCalc::isInList(const std::list<CalcPt*>& list, const CalcPt* point) const
{
	//�ж�ĳ���ڵ��Ƿ����б��У����ﲻ�ܱȽ�ָ�룬��Ϊÿ�μ����б����¿��ٵĽڵ㣬ֻ�ܱȽ�����
	for (auto p : list)
		if (p->pt.x == point->pt.x && p->pt.y == point->pt.y)
			return p;
	return NULL;
}

//�ж��Ƿ����ϰ���
bool AStarCalc::isInSites(const Point* point) const
{
	if (point->x < 0 || point->y < 0 || point->x >= sites.size()
		|| point->y >= sites[0].size()) return true;
	return sites[point->x][point->y] == 1;
}

//�ӿ����б��з���Fֵ��С�Ľڵ� 
CalcPt* AStarCalc::getLeastFpoint()
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

//����Gֵ
int AStarCalc::calcG(CalcPt* temp_start, CalcPt* point)
{
	//�жϵ�ǰ��������ԭ����ֱ�߻���б�ߣ���ȡ��ͬ������ֵ
	//�������ҵĵ��ƶ��õ���ֵ�϶���1����Ϊ1����б���ƶ�
	int tempG = abs(point->pt.x - temp_start->pt.x) + abs(point->pt.y - temp_start->pt.y) == 1 ? kCost1 : kCost2;
	//�ж��ǲ��ǳ�ʼ�Ľڵ㣬����ǳ�ʼ��Լ�����丸�ڵ�Ϊ��
	int parentG = point->parent == NULL ? 0 : point->parent->G;
	//����G��������ж�����ֱ�ߺ�б�������ĵ���G
	return parentG + tempG;
}

//����Hֵ
int AStarCalc::calcH(CalcPt* point, CalcPt* end)
{
	//�����յ㵽��ǰ���Point��ֵ
	Point tmppoint = end->pt - point->pt;
	//����ŷ����¾������H
	return sqrt(pow(tmppoint.x, 2) + pow(tmppoint.y, 2)) * 10;
}

//����Fֵ
int AStarCalc::calcF(CalcPt* point)
{
	//��ʽ F=G+H
	return point->F = point->G + point->H;
}

//�Ż��Ĳ���д��
void AStarCalc::insertList(list<CalcPt*>& list, CalcPt* point)
{
	if (list.size() == 0) {
		list.push_front(point);
		return;
	}
	//����������
	std::list<CalcPt*>::iterator it = list.begin();
	//���ұ���������
	for (int i = 0; i < list.size() && it != list.end(); ++i) {
		if (it.operator*()->F >= point->F) {
			list.insert(it, point);
			return;
		}
		++it;
	}
	//û�������Ļ����뵽���
	list.push_back(point);
}

CalcPt* AStarCalc::getInsertListFpoint()
{
	if (!openList.empty())
	{
		return openList.front();
	}
	return NULL;
}

//��ʼ����ͼ
void AStarCalc::InitSites(vector<vector<int>> _sites)
{
	sites = _sites;
}


list<CalcPt*> AStarCalc::GetPath(CalcPt& startPoint, CalcPt& endPoint, bool isIgnoreCorner)
{
	CalcPt* result = findPath(startPoint, endPoint, isIgnoreCorner);
	list<CalcPt*> path;
	//����·�������û���ҵ�·�������ؿ�����
	while (result) {
		path.push_back(result);
		result = result->parent;
	}
	return path;
}
