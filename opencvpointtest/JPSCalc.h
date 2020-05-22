
#include<opencv2/opencv.hpp>
#include<iostream>


using namespace cv;
using namespace std;


struct CalcJPSPt
{
	Point pt; //OpenCV�е����� 
	int F, G, H; //F=G+H 
	CalcJPSPt* parent; //parent�����꣬����û����ָ�룬�Ӷ��򻯴��� 
	CalcJPSPt(Point _pt) :pt(_pt), F(0), G(0), H(0), parent(NULL)  //������ʼ�� 
	{
	}
	//��ȡ�ж����� 
	//{1,0}=��  {-1,0}=��
	//{0,1}=��  {0,-1}=��
	//{1,-1}=����  {1,1}=����
	//{-1,-1}=����  {-1,1}=����
	Point getDirection() {
		if (parent == NULL) return Point(0, 0);
		//����ӵ�ǰ�㵽���ڵ���ж�����
		int x = (pt.x - parent->pt.x) / max(abs(pt.x - parent->pt.x), 1);
		int y = (pt.y - parent->pt.y) / max(abs(pt.y - parent->pt.y), 1);
		return Point(x, y);
	}
};

class JPSCalc
{
private:

	CalcJPSPt* findPath(CalcJPSPt& startPoint, CalcJPSPt& endPoint);
	//��ȡ��Ҫ��ʼ�����������
	vector<Point> getNeighbourPoints(CalcJPSPt* point) const;
	//Ѱ����Ծ��
	Point checkJumpPoint(Point targetpt, Point prept);

	//�жϿ���/�ر��б����Ƿ����ĳ��
	CalcJPSPt* isInList(const std::list<CalcJPSPt*>& list, const CalcJPSPt* point) const;
	//����ϰ���
	bool isInSites(const int x, int y) const;
	//�ӿ����б��з������ڵ�ֵ��С�Ľڵ� 
	CalcJPSPt* getLeastFpoint();


	vector<vector<int>> sites;  //ͼƬ�� 0-��ͨ�У�1-�ϰ���
	Point finalpoint;   //�յ�
	list<CalcJPSPt*> openList;  //�����б� 
	list<CalcJPSPt*> closeList; //�ر��б� 

public:
public:
	void InitSites(vector<vector<int>> _sites); //��ʼ����ͼ
	//��ȡ��·��
	list<CalcJPSPt*> GetPath(Point startPoint, Point endPoint);
};

