#include<opencv2/opencv.hpp>
#include<iostream>


using namespace cv;
using namespace std;

const int kCost1 = 10; //ֱ��һ��������G 
const int kCost2 = 14; //б��һ��������G 

struct CalcPt
{
public :
	Point pt; //OpenCV�е����� 
	int F, G, H; //F=G+H 
	CalcPt* parent; //parent�����꣬����û����ָ�룬�Ӷ��򻯴��� 
	CalcPt(Point _pt) :pt(_pt), F(0), G(0), H(0), parent(NULL)  //������ʼ�� 
	{
	}
};

class AStarCalc
{
private:

	CalcPt* findPath(CalcPt& startPoint, CalcPt& endPoint, bool isIgnoreCorner);
	//�����ٽ��˸���
	vector<CalcPt*> getSurroundPoints(const CalcPt* point, bool isIgnoreCorner) const;
	//�ж�ĳ���Ƿ����������һ���ж� 
	bool isCanreach(const CalcPt* point, const CalcPt* target, bool isIgnoreCorner) const;
	//�жϿ���/�ر��б����Ƿ����ĳ��
	CalcPt* isInList(const std::list<CalcPt*>& list, const CalcPt* point) const;
	//����ϰ���
	bool isInSites(const Point* point) const;
	//�ӿ����б��з���Fֵ��С�Ľڵ� 
	CalcPt* getLeastFpoint();
	//����FGHֵ 
	int calcG(CalcPt* temp_start, CalcPt* point);
	int calcH(CalcPt* point, CalcPt* end);
	int calcF(CalcPt* point);

	vector<vector<int>> sites;  //ͼƬ�� 0-��ͨ�У�1-�ϰ���
	list<CalcPt*> openList;  //�����б� 
	list<CalcPt*> closeList; //�ر��б� 

	//�Ż�����д��
	void insertList(list<CalcPt*>& list, CalcPt* point);
	//�ӿ����б��з���Fֵ��С�Ľڵ� 
	CalcPt* getInsertListFpoint();
public:
	void InitSites(vector<vector<int>> _sites); //��ʼ����ͼ
	//��ȡ��·��
	list<CalcPt*> GetPath(CalcPt& startPoint, CalcPt& endPoint, bool isIgnoreCorner = false);
};

