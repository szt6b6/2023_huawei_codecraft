#include <list>
#include <map>
#include <queue>
#include <math.h>
#include <iostream>
#include <fstream> 
//#include <sstream>
#include <algorithm>
#include <cstring>
//#include <stdio.h>
//#include <stdlib.h>



//----------��������-----------//
extern int to_sell_need_frames_error;
extern int to_buy_need_frames_error;
extern double sell_bench_logical_dist_estimate;
extern double count_have_materials_weight;
extern double producing_status_weight;
extern double limit_bench_to_buy;
extern double average_speed;
extern int material_demand[7];
extern double bench_out_edge_num_weight;
extern double bench_in_edge_num_weight;
extern double material_demand_weight;

extern bool map_2_flag;
extern double priority_sell_weight;

#define row1 100 //����ȡ�ı��ļ������ݵ�����
#define col1 100 //����ȡ�ı��ļ������ݵ�����

//----------�ļ���������
#define robotSradius  0.45  //�����˳�̬�뾶
#define robotBradius  0.53    //���������ذ뾶
#define collGap   3.5//�������ж���ײ��С���
#define avoidBorderMDist  1.7 //��������ѡ��Զ���Ե����С�����Ե�ľ���
#define RobotNum 4
#define PI 3.1415926535    //Բ����

//----------�ļ���������
#define actionNum 3//�����˴ﵽһ������̨���3������

//----------�ļ���������
#define DeskMaxNum 50 //����̨�������  

#define maxForwardSpeed  6
#define maxBackSpeed  2
#define maxAngleSpeed PI
#define ANGELACC  38.621 //�������صĽǼ��ٶ�
#define ANGELACC1 20.032  //�����ؽǼ��ٶ�
#define LINESPEEDACC 19.475  //���������߼��ٶ�
#define LINESPEEDACC1   14.385 //�������߼��ٶ�
//findLine.c
#define BoderGridNum 15 //���߸�����Ŀ
#define borderLineWidth 5//��·�����
#define controlRow 2   //���Ƶ�����
extern int roadDir[RobotNum][BoderGridNum];//1Ϊ�ң�2Ϊ�ϣ�3Ϊ��4Ϊ��
extern int realControlRow;
extern int hasBoderFlag[RobotNum];  //��������Χ�Ƿ�����ϰ�
extern int leftBoderGrid[RobotNum][BoderGridNum][2];  //��߱��߸���
extern int rightBoderGrid[RobotNum][BoderGridNum][2];  //�ұ߱��߸���
extern double midPoint[RobotNum][BoderGridNum][2];  //�������
extern int leaveGridNum[RobotNum];          //·��ʣ�������Ŀ
extern double roadWidth[RobotNum][BoderGridNum]; //��·���
extern int realBoderGridNum[RobotNum];   //��ʵ���߸�����Ŀ
extern double controlPoint[RobotNum][2];   //�����˵Ŀ��Ƶ�
extern int temGridList[RobotNum][BoderGridNum][2];//·��������
extern  int findLineFlag[RobotNum][BoderGridNum];//1û���ߣ�0������,2�ұ߶��ߣ�3�ϱ߶���4��߶���5�±߶�


extern double aimDist[RobotNum];   //ÿ�θ���Ŀ�깤��̨����Ҫ������Ӧ��Ŀ�깤��̨����
extern int aimDesk[RobotNum];//��������̨id
extern int aimAction[RobotNum][actionNum];//�����������У�0��ʾ��������    1������2��3����

extern int hasBoderFlag[RobotNum];
extern double avoidBorderGap[RobotNum]; //�ı��������Զ���Եģʽ�Ļ����
extern int legSelectFlag[RobotNum];//ÿ�α����ϰ���ת������ת��leg����1��ʾ������ת��Ϊ����2��ʾΪ����0��ʾδ����ѡ��
extern int collisionFlag[RobotNum][RobotNum];//��ײ��־λ,��ά�����ʾ
extern int collisionStateFlag[RobotNum];//��ײ״̬��־λ��0��û����1������ײ��
extern double avoidAngleGap[RobotNum];//�ı������ͨģʽ�Ļ����
extern char map_data[100][100];//��ͼ����
extern char map_data_ob_expand_sell[100][100]; //��չ�ϰ���ĵ�ͼ
extern char map_data_ob_expand_buy[100][100];
extern double map_data_cost[100][100];
extern int frameID;  //��ǰ֡���
extern int hasMoney;//ӵ�н�Ǯ
extern int deskNum;//����̨����
extern int robotNum;//����������


extern double angleErr[RobotNum];;//�Ƕ�ƫ��


extern int leftBoderGrid[RobotNum][BoderGridNum][2];//��߽߱��������
extern int rightBoderGrid[RobotNum][BoderGridNum][2];//�������ұ߽߱��������
extern double midPoint[RobotNum][BoderGridNum][2];//��������������
extern double lineSpeed[RobotNum];//��ǰ���ٶȱ���
extern int backFlag[RobotNum];//�������Ƿ����
extern double robot2robotDist[RobotNum][RobotNum]; //�������໥֮��ľ���
extern double setAngleSpeed[RobotNum];//���ý��ٶȣ���ʱ��Ϊ��
extern double setLineSpeed[RobotNum];//�������ٶȣ���ʱ��Ϊ��
extern double  lastSetSpeed[RobotNum];//��һ���������ٶ�
extern double  lastSetAngleSpeed[RobotNum];//��һ�����ý��ٶ�
extern double expectSpeed[RobotNum]; //�����ٶ�
extern double avoidExAngle[RobotNum];//���ϻ�������
extern int gridCollFlag[RobotNum] ;//�����˸�����ײ��־λ
extern double avGridExAngle[RobotNum];//�˳���ײ״̬������
extern double avoidGridGap[RobotNum];//�ı������ͨģʽ�Ļ����
extern int hasBoderFlag[RobotNum];//��������Χ�Ƿ�����ϰ���
extern double direction_Kp;
extern double direction_Kd;
extern double speed_Kp;
extern double speed_Kd;
extern int mapFlag;

typedef struct Desk
{
	int deskType[DeskMaxNum];//����̨����
	double local[DeskMaxNum][2];//����
	int waitTime[DeskMaxNum];//ʣ������ʱ��֡
	int materialState[DeskMaxNum];//ԭ���ϸ�״̬
	int productState[DeskMaxNum];//��Ʒ��״̬
}DeskData;
typedef struct Robot {
	int deskID[RobotNum];//��������̨ ID
	int productType[RobotNum];//Я����Ʒ����
	double local[RobotNum][2];//����
	double timeC[RobotNum];//ʱ��ϵ��
	double collC[RobotNum];//��ײϵ��
	double angleSpeed[RobotNum];//���ٶ�
	double lineSpeed[RobotNum][2];//���ٶ�
	double towardAngle[RobotNum];//����
	double radius[RobotNum];//�뾶
	double angleAcc[RobotNum]; //�Ǽ��ٶ�
	double lineSpeedAcc[RobotNum];//�߼��ٶ�
	int  grid[RobotNum][2];//�����������ڸ������꣬���Ͻ���0 0
}RobotData;
extern DeskData workDesk;//����̨�ṹ����
extern RobotData robot;//�����˽ṹ����

//ȫ��·��A*�㷨 һ��node����һ����
class Node {
public:
	Node *parent; //ָ����һ���� ������ʼ�ڵ�
	Node *next; //ָ����һ���ڵ� Ѱ·��ʹ��
	int row;
	int col;
	double real_cost;
	double esti_cost;

	Node(int row, int col);
	Node(int row, int col, Node *parent);
	Node(Node *p);

	int* local();
	double get_total_cost();

};

//pathDesign_c
typedef struct {
	int x;
	int y;
} Point;

typedef struct Node_c {
	Point point;
	double f; // f = g + h
	double g;
	double c;
	struct Node_c* parent;
	struct Node_c* next;
	int dir;
} Node_c;


typedef struct WorkingRobot {

	int robotID;//�����˱��0-3
	int buyBenchSeq;//����̨id
	int sellBenchSeq;//����̨id
	int productType;//������Ʒ����
	int buyOrSell;//buy->1; sell->0; just go->2
	double predictFrameDoneTask;
	std::list<Node_c> path;

} WorkingRobot;


extern std::list<int> buy_task_seq_queue;//������̨��Ŷ���
extern WorkingRobot working_robots_queue[4];//�����л����˶���
extern std::map<int, std::list<int>> where_to_sell;
extern double product_profit_dict[8];
extern bool sell_benchs_in_queue[50][8];
extern int buy_benchs_in_queue[50];
extern bool free_robots[4];
extern std::map<int, std::list<int>> infor_working_benchs_dict;//�洢��Ӧ���͵Ĺ���̨
extern int bench_in_edge_num[DeskMaxNum]; //��¼����̨���������
extern int bench_out_edge_num[DeskMaxNum]; //��¼����̨��������


//----------�ļ���������
extern void avoidOb();
extern bool dealColl(int robotid1, int robotid2);
extern void avoidBorder();
extern void avoidGrid(int robotid);
extern int judgeColl(int robotid);
extern bool judgeLaterColl(int robotid1, int robotid2, double laterLineSpeed1[2], double laterLineSpeed2[2]);
extern void filterCollSpeed(double time);
extern void findBorderLine(int robotID);

//----------�ļ���������
//----------�ļ���������
extern double calculateAngle(double*startPoint, double* endPoint);
extern double calculateDist(double* startPoint, double* endPoint);
extern double calculateL2(double x, double y);
extern double minF(double x, double y);
extern int minI(int x, int y);
extern int maxI(int x, int y);
extern double absF(double x);
extern int absI(int x);
//extern int sumIntArray(int a[]);

//----------�ļ���������

extern bool readMap();//��ʼ����ȡ��ͼ
extern bool readState();//ÿ֡�����˺͹���̨״̬��ȡ
extern void outCMD(void);//�������ָ��
extern void expand_ob_sell(int i, int j); //������չ�ϰ����ͼ

extern void update_queue();
extern void update_queue_init(); //��ʼ����ʱ��ͽ�����

extern void run_task();

extern double getPriority(int bench_type);

//----------�ļ���������
extern void getAim();



//----------�ļ���������
extern void speedControl();
extern void linSpeedControl(int robotid);//�õ��������ٶ�
extern void Direction_Control_Error(int robotid);//�õ��������ٶ�
extern bool judgeHasBorder(int robotid, int len,int startLine);
extern void avoidDireControl(int robotid);
extern void avoidSpeedControl(int robotid);
extern void calculateRobotState();//�õ���������صĲ���
extern double getAngleErr(double* point1, double* point2, double towardAngle);
extern double estiamteTime(double* point1, double* point2, double towardAngle);//���ƻ����˵��﹤��̨ʱ��
extern double estiamteTime2(double* point1, double* point2, double* point3, double towardAngle);
extern void estiamteSpeed(int robotid, double time, double speedAcc, double angleAcc, double* estimateV);
extern void getAimPoint(double* aimPoint, int robotID);
extern void avGridDirControl(int robotid);
extern void avGridSpeedControl(int robotid);
extern void judgeGridColl(int robotid);
extern void dealGridColl(int robotid, double* mayBePoint, int currentSearchNum);
extern void colledDeal();


extern Node* sourceNode; //������
extern Node* targetNode; //Ŀ���
extern std::list<Node> res_path;
extern Node* all_node_pointers[100][100];


extern bool node_added[100][100];//��ʾ���Ƿ��Ѿ����������ȼ��б���
extern bool node_isSearched[100][100]; //��ʾ���Ƿ��Ѿ�������
extern char map_data_to_look_path[100][100];
//����NodeС���ѵıȽ������
class cmp_a_star {
public:
	bool operator()(Node* a, Node* b) {
		return a->get_total_cost() > b->get_total_cost();
	}
};
class cmp_dijkstra {
public:
	bool operator()(Node* a, Node* b) {
		return a->real_cost > b->real_cost;
	}
};
extern std::priority_queue<Node*, std::vector<Node*>, cmp_a_star> pq_astar;//С����
extern std::priority_queue<Node*, std::vector<Node*>, cmp_dijkstra> pq_dijkstra;//С����

//methods declare
extern void iteration_astar_buy();
extern void iteration_astar_sell();
extern void iteration_dijkstra_buy();
extern void iteration_dijkstra_sell();
extern std::list<Node> search_Astar_buy(int* source, int* target);
extern std::list<Node> search_Astar_sell(int* source, int* target);
extern std::list<Node> search_dijkstra_buy(int* source, int* target);
extern std::list<Node> search_dijkstra_sell(int* source, int* target);


extern bool is_expand_or_set_sell[100][100]; //������չ�ϰ����ͼ
extern bool is_expand_or_set_buy[100][100]; 
extern std::list<Node> bench2bench_paths_buy[50][50];
//extern std::list<Node> bench2bench_paths_sell[50][50];
extern int bench_seq_type_dist[50];//ĳ��ŵĹ���̨��Ӧ������
extern void init_bench2bench_paths();


//bfs:�ĸ��������ҵ����ܵ���Ĺ���̨����
//roborts_can_go_bench_map��first�ǻ����˱�ţ�second�����ܵ���Ĺ���̨����

typedef struct
{
	int x;
	int y;
} coordinate;



//���ݽṹ
//extern char map_data[row1][col1]; // ��ͼ����
extern int bench_seq_map[row1][col1];//����̨��ŵĵ�ͼ
extern int robort_seq_map[4][2];//��һά�ǻ����˱�ţ��ڶ�ά�ǻ����˳�ʼ����
extern int bench_seq_dist[50][2]; //��һά�ǹ���̨��ţ��ڶ�ά�ǹ���̨����
extern std::map<int, std::vector<int>> roborts_can_go_bench_map;
extern bool roborts_can_go_bench_buy_array[4][50];
extern bool roborts_can_go_bench_sell_array[4][50];
extern std::vector<int> r_can_go_bench_vec;
extern std::vector<int> together_robort;
extern int init_bench_sum;

//����
extern int getBenchSeq(int i, int j);              // ��������õ�����̨��ŵĺ���
extern int getBenchXY(int bench_seq, int &X, int &Y);
extern void getRobortXY(int r_seq, int &X, int &Y); // ͨ����ŵõ����������꣬���ûش�
extern int getRobortSeq(int i, int j);             //ͨ������õ����������
extern void init_bench_robort_seq_map();
extern void ReadTxtData(std::string filename, char a[row1][col1]);
extern void coordinateTransf(int x0, int y0, double& x1, double& y1);//100*100����ת��Ϊ50*50
extern void BFS(int r_id, int x0, int y0);
extern void BFSTrave();

extern int obstacles[100][100];
extern std::list<Node_c> find_path(Point start, Point goal, bool buy_or_sell);
extern void add_into_heap(Node_c* node);
extern Node_c* pop_from_heap();
extern double heuristic_dist(Point p, Point goal);
extern Node_c* node_pointers[100][100];

extern std::list<Node_c> bench2bench_paths_sell[50][50];

extern int return_highter_robot(int robotID);
extern void out_of_way_robot(int r_id, int higher_r_id);
extern bool check_in_path(int c_robot_id, int o_robot_id);
extern double map_coor_axis_revised[100][100][2];

extern bool check_other_in_forward_path(int c_robot_id);