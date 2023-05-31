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



//----------调参区域-----------//
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

#define row1 100 //待读取文本文件中数据的行数
#define col1 100 //待读取文本文件中数据的列数

//----------文件变量声明
#define robotSradius  0.45  //机器人常态半径
#define robotBradius  0.53    //机器人载重半径
#define collGap   3.5//机器人判断碰撞最小间隔
#define avoidBorderMDist  1.7 //避障优先选择远离边缘的最小的离边缘的距离
#define RobotNum 4
#define PI 3.1415926535    //圆周率

//----------文件变量声明
#define actionNum 3//机器人达到一个工作台最多3个动作

//----------文件变量声明
#define DeskMaxNum 50 //工作台最大数量  

#define maxForwardSpeed  6
#define maxBackSpeed  2
#define maxAngleSpeed PI
#define ANGELACC  38.621 //不带负载的角加速度
#define ANGELACC1 20.032  //带负载角加速度
#define LINESPEEDACC 19.475  //不带负载线加速度
#define LINESPEEDACC1   14.385 //带负载线加速度
//findLine.c
#define BoderGridNum 15 //边线格子数目
#define borderLineWidth 5//道路最大宽度
#define controlRow 2   //控制点行数
extern int roadDir[RobotNum][BoderGridNum];//1为右，2为上，3为左，4为下
extern int realControlRow;
extern int hasBoderFlag[RobotNum];  //机器人周围是否存在障碍
extern int leftBoderGrid[RobotNum][BoderGridNum][2];  //左边边线格子
extern int rightBoderGrid[RobotNum][BoderGridNum][2];  //右边边线格子
extern double midPoint[RobotNum][BoderGridNum][2];  //中线左边
extern int leaveGridNum[RobotNum];          //路径剩余格子数目
extern double roadWidth[RobotNum][BoderGridNum]; //道路宽度
extern int realBoderGridNum[RobotNum];   //真实边线格子数目
extern double controlPoint[RobotNum][2];   //机器人的控制点
extern int temGridList[RobotNum][BoderGridNum][2];//路径点数组
extern  int findLineFlag[RobotNum][BoderGridNum];//1没丢线，0都丢线,2右边丢线，3上边丢，4左边丢，5下边丢


extern double aimDist[RobotNum];   //每次更新目标工作台后需要更新相应的目标工作台距离
extern int aimDesk[RobotNum];//期望工作台id
extern int aimAction[RobotNum][actionNum];//期望动作序列，0表示不动作，    1，卖；2买；3销毁

extern int hasBoderFlag[RobotNum];
extern double avoidBorderGap[RobotNum]; //改变避障优先远离边缘模式的缓冲带
extern int legSelectFlag[RobotNum];//每次避免障碍旋转优先旋转的leg方向；1表示期望旋转角为正；2表示为负；0表示未进行选择；
extern int collisionFlag[RobotNum][RobotNum];//碰撞标志位,二维数组表示
extern int collisionStateFlag[RobotNum];//碰撞状态标志位，0，没碰，1处于碰撞中
extern double avoidAngleGap[RobotNum];//改变避障普通模式的缓冲带
extern char map_data[100][100];//地图数据
extern char map_data_ob_expand_sell[100][100]; //扩展障碍物的地图
extern char map_data_ob_expand_buy[100][100];
extern double map_data_cost[100][100];
extern int frameID;  //当前帧序号
extern int hasMoney;//拥有金钱
extern int deskNum;//工作台数量
extern int robotNum;//机器人数量


extern double angleErr[RobotNum];;//角度偏差


extern int leftBoderGrid[RobotNum][BoderGridNum][2];//左边边界格子坐标
extern int rightBoderGrid[RobotNum][BoderGridNum][2];//机器人右边边界格子坐标
extern double midPoint[RobotNum][BoderGridNum][2];//机器人中线坐标
extern double lineSpeed[RobotNum];//当前线速度标量
extern int backFlag[RobotNum];//机器人是否后退
extern double robot2robotDist[RobotNum][RobotNum]; //机器人相互之间的距离
extern double setAngleSpeed[RobotNum];//设置角速度，逆时针为正
extern double setLineSpeed[RobotNum];//设置线速度，逆时针为正
extern double  lastSetSpeed[RobotNum];//上一次设置线速度
extern double  lastSetAngleSpeed[RobotNum];//上一次设置角速度
extern double expectSpeed[RobotNum]; //期望速度
extern double avoidExAngle[RobotNum];//避障环期望角
extern int gridCollFlag[RobotNum] ;//机器人格子碰撞标志位
extern double avGridExAngle[RobotNum];//退出碰撞状态期望角
extern double avoidGridGap[RobotNum];//改变避障普通模式的缓冲带
extern int hasBoderFlag[RobotNum];//机器人周围是否存在障碍物
extern double direction_Kp;
extern double direction_Kd;
extern double speed_Kp;
extern double speed_Kd;
extern int mapFlag;

typedef struct Desk
{
	int deskType[DeskMaxNum];//工作台类型
	double local[DeskMaxNum][2];//坐标
	int waitTime[DeskMaxNum];//剩余生产时间帧
	int materialState[DeskMaxNum];//原材料格状态
	int productState[DeskMaxNum];//产品格状态
}DeskData;
typedef struct Robot {
	int deskID[RobotNum];//所处工作台 ID
	int productType[RobotNum];//携带物品类型
	double local[RobotNum][2];//坐标
	double timeC[RobotNum];//时间系数
	double collC[RobotNum];//碰撞系数
	double angleSpeed[RobotNum];//角速度
	double lineSpeed[RobotNum][2];//线速度
	double towardAngle[RobotNum];//朝向
	double radius[RobotNum];//半径
	double angleAcc[RobotNum]; //角加速度
	double lineSpeedAcc[RobotNum];//线加速度
	int  grid[RobotNum][2];//机器人所处于格子坐标，左上角是0 0
}RobotData;
extern DeskData workDesk;//工作台结构变量
extern RobotData robot;//机器人结构变量

//全局路径A*算法 一个node代表一个点
class Node {
public:
	Node *parent; //指向上一个点 回溯起始节点
	Node *next; //指向下一个节点 寻路是使用
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

	int robotID;//机器人编号0-3
	int buyBenchSeq;//买工作台id
	int sellBenchSeq;//买工作台id
	int productType;//买卖产品类型
	int buyOrSell;//buy->1; sell->0; just go->2
	double predictFrameDoneTask;
	std::list<Node_c> path;

} WorkingRobot;


extern std::list<int> buy_task_seq_queue;//可买工作台序号队列
extern WorkingRobot working_robots_queue[4];//工作中机器人队列
extern std::map<int, std::list<int>> where_to_sell;
extern double product_profit_dict[8];
extern bool sell_benchs_in_queue[50][8];
extern int buy_benchs_in_queue[50];
extern bool free_robots[4];
extern std::map<int, std::list<int>> infor_working_benchs_dict;//存储对应类型的工作台
extern int bench_in_edge_num[DeskMaxNum]; //记录工作台的入边数量
extern int bench_out_edge_num[DeskMaxNum]; //记录工作台出边数量


//----------文件函数声明
extern void avoidOb();
extern bool dealColl(int robotid1, int robotid2);
extern void avoidBorder();
extern void avoidGrid(int robotid);
extern int judgeColl(int robotid);
extern bool judgeLaterColl(int robotid1, int robotid2, double laterLineSpeed1[2], double laterLineSpeed2[2]);
extern void filterCollSpeed(double time);
extern void findBorderLine(int robotID);

//----------文件函数声明
//----------文件函数声明
extern double calculateAngle(double*startPoint, double* endPoint);
extern double calculateDist(double* startPoint, double* endPoint);
extern double calculateL2(double x, double y);
extern double minF(double x, double y);
extern int minI(int x, int y);
extern int maxI(int x, int y);
extern double absF(double x);
extern int absI(int x);
//extern int sumIntArray(int a[]);

//----------文件函数声明

extern bool readMap();//初始化读取地图
extern bool readState();//每帧机器人和工作台状态读取
extern void outCMD(void);//输出控制指令
extern void expand_ob_sell(int i, int j); //用来扩展障碍物地图

extern void update_queue();
extern void update_queue_init(); //初始化的时候就接任务

extern void run_task();

extern double getPriority(int bench_type);

//----------文件函数声明
extern void getAim();



//----------文件函数声明
extern void speedControl();
extern void linSpeedControl(int robotid);//得到期望线速度
extern void Direction_Control_Error(int robotid);//得到期望角速度
extern bool judgeHasBorder(int robotid, int len,int startLine);
extern void avoidDireControl(int robotid);
extern void avoidSpeedControl(int robotid);
extern void calculateRobotState();//得到机器人相关的参数
extern double getAngleErr(double* point1, double* point2, double towardAngle);
extern double estiamteTime(double* point1, double* point2, double towardAngle);//估计机器人到达工作台时间
extern double estiamteTime2(double* point1, double* point2, double* point3, double towardAngle);
extern void estiamteSpeed(int robotid, double time, double speedAcc, double angleAcc, double* estimateV);
extern void getAimPoint(double* aimPoint, int robotID);
extern void avGridDirControl(int robotid);
extern void avGridSpeedControl(int robotid);
extern void judgeGridColl(int robotid);
extern void dealGridColl(int robotid, double* mayBePoint, int currentSearchNum);
extern void colledDeal();


extern Node* sourceNode; //出发点
extern Node* targetNode; //目标点
extern std::list<Node> res_path;
extern Node* all_node_pointers[100][100];


extern bool node_added[100][100];//表示点是否已经加入了优先级列表中
extern bool node_isSearched[100][100]; //表示点是否已经搜索过
extern char map_data_to_look_path[100][100];
//重载Node小根堆的比较运算符
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
extern std::priority_queue<Node*, std::vector<Node*>, cmp_a_star> pq_astar;//小根堆
extern std::priority_queue<Node*, std::vector<Node*>, cmp_dijkstra> pq_dijkstra;//小根堆

//methods declare
extern void iteration_astar_buy();
extern void iteration_astar_sell();
extern void iteration_dijkstra_buy();
extern void iteration_dijkstra_sell();
extern std::list<Node> search_Astar_buy(int* source, int* target);
extern std::list<Node> search_Astar_sell(int* source, int* target);
extern std::list<Node> search_dijkstra_buy(int* source, int* target);
extern std::list<Node> search_dijkstra_sell(int* source, int* target);


extern bool is_expand_or_set_sell[100][100]; //用来扩展障碍物地图
extern bool is_expand_or_set_buy[100][100]; 
extern std::list<Node> bench2bench_paths_buy[50][50];
//extern std::list<Node> bench2bench_paths_sell[50][50];
extern int bench_seq_type_dist[50];//某编号的工作台对应的类型
extern void init_bench2bench_paths();


//bfs:四个机器人找到它能到达的工作台序列
//roborts_can_go_bench_map，first是机器人编号，second是它能到达的工作台序列

typedef struct
{
	int x;
	int y;
} coordinate;



//数据结构
//extern char map_data[row1][col1]; // 地图数据
extern int bench_seq_map[row1][col1];//工作台编号的地图
extern int robort_seq_map[4][2];//第一维是机器人编号，第二维是机器人初始坐标
extern int bench_seq_dist[50][2]; //第一维是工作台编号，第二维是工作台坐标
extern std::map<int, std::vector<int>> roborts_can_go_bench_map;
extern bool roborts_can_go_bench_buy_array[4][50];
extern bool roborts_can_go_bench_sell_array[4][50];
extern std::vector<int> r_can_go_bench_vec;
extern std::vector<int> together_robort;
extern int init_bench_sum;

//函数
extern int getBenchSeq(int i, int j);              // 根据坐标得到工作台编号的函数
extern int getBenchXY(int bench_seq, int &X, int &Y);
extern void getRobortXY(int r_seq, int &X, int &Y); // 通过序号得到机器人坐标，引用回传
extern int getRobortSeq(int i, int j);             //通过坐标得到机器人序号
extern void init_bench_robort_seq_map();
extern void ReadTxtData(std::string filename, char a[row1][col1]);
extern void coordinateTransf(int x0, int y0, double& x1, double& y1);//100*100坐标转换为50*50
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