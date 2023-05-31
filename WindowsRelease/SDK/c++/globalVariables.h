#pragma once
#include <list>
#include <map>
#include <math.h>
#include <iostream>
#include <algorithm>

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

extern double aimDist[RobotNum];   //每次更新目标工作台后需要更新相应的目标工作台距离
extern int aimDesk[RobotNum];//期望工作台id
extern int aimAction[RobotNum][actionNum];//期望动作序列，0表示不动作，    1，卖；2买；3销毁


extern double avoidBorderGap[RobotNum]; //改变避障优先远离边缘模式的缓冲带
extern int legSelectFlag[RobotNum];//每次避免障碍旋转优先旋转的leg方向；1表示期望旋转角为正；2表示为负；0表示未进行选择；
extern int collisionFlag[RobotNum][RobotNum];//碰撞标志位,二维数组表示
extern int collisionStateFlag[RobotNum] ;//碰撞状态标志位，0，没碰，1处于碰撞中
extern double avoidAngleGap[RobotNum];//改变避障普通模式的缓冲带
extern char map_data[100][100];//地图数据
extern int frameID;  //当前帧序号
extern int hasMoney;//拥有金钱
extern int deskNum;//工作台数量
extern int robotNum;//机器人数量


extern double angleErr[RobotNum];;//角度偏差


extern double lineSpeed[RobotNum];//当前线速度标量
extern int backFlag[RobotNum];//机器人是否后退
extern double robot2robotDist[RobotNum][RobotNum]; //机器人相互之间的距离
extern double setAngleSpeed[RobotNum];//设置角速度，逆时针为正
extern double setLineSpeed[RobotNum];//设置线速度，逆时针为正
extern double  lastSetSpeed[RobotNum];//上一次设置线速度
extern double  lastSetAngleSpeed[RobotNum];//上一次设置角速度
extern double expectSpeed[RobotNum]; //期望速度
extern double avoidExAngle[RobotNum];//避障环期望角
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
}RobotData;
extern DeskData workDesk;//工作台结构变量
extern RobotData robot;//机器人结构变量


typedef struct WorkingRobot {

	int robotID;//机器人编号0-3
	int buyBenchSeq;//买工作台id
	int sellBenchSeq;//买工作台id
	int productType;//买卖产品类型
	bool buyOrSell;//buy->true; sell->false
	double predictFrameDoneTask;

} WorkingRobot;


extern std::list<int> buyTaskSeqQueue;//可买工作台序号队列
extern std::list<WorkingRobot> workingRobotsQueue;//工作中机器人队列
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
extern int judgeColl(int robotid);
extern bool judgeLaterColl(int robotid1, int robotid2, double laterLineSpeed1[2], double laterLineSpeed2[2]);
extern void filterCollSpeed(double time);

//----------文件函数声明
extern double calculateAngle(double*startPoint, double* endPoint);
extern double calculateDist(double* startPoint, double* endPoint);
extern double calculateL2(double x, double y);
extern double minF(double x, double y);
extern double absF(double x);
//extern int sumIntArray(int a[]);

//----------文件函数声明
extern bool readMap();//初始化读取地图
extern bool readState();//每帧机器人和工作台状态读取
extern void outCMD(void);//输出控制指令

extern void update_queue();

extern void run_task();

extern double getPriority(int bench_type);

//----------文件函数声明
extern void getAim();



//----------文件函数声明
extern void speedControl();
extern void linSpeedControl(int robotid);//得到期望线速度
extern void Direction_Control_Error(int robotid);//得到期望角速度
extern void avoidDireControl(int robotid);
extern void avoidSpeedControl(int robotid);
extern void calculateRobotState();//得到机器人相关的参数
extern double getAngleErr(double* point1, double* point2, double towardAngle);
extern double estiamteTime(double* point1, double* point2, double towardAngle);//估计机器人到达工作台时间
extern double estiamteTime2(double* point1, double* point2, double* point3, double towardAngle);
extern void estiamteSpeed(int robotid, double time, double speedAcc, double angleAcc, double* estimateV);

