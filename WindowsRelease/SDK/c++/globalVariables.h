#pragma once
#include <list>
#include <map>
#include <math.h>
#include <iostream>
#include <algorithm>

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

extern double aimDist[RobotNum];   //ÿ�θ���Ŀ�깤��̨����Ҫ������Ӧ��Ŀ�깤��̨����
extern int aimDesk[RobotNum];//��������̨id
extern int aimAction[RobotNum][actionNum];//�����������У�0��ʾ��������    1������2��3����


extern double avoidBorderGap[RobotNum]; //�ı��������Զ���Եģʽ�Ļ����
extern int legSelectFlag[RobotNum];//ÿ�α����ϰ���ת������ת��leg����1��ʾ������ת��Ϊ����2��ʾΪ����0��ʾδ����ѡ��
extern int collisionFlag[RobotNum][RobotNum];//��ײ��־λ,��ά�����ʾ
extern int collisionStateFlag[RobotNum] ;//��ײ״̬��־λ��0��û����1������ײ��
extern double avoidAngleGap[RobotNum];//�ı������ͨģʽ�Ļ����
extern char map_data[100][100];//��ͼ����
extern int frameID;  //��ǰ֡���
extern int hasMoney;//ӵ�н�Ǯ
extern int deskNum;//����̨����
extern int robotNum;//����������


extern double angleErr[RobotNum];;//�Ƕ�ƫ��


extern double lineSpeed[RobotNum];//��ǰ���ٶȱ���
extern int backFlag[RobotNum];//�������Ƿ����
extern double robot2robotDist[RobotNum][RobotNum]; //�������໥֮��ľ���
extern double setAngleSpeed[RobotNum];//���ý��ٶȣ���ʱ��Ϊ��
extern double setLineSpeed[RobotNum];//�������ٶȣ���ʱ��Ϊ��
extern double  lastSetSpeed[RobotNum];//��һ���������ٶ�
extern double  lastSetAngleSpeed[RobotNum];//��һ�����ý��ٶ�
extern double expectSpeed[RobotNum]; //�����ٶ�
extern double avoidExAngle[RobotNum];//���ϻ�������
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
}RobotData;
extern DeskData workDesk;//����̨�ṹ����
extern RobotData robot;//�����˽ṹ����


typedef struct WorkingRobot {

	int robotID;//�����˱��0-3
	int buyBenchSeq;//����̨id
	int sellBenchSeq;//����̨id
	int productType;//������Ʒ����
	bool buyOrSell;//buy->true; sell->false
	double predictFrameDoneTask;

} WorkingRobot;


extern std::list<int> buyTaskSeqQueue;//������̨��Ŷ���
extern std::list<WorkingRobot> workingRobotsQueue;//�����л����˶���
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
extern int judgeColl(int robotid);
extern bool judgeLaterColl(int robotid1, int robotid2, double laterLineSpeed1[2], double laterLineSpeed2[2]);
extern void filterCollSpeed(double time);

//----------�ļ���������
extern double calculateAngle(double*startPoint, double* endPoint);
extern double calculateDist(double* startPoint, double* endPoint);
extern double calculateL2(double x, double y);
extern double minF(double x, double y);
extern double absF(double x);
//extern int sumIntArray(int a[]);

//----------�ļ���������
extern bool readMap();//��ʼ����ȡ��ͼ
extern bool readState();//ÿ֡�����˺͹���̨״̬��ȡ
extern void outCMD(void);//�������ָ��

extern void update_queue();

extern void run_task();

extern double getPriority(int bench_type);

//----------�ļ���������
extern void getAim();



//----------�ļ���������
extern void speedControl();
extern void linSpeedControl(int robotid);//�õ��������ٶ�
extern void Direction_Control_Error(int robotid);//�õ��������ٶ�
extern void avoidDireControl(int robotid);
extern void avoidSpeedControl(int robotid);
extern void calculateRobotState();//�õ���������صĲ���
extern double getAngleErr(double* point1, double* point2, double towardAngle);
extern double estiamteTime(double* point1, double* point2, double towardAngle);//���ƻ����˵��﹤��̨ʱ��
extern double estiamteTime2(double* point1, double* point2, double* point3, double towardAngle);
extern void estiamteSpeed(int robotid, double time, double speedAcc, double angleAcc, double* estimateV);

