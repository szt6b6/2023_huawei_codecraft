#include"globalVariables.h"
double angleErr[RobotNum] = { 0 }; //����ƫ��
double direction_Kp = 25;
double direction_Kd = 5.2;
double speed_Kp = 50;
double speed_Kd = 10.2;
double expectSpeed[RobotNum] = { 0 };
double avoidExAngle[RobotNum] = { 8,8,8,8 };
double setAngleSpeed[RobotNum] = { 0 };
double setLineSpeed[RobotNum] = { 0 };
double  lastSetSpeed[RobotNum];//��һ���������ٶ�
double  lastSetAngleSpeed[RobotNum];//��һ�����ý��ٶ�
int backFlag[RobotNum] = { 0 };
double robot2robotDist[RobotNum][RobotNum] = { 0 };
double lineSpeed[RobotNum];
int mapFlag = 0;
double avoid1 = 2;
double avoid2 = 3;
double avoid3 = 1;
double avoid4 = 1;
//ͼ��1.5��3.5
void judge_backFlag(int robotid)
{
	if (robot.towardAngle[robotid] <= PI / 2 && robot.towardAngle[robotid] >= 0)
	{
		if (robot.lineSpeed[robotid][0] <= 0 && robot.lineSpeed[robotid][1] <= 0)
			backFlag[robotid] = 1;
		else
			backFlag[robotid] = 0;
	}
	else if (robot.towardAngle[robotid] >= PI / 2) {
		if (robot.lineSpeed[robotid][0] >= 0 && robot.lineSpeed[robotid][1] <= 0)
			backFlag[robotid] = 1;
		else
			backFlag[robotid] = 0;
	}
	else if (robot.towardAngle[robotid] >= -PI / 2 && robot.towardAngle[robotid] <= 0) {
		if (robot.lineSpeed[robotid][0] <= 0 && robot.lineSpeed[robotid][1] >= 0)
			backFlag[robotid] = 1;
		else
			backFlag[robotid] = 0;
	}
	else if (robot.towardAngle[robotid] <= -PI / 2) {
		if (robot.lineSpeed[robotid][0] >= 0 && robot.lineSpeed[robotid][1] >= 0)
			backFlag[robotid] = 1;
		else
			backFlag[robotid] = 0;
	}
}
/**
 * FunctionName: Direction_Control_Error()
 * Input:
 * Return: ��
 * Description: �����˲������㺯��
 * Author: ����,2023/3/12
 */
void calculateRobotState()
{
	for (int i = 0; i < RobotNum; i++)
	{
		//����״̬�ж�
		judge_backFlag(i);
		lineSpeed[i] = calculateL2(robot.lineSpeed[i][0], robot.lineSpeed[i][1]);
		if (backFlag[i] == 1)
			lineSpeed[i] = -lineSpeed[i];
		//�������໥�������
		for (int j = 0; j < RobotNum; j++)
		{
			if (i == j)
				robot2robotDist[i][j] = 0;
			else if (i > j)
				robot2robotDist[i][j] = robot2robotDist[j][i];
			else
				robot2robotDist[i][j] = calculateDist(robot.local[i], robot.local[j]);
		}
		//�����˰뾶���ٶȸ�ֵ
		if (robot.productType[i] > 0)
		{
			robot.radius[i] = robotBradius;
			robot.lineSpeedAcc[i] = LINESPEEDACC1;
			robot.angleAcc[i] = ANGELACC1;
		}
		else
		{
			robot.radius[i] = robotSradius;
			robot.lineSpeedAcc[i] = LINESPEEDACC;
			robot.angleAcc[i] = ANGELACC;
		}
	}
}
/**
 * FunctionName: avoidDireControl()
 * Input: ���������
 * Return: ��
 * Description: ���Ϸ�����ƺ���
 * Author: ����,2023/3/17
 */

void avoidDireControl(int robotid)
{
	static double error1[RobotNum] = { 0 }, error2[RobotNum] = { 0 };
	int sum = 0;
	//avoidExAngle[robotid]==8���������Ϊ����ײֻ�ı�һ���Ƕ�
	for (int i = 0; i < RobotNum; i++)
		sum += collisionFlag[robotid][i];
	if (sum == 0 || avoidExAngle[robotid] == 8)
	{
		error1[robotid] = 0;
		error2[robotid] = 0;
	}
	else {
		error2[robotid] = error1[robotid]; //�����ϴ�ƫ��
		error1[robotid] = avoidExAngle[robotid]; //�������ƫ��
		setAngleSpeed[robotid] = -(direction_Kp * error1[robotid] + direction_Kd * (error1[robotid] - error2[robotid]));
	}
}
/**
 * FunctionName: avoidSpeedControl
 * Input:������id
 * Return:
 * Description:��ײ�ٶȿ���
 * Author: ����,2023/3/18
 */
void avoidSpeedControl(int robotid)
{
	//��������id
	int collID = judgeColl(robotid);
	if (collID > 0)
	{
		//
		if (mapFlag == 1)
		{
			//�����������
			//Խ���ٶȹ淶
			if (setLineSpeed[collID] > maxForwardSpeed)setLineSpeed[collID] = maxForwardSpeed;
			//С������ٶ�ֱ���˳�
			if (setLineSpeed[collID] > avoid1)
			{
				setLineSpeed[collID] = minF(setLineSpeed[collID], maxForwardSpeed - avoid3 * maxForwardSpeed / collGap * (robot.radius[robotid] + robot.radius[collID] + collGap - robot2robotDist[robotid][collID]));
				if (setLineSpeed[collID] < avoid1)setLineSpeed[collID] = avoid1;
			}
			//��ײ�����ٶȿ���
			if (1) {
				setLineSpeed[robotid] = maxForwardSpeed;
				//���ٶȵ�����Ӧ�ú�angleErr�йأ����¸��˸���������ת��ģ�ͱȽϿ�ѧ
				if ((avoidExAngle[robotid] > PI / 20.0 || avoidExAngle[robotid] < -PI / 20.0) && robot2robotDist[robotid][collID] < robot.radius[robotid] + robot.radius[collID] + 1)
					//����Ŀ����ٶ�Ϊ��������ý�һ���Ż�
				{
					setLineSpeed[robotid] = minF(setLineSpeed[robotid], maxForwardSpeed - avoid4 * maxForwardSpeed / collGap * (robot.radius[robotid] + robot.radius[collID] + collGap - robot2robotDist[robotid][collID]));
					if (setLineSpeed[robotid] < avoid2)setLineSpeed[robotid] = avoid2;
				}
				if ((avoidExAngle[robotid] >= PI / 2 || avoidExAngle[robotid] <= -PI / 2))
					setLineSpeed[robotid] = 0;
			}
		}
		else if (mapFlag == 2)
		{
			//�����������
			//Խ���ٶȹ淶
			if (setLineSpeed[collID] > maxForwardSpeed)setLineSpeed[collID] = maxForwardSpeed;
			//С������ٶ�ֱ���˳�
			if (setLineSpeed[collID] > avoid1)
			{
				setLineSpeed[collID] = minF(setLineSpeed[collID], maxForwardSpeed - avoid3 * maxForwardSpeed / collGap * (robot.radius[robotid] + robot.radius[collID] + collGap - robot2robotDist[robotid][collID]));
				if (setLineSpeed[collID] < avoid1)setLineSpeed[collID] = avoid1;
			}
			//��ײ�����ٶȿ���
			if (1) {
				setLineSpeed[robotid] = maxForwardSpeed;
				//���ٶȵ�����Ӧ�ú�angleErr�йأ����¸��˸���������ת��ģ�ͱȽϿ�ѧ
				if ((avoidExAngle[robotid] > PI / 20.0 || avoidExAngle[robotid] < -PI / 20.0) && robot2robotDist[robotid][collID] < robot.radius[robotid] + robot.radius[collID] + 1)
					//����Ŀ����ٶ�Ϊ��������ý�һ���Ż�
				{
					setLineSpeed[robotid] = minF(setLineSpeed[robotid], maxForwardSpeed - avoid4 * maxForwardSpeed / collGap * (robot.radius[robotid] + robot.radius[collID] + collGap - robot2robotDist[robotid][collID]));
					if (setLineSpeed[robotid] < avoid2)setLineSpeed[robotid] = avoid2;
				}
				if ((avoidExAngle[robotid] >= PI / 2 || avoidExAngle[robotid] <= -PI / 2))
					setLineSpeed[robotid] = 0;
			}
		}
		else if (mapFlag == 3)
		{
			//�����������
			//Խ���ٶȹ淶
			if (setLineSpeed[collID] > maxForwardSpeed)setLineSpeed[collID] = maxForwardSpeed;
			//С������ٶ�ֱ���˳�
			if (setLineSpeed[collID] > 1.5)
			{
				setLineSpeed[collID] = minF(setLineSpeed[collID], maxForwardSpeed - avoid3 * maxForwardSpeed / collGap * (robot.radius[robotid] + robot.radius[collID] + collGap - robot2robotDist[robotid][collID]));
				if (setLineSpeed[collID] < 1.5)setLineSpeed[collID] = 1.5;
			}
			//��ײ�����ٶȿ���
			if (1) {
				setLineSpeed[robotid] = maxForwardSpeed;
				//���ٶȵ�����Ӧ�ú�angleErr�йأ����¸��˸���������ת��ģ�ͱȽϿ�ѧ
				if ((avoidExAngle[robotid] > PI / 20.0 || avoidExAngle[robotid] < -PI / 20.0) && robot2robotDist[robotid][collID] < robot.radius[robotid] + robot.radius[collID] + 1)
					//����Ŀ����ٶ�Ϊ��������ý�һ���Ż�
				{
					setLineSpeed[robotid] = minF(setLineSpeed[robotid], maxForwardSpeed - avoid4 * maxForwardSpeed / collGap * (robot.radius[robotid] + robot.radius[collID] + collGap - robot2robotDist[robotid][collID]));
					if (setLineSpeed[robotid] < 3.5)setLineSpeed[robotid] = 3.5;
				}
				if ((avoidExAngle[robotid] >= PI / 2 || avoidExAngle[robotid] <= -PI / 2))
					setLineSpeed[robotid] = 0;
			}
		}
		else
		{
			//�����������
			//Խ���ٶȹ淶
			if (setLineSpeed[collID] > maxForwardSpeed)setLineSpeed[collID] = maxForwardSpeed;
			//С������ٶ�ֱ���˳�
			if (setLineSpeed[collID] > avoid1)
			{
				setLineSpeed[collID] = minF(setLineSpeed[collID], maxForwardSpeed - avoid3 * maxForwardSpeed / collGap * (robot.radius[robotid] + robot.radius[collID] + collGap - robot2robotDist[robotid][collID]));
				if (setLineSpeed[collID] < avoid1)setLineSpeed[collID] = avoid1;
			}
			//��ײ�����ٶȿ���
			if (1) {
				setLineSpeed[robotid] = maxForwardSpeed;
				//���ٶȵ�����Ӧ�ú�angleErr�йأ����¸��˸���������ת��ģ�ͱȽϿ�ѧ
				if ((avoidExAngle[robotid] > PI / 20.0 || avoidExAngle[robotid] < -PI / 20.0) && robot2robotDist[robotid][collID] < robot.radius[robotid] + robot.radius[collID] + 1)
					//����Ŀ����ٶ�Ϊ��������ý�һ���Ż�
				{
					setLineSpeed[robotid] = minF(setLineSpeed[robotid], maxForwardSpeed - avoid4 * maxForwardSpeed / collGap * (robot.radius[robotid] + robot.radius[collID] + collGap - robot2robotDist[robotid][collID]));
					if (setLineSpeed[robotid] < avoid2)setLineSpeed[robotid] = avoid2;
				}
				if ((avoidExAngle[robotid] >= PI / 2 || avoidExAngle[robotid] <= -PI / 2))
					setLineSpeed[robotid] = 0;
			}
		}
	}

}
/**
 * FunctionName: Direction_Control_Error()
 * Input: ���������
 * Return: ��
 * Description: ������ƺ���
 * Author: ����,2023/3/11
 */
void Direction_Control_Error(int robotid)
{
	static double error1[RobotNum] = { 0 }, error2[RobotNum] = { 0 };
	int  deskid = aimDesk[robotid];
	angleErr[robotid] = getAngleErr(robot.local[robotid], workDesk.local[deskid], robot.towardAngle[robotid]);
	/*if (angleErr[robotid] >= PI || angleErr[robotid] <= -PI)angleErr[robotid] = -angleErr[robotid];*/
	error2[robotid] = error1[robotid]; //�����ϴ�ƫ��
	error1[robotid] = angleErr[robotid]; //�������ƫ��
	setAngleSpeed[robotid] = -(direction_Kp * error1[robotid] + direction_Kd * (error1[robotid] - error2[robotid]));
	//����Ŀ�����û�и���Ŀ�깤��̨����0���ٶ�
	if (aimDist[robotid] < 0.4)
	{
		error2[robotid] = 0; error1[robotid] = 0; setAngleSpeed[robotid] = 0;
	}
	////���ٶ�������
	//if (setAngleSpeed[robotid] <= -maxAngleSpeed)setAngleSpeed[robotid] = -maxAngleSpeed;
	//if (setAngleSpeed[robotid] >= maxAngleSpeed)setAngleSpeed[robotid] = maxAngleSpeed;
}
/**
 * FunctionName: linSpeedControl()
 * Input: robotid
 * Return: ��
 * Description: ���ٶȿ��ƺ���
 * Author: ����,2023/3/12
 */
void linSpeedControl(int robotid)
{
	if (deskNum == 43)
		mapFlag = 1;
	else if (deskNum == 25)
		mapFlag = 2;
	else if (deskNum == 50)
		mapFlag = 3;
	else
		mapFlag = 4;
	//static double speed_error_t1[RobotNum] = { 0 };			//�ٶ�ƫ��e(t-1)
	//static double speed_error [RobotNum] = { 0 };
	//expectSpeed[robotid] = 6.02;
	////���ٶȵ�����Ӧ�ú�angleErr�йأ����¸��˸���������ת��ģ�ͱȽϿ�ѧ
	//if ((angleErr[robotid] > PI / 50.0 || angleErr[robotid] < -PI / 50.0) && aimDist[robotid] < 2.5)
	//	//����Ŀ����ٶ�Ϊ��������ý�һ���Ż�
	//{
	//	expectSpeed[robotid] = aimDist[robotid] * 6 - 1.8* angleErr[robotid] * angleErr[robotid];
	//	if (expectSpeed[robotid] < 0)expectSpeed[robotid] = 0;
	//}
	//if ((angleErr[robotid] >= PI / 2 || angleErr[robotid] <= -PI / 2 || aimDist[robotid] < 0.4))
	//	expectSpeed[robotid] = 0;
	////�����ٶ�ƫ��
	//speed_error_t1[robotid] = speed_error[robotid];
	//speed_error[robotid] = expectSpeed[robotid] -lineSpeed[robotid];   //����ƫ��
	//setLineSpeed[robotid] = (speed_Kp * speed_error[robotid] + speed_Kd * (speed_error[robotid] - speed_error_t1[robotid]));
	//if (setLineSpeed[robotid] < 0)setLineSpeed[robotid] = 0;
	//������
	if (1)
	{
		setLineSpeed[robotid] = 6.02;
		//���ٶȵ�����Ӧ�ú�angleErr�йأ����¸��˸���������ת��ģ�ͱȽϿ�ѧ
		if ((angleErr[robotid] > PI / 60.0 || angleErr[robotid] < -PI / 60.0) && aimDist[robotid] < 2.5 && mapFlag == 1)
			//����Ŀ����ٶ�Ϊ��������ý�һ���Ż�
		{
			setLineSpeed[robotid] = aimDist[robotid] * 6 - 2.25 * angleErr[robotid] * angleErr[robotid];
			if (setLineSpeed[robotid] < 0)setLineSpeed[robotid] = 0;
		}
		else if ((angleErr[robotid] > PI / 50.0 || angleErr[robotid] < -PI / 50.0) && aimDist[robotid] < 2.5 && mapFlag == 2)
			//����Ŀ����ٶ�Ϊ��������ý�һ���Ż�
		{
			setLineSpeed[robotid] = aimDist[robotid] * 6 - 2.55 * angleErr[robotid] * angleErr[robotid];
			if (setLineSpeed[robotid] < 0)setLineSpeed[robotid] = 0;
		}
		else if ((angleErr[robotid] > PI / 60.0 || angleErr[robotid] < -PI / 60.0) && aimDist[robotid] < 2.25 && mapFlag == 3)
			//����Ŀ����ٶ�Ϊ��������ý�һ���Ż�
		{
			setLineSpeed[robotid] = aimDist[robotid] * 6 - 2.75 * angleErr[robotid] * angleErr[robotid];
			if (setLineSpeed[robotid] < 0)setLineSpeed[robotid] = 0;
		}
		else if ((angleErr[robotid] > PI / 60.0 || angleErr[robotid] < -PI / 60.0) && aimDist[robotid] < 2.25 && mapFlag == 4)
			//����Ŀ����ٶ�Ϊ��������ý�һ���Ż�
		{
			setLineSpeed[robotid] = aimDist[robotid] * 6 - 2.65 * angleErr[robotid] * angleErr[robotid];
			if (setLineSpeed[robotid] < 0)setLineSpeed[robotid] = 0;
		}
		//�����ŵ�ͼ�ֱ���0.42,0.472,0.485,0.472
		//���ŵ�ͼsetline:0.5,0],0,0],1.5,0]2,2]
		if (mapFlag == 1)
		{
			if ((absF(angleErr[robotid]) >= PI * 0.42&& absF(angleErr[robotid]) <= PI * 0.75) || aimDist[robotid] < 0.4)
				/*setLineSpeed[robotid] =0.95*(6 - 24 / PI/PI * (angleErr[robotid] - PI / 2)* (angleErr[robotid] - PI / 2));*/
				setLineSpeed[robotid] = 0.5;
			else if (absF(angleErr[robotid]) > PI * 0.75)
				setLineSpeed[robotid] = 0;
		}
		else if (mapFlag == 2)
		{
			if ((absF(angleErr[robotid]) >= PI * 0.472 && absF(angleErr[robotid]) <= PI * 0.75) || aimDist[robotid] < 0.4)
				setLineSpeed[robotid] = 0;
			else if (absF(angleErr[robotid]) > PI * 0.75)
				setLineSpeed[robotid] = 0;
		}
		else if (mapFlag == 3)
		{
			if ((absF(angleErr[robotid]) >= PI * 0.485 && absF(angleErr[robotid]) <= PI * 0.75) || aimDist[robotid] < 0.4)
				setLineSpeed[robotid] = 1.5;
			else if (absF(angleErr[robotid]) > PI * 0.75)
				setLineSpeed[robotid] = 0;
		}
		else
		{
			if ((absF(angleErr[robotid]) >= PI * 0.472 && absF(angleErr[robotid]) <= PI * 0.75) || aimDist[robotid] < 0.4)
				setLineSpeed[robotid] = 2;
			else if (absF(angleErr[robotid]) > PI * 0.75)
				setLineSpeed[robotid] = 2;
		}
	}
	else
		setLineSpeed[robotid] = 0;
	////�ٶ���ֵ����
	//if (setLineSpeed[robotid] > maxForwardSpeed)setLineSpeed[robotid] = maxForwardSpeed;
	//if (setLineSpeed[robotid] < -maxBackSpeed)setLineSpeed[robotid] = -maxBackSpeed;
}

/**
 * FunctionName: estiamteTime()
 * Input:�����˵�ǰ���ꡣ����̨���ꡣ�����˵�ǰ�ٶ�����
 * Return: ��
 * Description: ���ƴӵ�ǰλ�õ�Ŀ�깤��̨��ʱ��
 * Author: ����,2023/3/12
 */
double estiamteTime(double* point1, double* point2, double towardAngle)
{
	//������������ο��˶�ʱ��Ԥ��ģ��ͼ
	double point0[2] = { 0 };
	double theta1, theta;
	double L1; double L2; double L3;
	double t; double t2; double t1;
	double x0; double y0;
	double x1 = point1[0]; double y1 = point1[1];
	double x2 = point2[0]; double y2 = point2[1];
	double R = 6 / PI;
	double angleErr1 = getAngleErr(point1, point2, towardAngle);
	//���ü��ι�ϵ���Բ��
	if (point2[0] >= point1[0]) {
		x0 = x1 + R * sin(towardAngle);
	}
	else {
		x0 = x1 - R * sin(towardAngle);
	}
	if (point2[1] <= point1[1]) {
		y0 = y1 + R * cos(towardAngle);
	}
	else {
		y0 = y1 - R * cos(towardAngle);
	}
	point0[0] = x0; point0[1] = y0;
	//Բ�ĵ�Ŀ������
	L1 = calculateDist(point0, point2);
	//��ʼ�㵽Ŀ�����
	L2 = calculateDist(point2, point1);
	L3 = sqrt(absF(L1 * L1 - R * R));
	theta1 = acos(R / L1);//[0,pi]
	theta = -theta1 + (L1 * L1 + R * R - L2 * L2) / (2 * L1 * R);
	if (theta < 0)theta = -theta;
	t1 = theta / PI;
	t2 = L3 / 6;
	t = t1 + t2 + 0.4;
	double tExp = L2 / 6 + angleErr1 / PI + 0.45;
	double errExp = absF(tExp);
	//��ģ���������ʱ��;���ʱ����бȽ�Լ��
	if (absF(errExp - tExp) / tExp > 0.2)
		return tExp;
	return t;
}
/**
 * FunctionName: estiamteTime2()
 * Input:�����˵�ǰ���ꡣ����̨���꣬������̨���ꡣ�����˷���
 * Return: ��
 * Description: ���ƴӵ�ǰλ�õ�Ŀ�깤��̨��ʱ��
 * Author: ����,2023/3/12
 */
 //double estiamteTime2(double* point1, double* point2, double* point3, double towardAngle)
 //{
 //	double theta0 = 0;
 //	double* theta=&theta0; double tsell, tbuy;
 //	tbuy = estiamteTime(point1, point2, towardAngle,theta);
 //	if(point1[0]< point2[0])
 //		tsell= estiamteTime(point1, point2, towardAngle- theta0, theta);
 //	else
 //		tsell = estiamteTime(point2, point3, towardAngle + *theta, theta);
 //	return tsell + tbuy;
 //}
 /**
  * FunctionName: getAngleErr()
  * Input: ��ʼ���Ŀ������꣬�ٶȷ���
  * Return: ƫ���[0��PI]
  * Description: ƫ��Ǽ���
  * Author: ����,2023/3/16
  */
double getAngleErr(double* point1, double* point2, double towardAngle)
{
	double p2top1Angle = calculateAngle(point1, point2);
	double angleErr1 = towardAngle - p2top1Angle; //��������Ŀ���ǶȲ�
		//���ڻ��Ƚ���ͻ����Ҫ��ƫ��Ƕ�������
	if (angleErr1 >= PI)
	{
		angleErr1 = 2 * PI - angleErr1;
		angleErr1 = -angleErr1;
	}
	if (angleErr1 <= -PI)
	{
		angleErr1 = 2 * PI + angleErr1;
	}
	return angleErr1;
}
/**
 * FunctionName: estiamteSpeed()
 * Input:��������š�ʱ�䡣���ٶȣ�����һ��ʱ�����ٶ�����
 * Return: ��
 * Description: ���ƴӵ�ǰλ�õ�Ŀ�깤��̨��ʱ��
 * Author: ����,2023/3/12
 */
void estiamteSpeed(int robotid, double time, double speedAcc, double angleAcc, double* estimateV)
{
	double estimateAngle = 0;
	if (setLineSpeed[robotid] <= lineSpeed[robotid])speedAcc = -speedAcc;
	if (setAngleSpeed[robotid] <= robot.angleSpeed[robotid])angleAcc = -angleAcc;
	double estimateLineSpeed = lineSpeed[robotid] + speedAcc * time;
	if (estimateLineSpeed > maxForwardSpeed)estimateLineSpeed = maxForwardSpeed;
	if (estimateLineSpeed < -maxBackSpeed)estimateLineSpeed = -maxBackSpeed;
	if (robot.angleSpeed[robotid] > PI)robot.angleSpeed[robotid] = PI;
	if (robot.angleSpeed[robotid] < -PI)robot.angleSpeed[robotid] = -PI;
	if (robot.angleSpeed[robotid] + angleAcc * time > PI)
	{
		//���ٽ׶�
		double t1 = (PI - robot.angleSpeed[robotid]) / angleAcc;
		double t2 = time - t1;
		estimateAngle = robot.towardAngle[robotid] + (robot.towardAngle[robotid] + PI) / 2 * t1 + PI * t2;
	}
	else if (robot.angleSpeed[robotid] + angleAcc * time < -PI)
	{
		//���ٽ׶�
		double t1 = (PI + robot.angleSpeed[robotid]) / angleAcc;
		double t2 = time - t1;
		estimateAngle = robot.towardAngle[robotid] + (robot.towardAngle[robotid] - PI) / 2 * t1 - PI * t2;
	}
	else
		estimateAngle = robot.towardAngle[robotid] + robot.angleSpeed[robotid] * time + 0.5 * angleAcc * time * time;
	estimateV[0] = estimateLineSpeed * cos(estimateAngle); estimateV[1] = estimateLineSpeed * sin(estimateAngle);
}
/**
 * FunctionName: speedControl()
 * Input: ��
 * Return: ��
 * Description: ��������ٶȿ��ƺ���
 * Author: ����,2023/3/12
 */
void speedControl() {
	for (int i = 0; i < RobotNum; i++)
	{
		//���¹���̨��ز���
		aimDist[i] = calculateDist(robot.local[i], workDesk.local[aimDesk[i]]);
		Direction_Control_Error(i);
		linSpeedControl(i);
	}
}
