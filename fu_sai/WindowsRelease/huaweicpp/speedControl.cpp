#include"globalVariables.h"
double angleErr[RobotNum] = { 0 }; //����ƫ��
double direction_Kp = 20;
double direction_Kd = 60.2;
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
int gridCollFlag[RobotNum] = { 0 };
int nearFlag[RobotNum] = { 0 };
double avGridExAngle[RobotNum];
double aimPointArray[RobotNum][2] = { 0 };
int mapFlag = 0;
double avoid1 = 2;
double avoid2 = 3;
double avoid3 = 1;
double avoid4 = 1;
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
 * FunctionName: calculateRobotState()
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
		//��������������x,y�����ͼ���
		robot.grid[i][1] = int(robot.local[i][0] / 0.5);
		robot.grid[i][0] = 99 - int(robot.local[i][1] / 0.5);
		//�õ��Ի��������ڸ�������Ϊ���ģ�2.5x2.5��Χ���Ƿ�����ϰ�
		hasBoderFlag[i] = judgeHasBorder(i, 2, 0);
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
				setLineSpeed[robotid] = avoid2;
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
void Direction_Control_Error(int robotid, double* aimPoint)
{
	static double error1[RobotNum] = { 0 }, error2[RobotNum] = { 0 };
	//int  deskid = aimDesk[robotid];
	angleErr[robotid] = getAngleErr(robot.local[robotid], aimPoint, robot.towardAngle[robotid]);
	error2[robotid] = error1[robotid]; //�����ϴ�ƫ��
	error1[robotid] = angleErr[robotid]; //�������ƫ��
	setAngleSpeed[robotid] = -(direction_Kp * error1[robotid] + direction_Kd * (error1[robotid] - error2[robotid]));
	//����Ŀ�����û�и���Ŀ�깤��̨����0�ٶ�
	if (aimDist[robotid] < 0.4 && leaveGridNum[robotid] <= 4)
	{
		error2[robotid] = 0; error1[robotid] = 0; setAngleSpeed[robotid] = 0;
	}
	//���ٶ�������
	if (setAngleSpeed[robotid] <= -maxAngleSpeed)setAngleSpeed[robotid] = -maxAngleSpeed;
	if (setAngleSpeed[robotid] >= maxAngleSpeed)setAngleSpeed[robotid] = maxAngleSpeed;
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
	if (deskNum)
		mapFlag = 1;
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
		setLineSpeed[robotid] = maxForwardSpeed;
		//���ٶȵ�����Ӧ�ú�angleErr�йأ����¸��˸���������ת��ģ�ͱȽϿ�ѧ
		if ((aimDist[robotid] <= 2.4))
			//����Ŀ����ٶ�Ϊ��������ý�һ���Ż�
		{
			//���빤��̨�ܽ�
			if (leaveGridNum[robotid] <= 6)
			{
				setLineSpeed[robotid] = aimDist[robotid] * 4 - 3 * angleErr[robotid] * angleErr[robotid];
			}
			//ƽʱ�ٶȿ���.��ͼ3,5
			else
				//setLineSpeed[robotid] = cos(angleErr[robotid]) * cos(angleErr[robotid]) * maxForwardSpeed;
				setLineSpeed[robotid] = aimDist[robotid] * 6 - 20 * angleErr[robotid] * angleErr[robotid];
			if (setLineSpeed[robotid] < 0)setLineSpeed[robotid] = 0;
		}
		//��ƫ���ԽС�ٶ�Խ��
		if ((absF(angleErr[robotid]) >= PI * 0.5 && absF(angleErr[robotid]) <= PI * 0.75) || (aimDist[robotid] < 0.4 && leaveGridNum[robotid] <= 3))
			setLineSpeed[robotid] = 0.5;
		else if (absF(angleErr[robotid]) > PI * 0.75)
			setLineSpeed[robotid] = 0;
	}
	else
		setLineSpeed[robotid] = 0;
	////�ٶ���ֵ����
	if (setLineSpeed[robotid] > maxForwardSpeed)setLineSpeed[robotid] = maxForwardSpeed;
	if (setLineSpeed[robotid] < -maxBackSpeed)setLineSpeed[robotid] = -maxBackSpeed;
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
 * FunctionName: getAimPoint()
 * Input:�洢���Ƶ�����
 * Return: ��
 * Description: �õ����Ƶ�
 * Author: ����,2023/4/4
 */
void getAimPoint(double* aimPoint, int robotID)
{
	findBorderLine(robotID);
	int temRoadDir = roadDir[robotID][1];//���ڵ�·����
	realControlRow = controlRow;
	//��Զ���Ƶ��ж�����
	int endRow = minI(controlRow + 8, leaveGridNum[robotID] - 3);
	//controlRow��ǰ�ң��ҵ�·������ı����,ΪrealControlRow
	for (int j = 1; j < endRow; j++)
	{
		//���Ҳ������ҵ�·����쳣��
		if (findLineFlag[robotID][j] == 1 && roadWidth[robotID][j] <= 2.5 && absF(roadWidth[robotID][j + 1] - roadWidth[robotID][j]) >= 1)
		{
			realControlRow = j;
			break;
		}
	}
	//�Ҹı䷽�����
	for (int k = 1; k < endRow; k++)
	{
		if (roadDir[robotID][k] != temRoadDir)
		{
			realControlRow = minI(k, realControlRow);//�����ߺ͸ı䷽��ȡ��С
			break;
		}
		if (k == endRow - 1)
			realControlRow = minI(k, realControlRow);
	}
	int footstepLine = 0;
	while ((roadDir[robotID][footstepLine] != roadDir[robotID][footstepLine + 1]) && (roadDir[robotID][footstepLine] == roadDir[robotID][footstepLine + 2]))
	{
		footstepLine += 1;
		if (footstepLine >= 5)
			break;
	}
	realControlRow = maxI(realControlRow, footstepLine);
	//��ⳤֱ��
	int temRow = 0;
	while (temRow + 4 < endRow && roadDir[robotID][temRow] == roadDir[robotID][temRow + 1]
		&& roadDir[robotID][temRow + 1] == roadDir[robotID][temRow + 2]
		&& roadDir[robotID][temRow + 2] == roadDir[robotID][temRow + 3] && roadDir[robotID][temRow + 3] == roadDir[robotID][temRow + 4]
		)
		temRow += 1;
	realControlRow = maxI(realControlRow, temRow);
	//int noObLine = 0;//����û���ϰ���·������
	//while (!judgeHasBorder(robotID, 3, noObLine))
	//{
	//	noObLine += 1;
	//	if (noObLine >= 5)
	//		break;
	//}
	//realControlRow = maxI(realControlRow, noObLine - 1);
	int narrowLine = 1;//���Ŀ���������
	while (roadWidth[robotID][narrowLine] >= 3.5)
	{
		narrowLine += 1;
		if (narrowLine >= 5)
			break;
	}
	realControlRow = maxI(realControlRow, narrowLine);
	if (realControlRow > endRow - 1)
		realControlRow = endRow - 1;
	aimPoint[0] = midPoint[robotID][realControlRow][0];
	aimPoint[1] = midPoint[robotID][realControlRow][1];
	aimDist[robotID] = calculateDist(robot.local[robotID], aimPoint);
	//����������ת
	double minControlDist = 0.6;
	if (mapFlag == 3)
		minControlDist = 1.2;
	while (aimDist[robotID] < minControlDist)
	{
		if (realControlRow < realBoderGridNum[robotID] - 1)
			realControlRow += 1;
		else
			break;
		aimPoint[0] = midPoint[robotID][realControlRow][0];
		aimPoint[1] = midPoint[robotID][realControlRow][1];
		aimDist[robotID] = calculateDist(robot.local[robotID], aimPoint);
	}
}
/**
 * FunctionName: avGridDirControl()
 * Input: robotid
 * Return: ��
 * Description: ������ƺ���
 * Author: ����,2023/4/4
 */

void avGridDirControl(int robotid)
{
	static double error1[RobotNum] = { 0 }, error2[RobotNum] = { 0 };
	if (gridCollFlag[robotid] == 0)
	{
		error1[robotid] = 0;
		error2[robotid] = 0;
	}
	else {
		error2[robotid] = error1[robotid]; //�����ϴ�ƫ��
		error1[robotid] = avGridExAngle[robotid]; //�������ƫ��
		setAngleSpeed[robotid] = -(direction_Kp * error1[robotid] + 0.8 * direction_Kd * (error1[robotid] - error2[robotid]));
	}
}
/**
 * FunctionName: avGridSpeedControl()
 * Input: robotid
 * Return: ��
 * Description: ������ײ�ϰ��ٶȿ��ƺ���
 * Author: ����,2023/4/5
 */
void avGridSpeedControl(int robotid)
{
	setLineSpeed[robotid] = maxForwardSpeed;
	//���ٶȵ�����Ӧ�ú�angleErr�йأ����¸��˸���������ת��ģ�ͱȽϿ�ѧ
	//��ƫ���ԽС�ٶ�Խ��
	setLineSpeed[robotid] = cos(avGridExAngle[robotid]) * cos(avGridExAngle[robotid]) * maxForwardSpeed;
	if (absF(avGridExAngle[robotid]) >= PI * 0.25)
		setLineSpeed[robotid] = 0;
	////�ٶ���ֵ����
	if (setLineSpeed[robotid] > maxForwardSpeed)setLineSpeed[robotid] = maxForwardSpeed;
	if (setLineSpeed[robotid] < -maxBackSpeed)setLineSpeed[robotid] = -maxBackSpeed;
}
/**
 * FunctionName: speedControl()
 * Input: ��
 * Return: ��
 * Description: ��������ٶȿ��ƺ���
 * Author: ����,2023/3/12
 */
void speedControl() {
	//clock_t start, finish;
	//clock_t finish1, finish2, finish3;
	//double duration, duration1, duration2, duration3;
	for (int i = 0; i < RobotNum; i++)
	{
		//��û����˵�Ŀ�깤��̨·���б�
		int  deskid = aimDesk[i];
		//absF(aimDist[i] - leaveGridNum[i] * 0.5 + 0.5) <= 1
		if (aimDist[i] <= 2.4 && leaveGridNum[i] <= 5)
		{
			//gridCollFlag[i] = 0;
			//���¹���̨��ز���
			if (0 <= aimDesk[i] && aimDesk[i] <= 50)
				aimDist[i] = calculateDist(robot.local[i], workDesk.local[aimDesk[i]]);
			//��0��־λ
			aimPointArray[i][0] = workDesk.local[aimDesk[i]][0];
			aimPointArray[i][1] = workDesk.local[aimDesk[i]][1];
			if (leaveGridNum[i] >= 3)
			{
				//�ø���roadDir����
				findBorderLine(i);
				avoidGrid(i);
			}
			else
				gridCollFlag[i] = 0;

		}
		else {
			//���¹���̨��ز���
			if (0 <= aimDesk[i] && aimDesk[i] <= 50)
				aimDist[i] = calculateDist(robot.local[i], workDesk.local[aimDesk[i]]);
			getAimPoint(aimPointArray[i], i);
			avoidGrid(i);
		}
		if (gridCollFlag[i] == 0)
		{
			Direction_Control_Error(i, aimPointArray[i]);
			linSpeedControl(i);
		}
	}
}
