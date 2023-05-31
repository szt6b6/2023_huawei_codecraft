#include"globalVariables.h"
int collisionFlag[RobotNum][RobotNum] = { 0 };
int  legSelectFlag[RobotNum] = { 0 };
double avoidBorderGap[RobotNum] = { 0 };
int collisionStateFlag[RobotNum] = { 0 };
double avoidAngleGap[RobotNum] = { 0 };
int colledFlag[RobotNum][RobotNum] = { 0 };
/**
 * FunctionName: judgeLaterColl
 * Input:
 * Return:0������,1������ײ
 * Description: �ж�һ��ʱ���С�����ײ
 * Author: ����,2023/3/21
 */
bool judgeLaterColl(int robotid1, int robotid2, double laterLineSpeed1[2], double laterLineSpeed2[2])
{
	//ab�İ뾶
	double Rab; double Pba[2];
	double v2pAngleErr;
	//����a���b���ٶ�
	double Va2b[2] = { 0 }; double Va2bLen;
	Va2b[0] = laterLineSpeed1[0] - laterLineSpeed2[0];
	Va2b[1] = laterLineSpeed1[1] - laterLineSpeed2[1];
	//�������ٶ���ֱͬ�ӷ���false
	if (Va2b[0] == 0 && Va2b[1] == 0)
		return 0;
	//ab�뾶֮��
	if ((robot.productType[robotid1] | robot.productType[robotid2]) == 0)
		Rab = 2 * robotSradius;
	else if ((robot.productType[robotid1] & robot.productType[robotid2]) == 1)
		Rab = 2 * robotBradius;
	else
		Rab = robotSradius + robotBradius;
	//ab���λ��ʸ��
	Pba[0] = robot.local[robotid2][0] - robot.local[robotid1][0];
	Pba[1] = robot.local[robotid2][1] - robot.local[robotid1][1];
	//�ٶ�ʸ����Ի���
	double Va2bAngle = calculateAngle(laterLineSpeed2, laterLineSpeed1);

	//����a���b���ٶ���ab���λ��ʸ���ĽǶ�֮��
	v2pAngleErr = getAngleErr(robot.local[robotid1], robot.local[robotid2], Va2bAngle);
	//λ��ʸ������
	double PbaLen = calculateL2(Pba[0], Pba[1]);
	double  legAngle = asin(minF(Rab / PbaLen, 1.0));
	//����߽�����
	if (absF(v2pAngleErr) >= legAngle)
	{
		return 0;
	}
	//ÿ��ֻ�н�С��ŵĻ�������Ҫ��ת
	else
	{
		return 1;
	}
}
/**
 * FunctionName: bool dealColl(int robotid1, int robotid2)
 * Input:
 * Return:0������,1������ײ
 * Description: ����С�����ײ
 * Author: ����,2023/3/17
 */
bool dealColl(int robotid1, int robotid2)
{
	//ab�İ뾶
	double Rab; double Pba[2];
	double v2pAngleErr;
	//����a���b���ٶ�
	double Va2b[2] = { 0 }; double Va2bLen;
	Va2b[0] = robot.lineSpeed[robotid1][0] - robot.lineSpeed[robotid2][0];
	Va2b[1] = robot.lineSpeed[robotid1][1] - robot.lineSpeed[robotid2][1];
	//�������ٶ���ֱͬ�ӷ���false
	if (Va2b[0] == 0 && Va2b[1] == 0)
		return 0;
	//ab�뾶֮��
	if ((robot.productType[robotid1] | robot.productType[robotid2]) == 0)
		Rab = 2 * robotSradius;
	else if ((robot.productType[robotid1] & robot.productType[robotid2]) == 1)
		Rab = 2 * robotBradius;
	else
		Rab = robotSradius + robotBradius;
	//ab���λ��ʸ��
	Pba[0] = robot.local[robotid2][0] - robot.local[robotid1][0];
	Pba[1] = robot.local[robotid2][1] - robot.local[robotid1][1];
	//�ٶ�ʸ����Ի���
	double Va2bAngle = calculateAngle(robot.lineSpeed[robotid2], robot.lineSpeed[robotid1]);

	//����a���b���ٶ���ab���λ��ʸ���ĽǶ�֮��
	v2pAngleErr = getAngleErr(robot.local[robotid1], robot.local[robotid2], Va2bAngle);
	//λ��ʸ������
	double PbaLen = calculateL2(Pba[0], Pba[1]);
	double  legAngle = asin(minF(Rab / PbaLen, 1.0));
	//����߽�����
	if (absF(v2pAngleErr) >= legAngle + 0.05)
	{
		return 0;
	}
	//ÿ��ֻ�н�С��ŵĻ�������Ҫ��ת
	else
	{
		//��Ҫ��ת�ĽǶ�
		double theta = legAngle - absF(v2pAngleErr);
		//�����ڱ߽�ʱ���ȳ�Զ��߽緽���˶�
		if (legSelectFlag[robotid1] == 0)
		{
			if (v2pAngleErr < 0) {
				//���ٶ�˳ʱ�뷽����ת
				legSelectFlag[robotid1] = 1;

			}
			else
				legSelectFlag[robotid1] = 2;
			//�Ż���������Ч������
			//�����Χ�л���������ת��û�л����˵��Ǳ�,,
			 //double minDist2robot1 = 10;
			 //for (int k = 0; k < RobotNum; k++)
			 //{
			 //	if (k == robotid1 || k == robotid2)
			 //		continue;
			 //	if (robot2robotDist[robotid1][k] < robot.radius[robotid1] + robot.radius[k] + collGap  && robot2robotDist[robotid1][k] < minDist2robot1
			 //		&& (robot2robotDist[robotid2][k] < robot.radius[robotid2] + robot.radius[k] + 2 || robot2robotDist[robotid1][k] < robot.radius[robotid1] + robot.radius[k] + 2))//����ײ�����˺ͱ���������С��һ�����룻
			 //	{
			 //		minDist2robot1 = robot2robotDist[robotid1][k];
			 //		//�л���������ʱ�뷽��˳ʱ����ת
			 //		double robot12OtherAngle = getAngleErr(robot.local[robotid1], robot.local[k], robot.towardAngle[robotid1]);
			 //		if (robot12OtherAngle <= 0 && robot12OtherAngle >= -PI /8)
			 //			legSelectFlag[robotid1] = 1;
			 //		else if (robot12OtherAngle >= 0 && robot12OtherAngle <= PI /8)
			 //			legSelectFlag[robotid1] = 2;
			 //	}
			 //}

			if (robot.local[robotid1][0] < avoidBorderMDist + avoidBorderGap[robotid1] && robot.local[robotid2][0] < avoidBorderMDist + avoidBorderGap[robotid1])
			{
				//��������Ǳ߽�����˳ʱ��
				avoidBorderGap[robotid1] = 1.5;
				if (robot.towardAngle[robotid1] >= 0)
					legSelectFlag[robotid1] = 1;
				else
					legSelectFlag[robotid1] = 2;
			}
			else if (robot.local[robotid1][0] > 50 - avoidBorderMDist - avoidBorderGap[robotid1] && robot.local[robotid2][0] > 50 - avoidBorderMDist - avoidBorderGap[robotid1])
			{
				//�����ұ��Ǳ߽�������ʱ��
				avoidBorderGap[robotid1] = 1.5;
				if (robot.towardAngle[robotid1] >= 0)
					legSelectFlag[robotid1] = 2;
				else
					legSelectFlag[robotid1] = 1;
			}
			else if (robot.local[robotid1][1] < avoidBorderMDist + avoidBorderGap[robotid1] && robot.local[robotid2][1] < avoidBorderMDist + avoidBorderGap[robotid1])
			{
				//�����±��Ǳ߽�������ʱ��
				avoidBorderGap[robotid1] = 1.5;
				if (robot.towardAngle[robotid1] >= -PI / 2 && robot.towardAngle[robotid1] <= PI / 2)
					legSelectFlag[robotid1] = 2;
				else
					legSelectFlag[robotid1] = 1;
			}
			else if (robot.local[robotid1][1] > 50 - avoidBorderMDist - avoidBorderGap[robotid1] && robot.local[robotid2][1] > 50 - avoidBorderMDist - avoidBorderGap[robotid1])
			{
				//�����ϱ��Ǳ߽�����˳ʱ��
				avoidBorderGap[robotid1] = 1.5;
				if (robot.towardAngle[robotid1] >= -PI / 2 && robot.towardAngle[robotid1] <= PI / 2)
					legSelectFlag[robotid1] = 1;
				else
					legSelectFlag[robotid1] = 2;
			}
		}
		//��Ҫ��ת�ĽǶ�
		if (legSelectFlag[robotid1] == 1) {
			//���ٶ�˳ʱ�뷽����ת
			if (v2pAngleErr < 0)
				theta = legAngle - absF(v2pAngleErr) + 0.05;
			else
				theta = legAngle + absF(v2pAngleErr) + 0.05;
			avoidExAngle[robotid1] = theta;
			//avoidExAngle[robotid2] = -theta / 2;

		}
		else if (legSelectFlag[robotid1] == 2) {
			//��ʱ�뷽����ת
			if (v2pAngleErr < 0)
				theta = legAngle + absF(v2pAngleErr) + 0.05;
			else
				theta = legAngle - absF(v2pAngleErr) + 0.05;
			avoidExAngle[robotid1] = -theta;
			//avoidExAngle[robotid2] = theta / 2;
		}
		return 1;
	}
}
/**
 * FunctionName: avoidBorder
 * Input:
 * Return:
 * Description: ����߽���ײ
 * Author: ����,2023/3/17
 */
void avoidBorder()
{
	//����߽���ײ���������
	for (int i = 0; i < RobotNum; i++)
	{
		//��������
		if (robot.productType[i] == 0)
		{
			//�߽��ٶȽǶȺ;�������һ����������
			if (robot.local[i][0] < 2.25 && absF(robot.towardAngle[i]) > PI * 3 / 4)
				setLineSpeed[i] = minF(2 - 2 * (1.5 - robot.local[i][0]), setLineSpeed[i]);
			else if (robot.local[i][0] > 47.75 && absF(robot.towardAngle[i]) < PI / 4)
				setLineSpeed[i] = minF(2 - 2 * (robot.local[i][0] - 48.5), setLineSpeed[i]);
			if (robot.local[i][1] < 2.25 && absF(robot.towardAngle[i] + PI / 2) < PI / 4)
				setLineSpeed[i] = minF(2 - 2 * (1.5 - robot.local[i][1]), setLineSpeed[i]);
			else if (robot.local[i][1] > 47.75 && absF(robot.towardAngle[i] - PI / 2) < PI / 4)
				setLineSpeed[i] = minF(2 - 2 * (robot.local[i][1] - 48.5), setLineSpeed[i]);
		}
		else
		{
			//�߽��ٶȽǶȺ;�������һ����������
			if (robot.local[i][0] < 2.5 && absF(robot.towardAngle[i]) > PI * 3 / 4)
				setLineSpeed[i] = minF(2 - 0.8 * (2.25 - robot.local[i][0]), setLineSpeed[i]);
			if (robot.local[i][0] > 47.5 && absF(robot.towardAngle[i]) < PI / 4)
				setLineSpeed[i] = minF(2 - 0.8 * (robot.local[i][0] - 47.75), setLineSpeed[i]);
			if (robot.local[i][1] < 2.5 && absF(robot.towardAngle[i] + PI / 2) < PI / 4)
				setLineSpeed[i] = minF(2 - 0.8 * (2.25 - robot.local[i][1]), setLineSpeed[i]);
			if (robot.local[i][1] > 47.5 && absF(robot.towardAngle[i] - PI / 2) < PI / 4)
				setLineSpeed[i] = minF(2 - 0.8 * (robot.local[i][1] - 47.75), setLineSpeed[i]);
		}
	}
}
/**
 * FunctionName: judgeColl
 * Input:robotid
 * Return:��������id
 * Description: �ж���ײ״̬
 * Author: ����,2023/3/17
 */
int judgeColl(int robotid)
{
	for (int i = robotid + 1; i < RobotNum; i++)
	{
		if (collisionFlag[robotid][i] == 1)
			return i;
	}
	return 0;
}


/**
 * FunctionName:filterCollSpeed()
 * Input:��
 * Return: ��
 * Description: ����ʱ�䴰�ڣ����˵����ܲ�����ײ���ٶȿ���������ָ�Ϊ��һ֡���ٶ�
 * Author: ����,2023/3/21 */
void filterCollSpeed(double time)
{
	//��һʱ�䴰�ڻ������ٶ�����
	double nextlineSpeed[RobotNum][2];
	int robotid = 0;
	for (int i = 0; i < RobotNum; i++)
		estiamteSpeed(i, time, robot.lineSpeedAcc[i], robot.angleAcc[i], nextlineSpeed[i]);


	for (robotid = 0; robotid < RobotNum; robotid++)
	{
		//��������id
		int collID = judgeColl(robotid);
		if (collID > 0)
			continue;
		//Ԥ����һ֡�Ƿ�ᴦ��VO�������ᴦ�ڣ���ȡ��һ֡���趨�ٶ�
		for (int i = 0; i < RobotNum; i++)
		{
			if (i == robotid)
				continue;
			if (judgeLaterColl(robotid, i, nextlineSpeed[robotid], nextlineSpeed[i]) == 1)
			{
				int j = 2;
				setLineSpeed[robotid] = lastSetSpeed[robotid];
				setAngleSpeed[robotid] = lastSetAngleSpeed[robotid];
				break;
			}
		}
	}
}
/**
 * FunctionName:judgeGridColl(robotid)
 * Input:�����˱��
 * Return:��
 * Description:�ж��Ƿ���ײ����ֵ�����
 * Author: ����,2023/4/4 /*/
void judgeGridColl(int robotid)
{
	int searchDir[20][2] = {
	{  1, 0 },{  -1, 0 },
	{ 0,  1 },{ 0, -1 },
	{ 1, 1 },{ 1, -1 },
	{ -1,  +1 },{ -1, -1 },
	{2,0},{-2,0},{0,2},{0,-2},
	{2,1},{-2,0},{1,2},{1,-2},
	{2,-1},{-2,0},{-1,2},{-1,-2}
	};
	double mayBePoint[2] = { 0 };
	int searchNum;//�������12��С��8��
	//�Ի�����8�������������
	if (robot.radius[robotid] > 0.5)searchNum = 20;
	else
		searchNum = 8;
	for (int i = 0; i < searchNum; i++)
	{
		if (map_data[robot.grid[robotid][0] + searchDir[i][0]][robot.grid[robotid][1] + searchDir[i][1]] != '#')
			continue;
		mayBePoint[0] = (robot.grid[robotid][1] + searchDir[i][1]) * 0.5 + 0.25;
		mayBePoint[1] = (99 - (robot.grid[robotid][0] + searchDir[i][0])) * 0.5 + 0.25;
		dealGridColl(robotid, mayBePoint, i);
		if (gridCollFlag[robotid] == 1)
			return;
	}
	gridCollFlag[robotid] = 0;
}
/**
 * FunctionName: dealGridColl(int robotid1,double*mayBePoint )
 * Input:
 * Return:��
 * Description: �жϲ����������ײ����
 * Author: ����,2023/4/4
 */
void dealGridColl(int robotid, double* mayBePoint, int currentSearchNum)
{
	//ab�İ뾶
	double Rab; double Pba[2];
	double v2pAngleErr;
	//����a���b���ٶ�
	double Va2b[2] = { 0 }; double Va2bLen;
	//ab�뾶֮��
	if (robot.productType[robotid] == 0)
		Rab = robotSradius + 0.25 * 1.415;
	else
		Rab = 0.25 * 1.415 + robotBradius;
	//ab���λ��ʸ��
	Pba[0] = mayBePoint[0] - robot.local[robotid][0];
	Pba[1] = mayBePoint[1] - robot.local[robotid][1];
	if (calculateL2(Pba[0], Pba[1]) > Rab + 0.1 || lineSpeed[robotid] > 0.05)
	{
		gridCollFlag[robotid] = 0;
		return;
	}
	//�Ѿ���ײ
	if (calculateL2(Pba[0], Pba[1]) <= Rab)
	{
		gridCollFlag[robotid] = 1;
		//����0�нǶ���ΪĿ��Ƕ�
		double p2top1Angle = 0;//λ�ýǶ�
		if (robot.radius[robotid] > 0.5 && currentSearchNum > 7)
		{
			switch (roadDir[robotid][1])
			{
			case 0:p2top1Angle = robot.towardAngle[robotid]; break;
			case 1: p2top1Angle = 0; break;
			case 2: p2top1Angle = 0.5 * PI; break;
			case 3: p2top1Angle = PI; break;
			case 4: p2top1Angle = -0.5 * PI; break;
			case 5: p2top1Angle = 0.25 * PI; break;
			case 6: p2top1Angle = 0.75 * PI; break;
			case 7: p2top1Angle = -0.75 * PI; break;
			case 8: p2top1Angle = -0.25 * PI; break;
			}
		}
		else
		{
			switch (roadDir[robotid][0])
			{
			case 0:p2top1Angle = robot.towardAngle[robotid]; break;
			case 1: p2top1Angle = 0; break;
			case 2: p2top1Angle = 0.5 * PI; break;
			case 3: p2top1Angle = PI; break;
			case 4: p2top1Angle = -0.5 * PI; break;
			case 5: p2top1Angle = 0.25 * PI; break;
			case 6: p2top1Angle = 0.75 * PI; break;
			case 7: p2top1Angle = -0.75 * PI; break;
			case 8: p2top1Angle = -0.25 * PI; break;
			}
		}

		avGridExAngle[robotid] = robot.towardAngle[robotid] - p2top1Angle; //��������Ŀ���ǶȲ�
		//���ڻ��Ƚ���ͻ����Ҫ��ƫ��Ƕ�������
		if (avGridExAngle[robotid] >= PI)
		{
			avGridExAngle[robotid] = 2 * PI - avGridExAngle[robotid];
			avGridExAngle[robotid] = -avGridExAngle[robotid];
		}
		if (avGridExAngle[robotid] <= -PI)
		{
			avGridExAngle[robotid] = 2 * PI + avGridExAngle[robotid];
		}
		return;
	}
	Va2b[0] = robot.lineSpeed[robotid][0];
	Va2b[1] = robot.lineSpeed[robotid][1];
	//�ٶ�ʸ����Ի���
	double Va2bAngle = robot.towardAngle[robotid];

	//����a���b���ٶ���ab���λ��ʸ���ĽǶ�֮��
	v2pAngleErr = getAngleErr(robot.local[robotid], mayBePoint, Va2bAngle);
	//λ��ʸ������
	double PbaLen = calculateL2(Pba[0], Pba[1]);
	double  legAngle = asin(minF(Rab / (PbaLen), 1.0));
	//����߽�����,�������״̬����Ҫת������������һ���ǶȲ����˳�
	if (absF(v2pAngleErr) >= legAngle && ((gridCollFlag[robotid] == 1 && absF(avGridExAngle[robotid]) < 0.2) || gridCollFlag[robotid] == 0))
	{
		gridCollFlag[robotid] = 0;
	}
	else
	{
		//����0�нǶ���ΪĿ��Ƕ�
		double p2top1Angle = 0;//λ�ýǶ�
		if (robot.radius[robotid] > 0.5 && currentSearchNum > 7)
		{
			switch (roadDir[robotid][1])
			{
			case 1: p2top1Angle = 0; break;
			case 2: p2top1Angle = 0.5 * PI; break;
			case 3: p2top1Angle = PI; break;
			case 4: p2top1Angle = -0.5 * PI; break;
			case 5: p2top1Angle = 0.25 * PI; break;
			case 6: p2top1Angle = 0.75 * PI; break;
			case 7: p2top1Angle = -0.75 * PI; break;
			case 8: p2top1Angle = -0.25 * PI; break;
			}
		}
		else
		{
			switch (roadDir[robotid][0])
			{
			case 1: p2top1Angle = 0; break;
			case 2: p2top1Angle = 0.5 * PI; break;
			case 3: p2top1Angle = PI; break;
			case 4: p2top1Angle = -0.5 * PI; break;
			case 5: p2top1Angle = 0.25 * PI; break;
			case 6: p2top1Angle = 0.75 * PI; break;
			case 7: p2top1Angle = -0.75 * PI; break;
			case 8: p2top1Angle = -0.25 * PI; break;
			}
		}

		avGridExAngle[robotid] = robot.towardAngle[robotid] - p2top1Angle; //��������Ŀ���ǶȲ�
		//���ڻ��Ƚ���ͻ����Ҫ��ƫ��Ƕ�������
		if (avGridExAngle[robotid] >= PI)
		{
			avGridExAngle[robotid] = 2 * PI - avGridExAngle[robotid];
			avGridExAngle[robotid] = -avGridExAngle[robotid];
		}
		if (avGridExAngle[robotid] <= -PI)
		{
			avGridExAngle[robotid] = 2 * PI + avGridExAngle[robotid];
		}
		gridCollFlag[robotid] = 1;
	}
}
/**
 * FunctionName:avoidGrid(robotid)
 * Input:�����˱��
 * Return: ��
 * Description: ��ײ�ϰ����������
 * Author: ����,2023/4/2 */
void avoidGrid(int robotid)
{
	judgeGridColl(robotid);
	avGridDirControl(robotid);
	if (gridCollFlag[robotid] == 1)
		avGridSpeedControl(robotid);
}

void colledDeal()
{
	for (int i = 0; i < RobotNum; i++)
	{
		for (int j = 0; j < RobotNum; j++)
		{
			if (i == j)continue;
			//�Ѿ���ײ
			if ((robot2robotDist[i][j] <= robot.radius[i] + robot.radius[j] + 0.002 && (lineSpeed[i] < 0.5 && lineSpeed[j] < 0.5))
				|| (colledFlag[i][j] == 1 && robot2robotDist[i][j] < robot.radius[i] + robot.radius[j]+ 0.5))
			{
				colledFlag[i][j] = 1;
				setLineSpeed[i] = -1;
				setLineSpeed[j] = -1;
			}
			else
			{
				colledFlag[i][j] = 0;
			}

		}
	}
}
/**
 * FunctionName: avoidOb
 * Input:
 * Return:
 * Description: ������ײ
 * Author: ����,2023/3/17
 */
void avoidOb()
{
	//����߽���ײ���������
	avoidBorder();
	//С����ײ����
	//int newFlag[RobotNum] = { 0 };//����ÿ�α仯��Flag���飬�� newFlagΪ1ʱ˵��ǰ���ѭ���õ������id�Ļ����˿��ܷ�����ײ��ֱ���������ѭ��
	for (int i = 0; i < RobotNum - 1; i++)
	{
		//��ײ���ܵĶ������
		int count = 0;
		for (int j = i + 1; j < RobotNum; j++)
		{
			//�������һ��ֵ��Ϊ����ײ,��0��־λ�ͱ��ϻ������Ƕȸ���һ�������ܵ�ֵ
			if (robot2robotDist[i][j] >= robot.radius[i] + robot.radius[j] + collGap)
			{
				avoidBorderGap[i] = 0;
				collisionFlag[i][j] = 0;
				collisionFlag[j][i] = 0;
				avoidExAngle[i] = 8;
				//�ָ���ʼlegSelectFlag
				legSelectFlag[i] = 0;
				continue;
			}
			else
			{
				//���ܷ�����ײ;�ñ�־λ
				count += dealColl(i, j);
				if (count > 0) {
					collisionFlag[i][j] = 1;
					collisionFlag[j][i] = 1;
					//��ʱֻ��������������ײ
					//i��������עλ��0
					for (int k = j + 1; k < RobotNum; k++)
					{
						collisionFlag[i][k] = 0;
						collisionFlag[k][i] = 0;
					}
					break;
				}
				//û����ײ����,��0��־λ�ͱ��ϻ������Ƕȸ���һ�������ܵ�ֵ
				else
				{
					//�ָ���ʼlegSelectFlag
					avoidBorderGap[i] = 0;
					legSelectFlag[i] = 0;
					collisionFlag[i][j] = 0;
					collisionFlag[j][i] = 0;
					avoidExAngle[i] = 8;
				}
			}
		}
	}
	//���Ϸ��򻷵Ŀ���
	for (int i = 0; i < RobotNum; i++)
	{
		avoidDireControl(i);
		avoidSpeedControl(i);
	}
	////����Ч������
	//filterCollSpeed( 0.001);
}