#include"globalVariables.h"
int collisionFlag[RobotNum][RobotNum] = { 0 };
int  legSelectFlag[RobotNum] = { 0 };
double avoidBorderGap[RobotNum] = { 0 };
int collisionStateFlag[RobotNum] = { 0 };
double avoidAngleGap[RobotNum] = { 0 };
/**
 * FunctionName: judgeLaterColl
 * Input:
 * Return:0������,1������ײ
 * Description: �ж�һ��ʱ���С�����ײ
 * Author: ����,2023/3/21
 */
bool judgeLaterColl(int robotid1, int robotid2,double laterLineSpeed1[2], double laterLineSpeed2[2] )
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
	if (absF(v2pAngleErr) >= legAngle)
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
			//if (v2pAngleErr < 0)
			//	theta = legAngle - absF(v2pAngleErr);
			//else
			//	theta = legAngle +absF(v2pAngleErr);
			avoidExAngle[robotid1] = 0.6*theta;
			//avoidExAngle[robotid2] = -theta / 2;

		}
		else if (legSelectFlag[robotid1] == 2) {
			//��ʱ�뷽����ת
			//if (v2pAngleErr < 0)
			//	theta = legAngle +absF(v2pAngleErr);
			//else
			//	theta = legAngle - absF(v2pAngleErr);
			avoidExAngle[robotid1] = -theta * 0.6;
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
	int robotid=0;
	for (int i = 0; i < RobotNum; i++)
		estiamteSpeed(i, time, robot.lineSpeedAcc[i], robot.angleAcc[i], nextlineSpeed[i]);


	for (robotid = 0; robotid< RobotNum; robotid++)
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