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
 * Return:0，不碰,1可能碰撞
 * Description: 判断一段时间后小球的碰撞
 * Author: 余游,2023/3/21
 */
bool judgeLaterColl(int robotid1, int robotid2, double laterLineSpeed1[2], double laterLineSpeed2[2])
{
	//ab的半径
	double Rab; double Pba[2];
	double v2pAngleErr;
	//计算a相对b的速度
	double Va2b[2] = { 0 }; double Va2bLen;
	Va2b[0] = laterLineSpeed1[0] - laterLineSpeed2[0];
	Va2b[1] = laterLineSpeed1[1] - laterLineSpeed2[1];
	//两者线速度相同直接返回false
	if (Va2b[0] == 0 && Va2b[1] == 0)
		return 0;
	//ab半径之和
	if ((robot.productType[robotid1] | robot.productType[robotid2]) == 0)
		Rab = 2 * robotSradius;
	else if ((robot.productType[robotid1] & robot.productType[robotid2]) == 1)
		Rab = 2 * robotBradius;
	else
		Rab = robotSradius + robotBradius;
	//ab相对位置矢量
	Pba[0] = robot.local[robotid2][0] - robot.local[robotid1][0];
	Pba[1] = robot.local[robotid2][1] - robot.local[robotid1][1];
	//速度矢量相对弧度
	double Va2bAngle = calculateAngle(laterLineSpeed2, laterLineSpeed1);

	//计算a相对b的速度与ab相对位置矢量的角度之差
	v2pAngleErr = getAngleErr(robot.local[robotid1], robot.local[robotid2], Va2bAngle);
	//位置矢量长度
	double PbaLen = calculateL2(Pba[0], Pba[1]);
	double  legAngle = asin(minF(Rab / PbaLen, 1.0));
	//加入边界余量
	if (absF(v2pAngleErr) >= legAngle)
	{
		return 0;
	}
	//每次只有较小序号的机器人需要旋转
	else
	{
		return 1;
	}
}
/**
 * FunctionName: bool dealColl(int robotid1, int robotid2)
 * Input:
 * Return:0，不碰,1可能碰撞
 * Description: 处理小球的碰撞
 * Author: 余游,2023/3/17
 */
bool dealColl(int robotid1, int robotid2)
{
	//ab的半径
	double Rab; double Pba[2];
	double v2pAngleErr;
	//计算a相对b的速度
	double Va2b[2] = { 0 }; double Va2bLen;
	Va2b[0] = robot.lineSpeed[robotid1][0] - robot.lineSpeed[robotid2][0];
	Va2b[1] = robot.lineSpeed[robotid1][1] - robot.lineSpeed[robotid2][1];
	//两者线速度相同直接返回false
	if (Va2b[0] == 0 && Va2b[1] == 0)
		return 0;
	//ab半径之和
	if ((robot.productType[robotid1] | robot.productType[robotid2]) == 0)
		Rab = 2 * robotSradius;
	else if ((robot.productType[robotid1] & robot.productType[robotid2]) == 1)
		Rab = 2 * robotBradius;
	else
		Rab = robotSradius + robotBradius;
	//ab相对位置矢量
	Pba[0] = robot.local[robotid2][0] - robot.local[robotid1][0];
	Pba[1] = robot.local[robotid2][1] - robot.local[robotid1][1];
	//速度矢量相对弧度
	double Va2bAngle = calculateAngle(robot.lineSpeed[robotid2], robot.lineSpeed[robotid1]);

	//计算a相对b的速度与ab相对位置矢量的角度之差
	v2pAngleErr = getAngleErr(robot.local[robotid1], robot.local[robotid2], Va2bAngle);
	//位置矢量长度
	double PbaLen = calculateL2(Pba[0], Pba[1]);
	double  legAngle = asin(minF(Rab / PbaLen, 1.0));
	//加入边界余量
	if (absF(v2pAngleErr) >= legAngle + 0.05)
	{
		return 0;
	}
	//每次只有较小序号的机器人需要旋转
	else
	{
		//需要旋转的角度
		double theta = legAngle - absF(v2pAngleErr);
		//当处于边界时优先朝远离边界方向运动
		if (legSelectFlag[robotid1] == 0)
		{
			if (v2pAngleErr < 0) {
				//合速度顺时针方向旋转
				legSelectFlag[robotid1] = 1;

			}
			else
				legSelectFlag[robotid1] = 2;
			//优化方法测试效果不佳
			//如果周围有机器人优先转向没有机器人的那边,,
			 //double minDist2robot1 = 10;
			 //for (int k = 0; k < RobotNum; k++)
			 //{
			 //	if (k == robotid1 || k == robotid2)
			 //		continue;
			 //	if (robot2robotDist[robotid1][k] < robot.radius[robotid1] + robot.radius[k] + collGap  && robot2robotDist[robotid1][k] < minDist2robot1
			 //		&& (robot2robotDist[robotid2][k] < robot.radius[robotid2] + robot.radius[k] + 2 || robot2robotDist[robotid1][k] < robot.radius[robotid1] + robot.radius[k] + 2))//与碰撞机器人和被碰机器人小于一定距离；
			 //	{
			 //		minDist2robot1 = robot2robotDist[robotid1][k];
			 //		//有机器人在逆时针方向，顺时针旋转
			 //		double robot12OtherAngle = getAngleErr(robot.local[robotid1], robot.local[k], robot.towardAngle[robotid1]);
			 //		if (robot12OtherAngle <= 0 && robot12OtherAngle >= -PI /8)
			 //			legSelectFlag[robotid1] = 1;
			 //		else if (robot12OtherAngle >= 0 && robot12OtherAngle <= PI /8)
			 //			legSelectFlag[robotid1] = 2;
			 //	}
			 //}

			if (robot.local[robotid1][0] < avoidBorderMDist + avoidBorderGap[robotid1] && robot.local[robotid2][0] < avoidBorderMDist + avoidBorderGap[robotid1])
			{
				//向上左边是边界优先顺时针
				avoidBorderGap[robotid1] = 1.5;
				if (robot.towardAngle[robotid1] >= 0)
					legSelectFlag[robotid1] = 1;
				else
					legSelectFlag[robotid1] = 2;
			}
			else if (robot.local[robotid1][0] > 50 - avoidBorderMDist - avoidBorderGap[robotid1] && robot.local[robotid2][0] > 50 - avoidBorderMDist - avoidBorderGap[robotid1])
			{
				//向上右边是边界优先逆时针
				avoidBorderGap[robotid1] = 1.5;
				if (robot.towardAngle[robotid1] >= 0)
					legSelectFlag[robotid1] = 2;
				else
					legSelectFlag[robotid1] = 1;
			}
			else if (robot.local[robotid1][1] < avoidBorderMDist + avoidBorderGap[robotid1] && robot.local[robotid2][1] < avoidBorderMDist + avoidBorderGap[robotid1])
			{
				//向右下边是边界优先逆时针
				avoidBorderGap[robotid1] = 1.5;
				if (robot.towardAngle[robotid1] >= -PI / 2 && robot.towardAngle[robotid1] <= PI / 2)
					legSelectFlag[robotid1] = 2;
				else
					legSelectFlag[robotid1] = 1;
			}
			else if (robot.local[robotid1][1] > 50 - avoidBorderMDist - avoidBorderGap[robotid1] && robot.local[robotid2][1] > 50 - avoidBorderMDist - avoidBorderGap[robotid1])
			{
				//向右上边是边界优先顺时针
				avoidBorderGap[robotid1] = 1.5;
				if (robot.towardAngle[robotid1] >= -PI / 2 && robot.towardAngle[robotid1] <= PI / 2)
					legSelectFlag[robotid1] = 1;
				else
					legSelectFlag[robotid1] = 2;
			}
		}
		//需要旋转的角度
		if (legSelectFlag[robotid1] == 1) {
			//合速度顺时针方向旋转
			if (v2pAngleErr < 0)
				theta = legAngle - absF(v2pAngleErr) + 0.05;
			else
				theta = legAngle + absF(v2pAngleErr) + 0.05;
			avoidExAngle[robotid1] = theta;
			//avoidExAngle[robotid2] = -theta / 2;

		}
		else if (legSelectFlag[robotid1] == 2) {
			//逆时针方向旋转
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
 * Description: 处理边界碰撞
 * Author: 余游,2023/3/17
 */
void avoidBorder()
{
	//处理边界碰撞情况；减速
	for (int i = 0; i < RobotNum; i++)
	{
		//不带负载
		if (robot.productType[i] == 0)
		{
			//边界速度角度和距离满足一定情况则减速
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
			//边界速度角度和距离满足一定情况则减速
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
 * Return:被碰对象id
 * Description: 判断碰撞状态
 * Author: 余游,2023/3/17
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
 * Input:无
 * Return: 无
 * Description: 根据时间窗口，过滤掉可能产生碰撞的速度控制输出，恢复为上一帧的速度
 * Author: 余游,2023/3/21 */
void filterCollSpeed(double time)
{
	//下一时间窗口机器人速度数组
	double nextlineSpeed[RobotNum][2];
	int robotid = 0;
	for (int i = 0; i < RobotNum; i++)
		estiamteSpeed(i, time, robot.lineSpeedAcc[i], robot.angleAcc[i], nextlineSpeed[i]);


	for (robotid = 0; robotid < RobotNum; robotid++)
	{
		//被碰对象id
		int collID = judgeColl(robotid);
		if (collID > 0)
			continue;
		//预测下一帧是否会处于VO区域，若会处于，采取上一帧的设定速度
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
 * Input:机器人编号
 * Return:无
 * Description:判断是否碰撞并赋值方向角
 * Author: 余游,2023/4/4 /*/
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
	int searchNum;//大机器人12个小的8个
	//对机器人8个方向进行搜索
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
 * Return:无
 * Description: 判断并处理机器人撞格子
 * Author: 余游,2023/4/4
 */
void dealGridColl(int robotid, double* mayBePoint, int currentSearchNum)
{
	//ab的半径
	double Rab; double Pba[2];
	double v2pAngleErr;
	//计算a相对b的速度
	double Va2b[2] = { 0 }; double Va2bLen;
	//ab半径之和
	if (robot.productType[robotid] == 0)
		Rab = robotSradius + 0.25 * 1.415;
	else
		Rab = 0.25 * 1.415 + robotBradius;
	//ab相对位置矢量
	Pba[0] = mayBePoint[0] - robot.local[robotid][0];
	Pba[1] = mayBePoint[1] - robot.local[robotid][1];
	if (calculateL2(Pba[0], Pba[1]) > Rab + 0.1 || lineSpeed[robotid] > 0.05)
	{
		gridCollFlag[robotid] = 0;
		return;
	}
	//已经碰撞
	if (calculateL2(Pba[0], Pba[1]) <= Rab)
	{
		gridCollFlag[robotid] = 1;
		//将第0行角度视为目标角度
		double p2top1Angle = 0;//位置角度
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

		avGridExAngle[robotid] = robot.towardAngle[robotid] - p2top1Angle; //机器人与目标点角度差
		//由于弧度界限突变需要对偏差角度做处理
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
	//速度矢量相对弧度
	double Va2bAngle = robot.towardAngle[robotid];

	//计算a相对b的速度与ab相对位置矢量的角度之差
	v2pAngleErr = getAngleErr(robot.local[robotid], mayBePoint, Va2bAngle);
	//位置矢量长度
	double PbaLen = calculateL2(Pba[0], Pba[1]);
	double  legAngle = asin(minF(Rab / (PbaLen), 1.0));
	//加入边界余量,进入避障状态后需要转到距离期望角一定角度才能退出
	if (absF(v2pAngleErr) >= legAngle && ((gridCollFlag[robotid] == 1 && absF(avGridExAngle[robotid]) < 0.2) || gridCollFlag[robotid] == 0))
	{
		gridCollFlag[robotid] = 0;
	}
	else
	{
		//将第0行角度视为目标角度
		double p2top1Angle = 0;//位置角度
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

		avGridExAngle[robotid] = robot.towardAngle[robotid] - p2top1Angle; //机器人与目标点角度差
		//由于弧度界限突变需要对偏差角度做处理
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
 * Input:机器人编号
 * Return: 无
 * Description: 碰撞障碍物紧急处理
 * Author: 余游,2023/4/2 */
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
			//已经碰撞
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
 * Description: 处理碰撞
 * Author: 余游,2023/3/17
 */
void avoidOb()
{
	//处理边界碰撞情况；减速
	avoidBorder();
	//小球碰撞处理
	//int newFlag[RobotNum] = { 0 };//设置每次变化的Flag数组，当 newFlag为1时说明前面的循环得到了这个id的机器人可能发生碰撞，直接跳过这次循环
	for (int i = 0; i < RobotNum - 1; i++)
	{
		//碰撞可能的对象计数
		int count = 0;
		for (int j = i + 1; j < RobotNum; j++)
		{
			//距离大于一定值视为不碰撞,清0标志位和避障环期望角度赋予一个不可能的值
			if (robot2robotDist[i][j] >= robot.radius[i] + robot.radius[j] + collGap)
			{
				avoidBorderGap[i] = 0;
				collisionFlag[i][j] = 0;
				collisionFlag[j][i] = 0;
				avoidExAngle[i] = 8;
				//恢复初始legSelectFlag
				legSelectFlag[i] = 0;
				continue;
			}
			else
			{
				//可能发生碰撞;置标志位
				count += dealColl(i, j);
				if (count > 0) {
					collisionFlag[i][j] = 1;
					collisionFlag[j][i] = 1;
					//暂时只处理两个物体碰撞
					//i的其它标注位置0
					for (int k = j + 1; k < RobotNum; k++)
					{
						collisionFlag[i][k] = 0;
						collisionFlag[k][i] = 0;
					}
					break;
				}
				//没有碰撞可能,清0标志位和避障环期望角度赋予一个不可能的值
				else
				{
					//恢复初始legSelectFlag
					avoidBorderGap[i] = 0;
					legSelectFlag[i] = 0;
					collisionFlag[i][j] = 0;
					collisionFlag[j][i] = 0;
					avoidExAngle[i] = 8;
				}
			}
		}
	}
	//避障方向环的控制
	for (int i = 0; i < RobotNum; i++)
	{
		avoidDireControl(i);
		avoidSpeedControl(i);
	}
	////测试效果不佳
	//filterCollSpeed( 0.001);
}