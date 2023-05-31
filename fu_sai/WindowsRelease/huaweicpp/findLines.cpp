#include"globalVariables.h"
int hasBoderFlag[RobotNum] = { 0 };
int leftBoderGrid[RobotNum][BoderGridNum][2] = { 0 };
int rightBoderGrid[RobotNum][BoderGridNum][2] = { 0 };
double midPoint[RobotNum][BoderGridNum][2] = { 0 };
int leaveGridNum[RobotNum] = { 1000 ,1000,1000,1000 };
double roadWidth[RobotNum][BoderGridNum] = { 0 };
int realBoderGridNum[RobotNum];
double controlPoint[RobotNum][2] = { 0 };
int temGridList[RobotNum][BoderGridNum][2] = { 0 };
int roadDir[RobotNum][BoderGridNum] = { 0 };
int realControlRow;
int findLineFlag[RobotNum][BoderGridNum] = { 0 };
double  temGridPoint[BoderGridNum][2] = { 0 };
bool avoidPathConflic[4] = { false };
/**
 * FunctionName: judgeHasBorder()
 * Input: ���������,�������Ӹ����뾶��2������2.5X2.5��Χ
 * Return: 0,����û�ҵ�����Ϊ��Χ���ϰ�
 * Description: �жϻ�������Χ�Ƿ�����ϰ�
 * Author: ����,2023/3/29
 */
bool judgeHasBorder(int robotid, int len, int startLine)
{
	for (int i = 0; i < len + 1; i++)
	{
		for (int j = 0; j < len + 1; j++)
		{
			if (map_data[temGridList[robotid][startLine][0] + i][temGridList[robotid][startLine][1] + j] == '#' ||
				map_data[temGridList[robotid][startLine][0] + i][temGridList[robotid][startLine][1] - j] == '#' ||
				map_data[temGridList[robotid][startLine][0] - i][temGridList[robotid][startLine][1] + j] == '#' ||
				map_data[temGridList[robotid][startLine][0] - i][temGridList[robotid][startLine][1] - j] == '#'
				)
				return 1;
		}
	}
	return 0;
}
/**
 * FunctionName: findBorderLine()
 * Input: ���������
 * Return:
 * Description: ������ߣ��ұ��ߣ�������
 * Author: ����,2023/3/31
 */
void findBorderLine(int robotID)
{
	int  deskid = aimDesk[robotID];
	//��û����˵�Ŀ�깤��̨·���б�
	std::list<Node_c> temlist;

	//�ж�һ��·����ͻ 
	int higher_r_id = return_highter_robot(robotID);

	if (higher_r_id != -1 && check_in_path(robotID, higher_r_id)) {

		out_of_way_robot(robotID, higher_r_id);
		temlist = working_robots_queue[robotID].path;

		avoidPathConflic[robotID] = true;

	}
	else {
		Point current = { robot.grid[robotID][0], robot.grid[robotID][1] };
		Point target = { bench_seq_dist[deskid][0], bench_seq_dist[deskid][1] };
		temlist = find_path(current, target, working_robots_queue[robotID].buyOrSell == 1);
		working_robots_queue[robotID].path = temlist;
	}
	//if(temlist.size() > 0) temlist.pop_front(); //����һ���� Ҳ���ǵ�ǰ�ڵĵ�pop��
	/*for (int i = 0; i < 100; i++)
		for (int j = 0; j < 100; j++)
			map_data_to_look_path[i][j] = map_data[i][j];

	std::list<Node_c> tem = temlist;
	if (tem.size() >= 2) {
		tem.pop_front();
		tem.pop_back();
	}
	for (Node_c t : tem) {
		map_data_to_look_path[t.point.x][t.point.y] = '*';
	}

	std::fstream f;
	f.open(".\\data.txt", std::ios::out);

	for (int i = 0; i < 100; i++) {
		for (int j = 0; j < 100; j++) {
			f << map_data_to_look_path[i][j];
		}
		f << "\n";
	}
	f.close();
*/

	leaveGridNum[robotID] = temlist.size();
	
	realBoderGridNum[robotID] = minI(leaveGridNum[robotID] - 1, BoderGridNum);
	//��ÿ��·�����������浽temGridList
	int count = 0;
	//���·��������
	for (Node_c test : temlist)
	{
		temGridList[robotID][count][0] = test.point.x;
		temGridList[robotID][count][1] = test.point.y;
		count++;
		if (count == realBoderGridNum[robotID])
			break;
	}


	//����2��ʱ�������յ㲻��ͨ
	if (realBoderGridNum[robotID] > 2)
	{
		for (int i = 0; i < realBoderGridNum[robotID]; i++)
		{
			temGridPoint[i][0] = temGridList[robotID][i][1] * 0.5 + 0.25;
			temGridPoint[i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0.25;
			//�ж�·������
			//����
			if (temGridList[robotID][i][0] == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] + 1 == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 1;
			//����
			else if (temGridList[robotID][i][0] == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] - 1 == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 3;
			//����
			else if (temGridList[robotID][i][0] - 1 == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 2;
			//����
			else if (temGridList[robotID][i][0] + 1 == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 4;
			//����
			else if (temGridList[robotID][i][0] - 1 == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] + 1 == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 5;
			//����
			else if (temGridList[robotID][i][0] - 1 == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] - 1 == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 6;
			//����
			else if (temGridList[robotID][i][0] + 1 == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] - 1 == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 7;
			//����
			else if (temGridList[robotID][i][0] + 1 == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] + 1 == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 8;
			else
				roadDir[robotID][i] = 0;


			//ˮƽ�����Ƚϴ󣬰��е�����ֱ�������,ˮƽ�ٶ�
			//if (totalCountx >= totalCounty)
			/*if((absF(traceAngleAV)<0.25*PI|| absF(traceAngleAV)>0.75*PI)&&
				(absF(robot.towardAngle[robotID])<=0.25*PI || robot.towardAngle[robotID] + PI<= 0.25 * PI|| robot.towardAngle[robotID] - PI >= -0.25 * PI))*/
			if (roadDir[robotID][i] == 1 || roadDir[robotID][i] == 3)
			{
				//�����±���
				int downcount = 1;//�±���������
				for (; downcount < borderLineWidth; downcount++)
				{
					if (temGridList[robotID][i][0] + downcount <= 99)
					{
						if (map_data[temGridList[robotID][i][0] + downcount][temGridList[robotID][i][1]] == '#')
						{
							break;
						}
					}
					//���������±���û�ϰ���������Ϊ�߽�
					else
					{
						break;
					}

				}
				int upcount = 1;//����������
				for (; upcount < borderLineWidth; upcount++)
				{
					if (temGridList[robotID][i][0] - upcount >= 0)
					{
						if (map_data[temGridList[robotID][i][0] - upcount][temGridList[robotID][i][1]] == '#')
						{
							break;
						}
					}
					//���������ұ���û�ϰ���������Ϊ�߽�
					else
					{
						break;
					}
				}
				//��ֱ�����ܵ���������
				int totalCounty = downcount + upcount;

				float upcount1, downcount1;
				//���߶�û����
				if (upcount < borderLineWidth && downcount < borderLineWidth)
				{
					findLineFlag[robotID][i] = 1;
					upcount1 = float(upcount);
					downcount1 = float(downcount);
					roadWidth[robotID][i] = (upcount1 + downcount1 - 1) * 0.5;
					if (roadDir[robotID][i] == 1)
					{
						midPoint[robotID][i][1] = (-downcount1 + upcount1) / 2.0 * 0.5 + (99 - temGridList[robotID][i][0]) * 0.5 + 0.25;
						midPoint[robotID][i][0] = temGridList[robotID][i][1] * 0.5 + 0.5;
					}
					else
					{
						midPoint[robotID][i][1] = (-downcount1 + upcount1) / 2.0 * 0.5 + (99 - temGridList[robotID][i][0]) * 0.5 + 0.25;
						midPoint[robotID][i][0] = temGridList[robotID][i][1] * 0.5 + 0;
					}
				}
				//���߶����ߣ�����Ϊ·����
				else if (upcount == borderLineWidth && downcount == borderLineWidth)
				{
					findLineFlag[robotID][i] = 0;
					//�������е����ÿ��ǰ��
					if (roadDir[robotID][i] == 1)
					{
						midPoint[robotID][i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0.25;
						midPoint[robotID][i][0] = temGridList[robotID][i][1] * 0.5 + 0.5;
					}
					else
					{
						midPoint[robotID][i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0.25;
						midPoint[robotID][i][0] = temGridList[robotID][i][1] * 0.5 + 0;
					}
					roadWidth[robotID][i] = (upcount + downcount - 1) * 0.5;
				}
				//ֻ��һ�߶�����
				else {
					if (downcount < upcount)
					{
						findLineFlag[robotID][i] = 3;//�ϱ߶���
						if (downcount<=2)
						{
							downcount1 = downcount;
							upcount1 = downcount + (2 - downcount) * 0.5 + 1;
						}
						else
						{
							downcount1 = downcount;
							upcount1 = downcount;
						}
					}
					else if (upcount < downcount)
					{
						findLineFlag[robotID][i] = 5;//�±߶���
						if(upcount<=2)
							{
								downcount1 = upcount+(2 - upcount) * 0.5 + 1;
								upcount1 = upcount;
							}
							else
							{
								downcount1 = upcount;
								upcount1 = upcount;
							}
					}
					roadWidth[robotID][i] = (upcount + downcount - 1) * 0.5;
					//�������е����ÿ��ǰ��
					if (roadDir[robotID][i] == 1)
					{
						midPoint[robotID][i][1] = (-downcount1 + upcount1) / 2.0 * 0.5 + (99 - temGridList[robotID][i][0]) * 0.5 + 0.25;
						midPoint[robotID][i][0] = temGridList[robotID][i][1] * 0.5 + 0.5;
					}
					else
					{
						midPoint[robotID][i][1] = (-downcount1 + upcount1) / 2.0 * 0.5 + (99 - temGridList[robotID][i][0]) * 0.5 + 0.25;
						midPoint[robotID][i][0] = temGridList[robotID][i][1] * 0.5 + 0;
					}

				}
			}
			//��ֱ�����Ƚϴ󣬰��е���ˮƽ�������
			else if (roadDir[robotID][i] == 2 || roadDir[robotID][i] == 4)
			{
				//�����ұ���
				int leftcount = 1;//�����������
				for (; leftcount < borderLineWidth; leftcount++)
				{
					if (temGridList[robotID][i][1] - leftcount >= 0)
					{
						if (map_data[temGridList[robotID][i][0]][temGridList[robotID][i][1] - leftcount] == '#')
						{
							break;
						}
					}
					//�������������û�ϰ���������Ϊ�߽�
					else
					{
						break;
					}

				}
				int rightcount = 1;//�ұ���������
				for (; rightcount < borderLineWidth; rightcount++)
				{
					if (temGridList[robotID][i][1] + rightcount <= 99)
					{
						if (map_data[temGridList[robotID][i][0]][temGridList[robotID][i][1] + rightcount] == '#')
						{
							break;
						}
					}
					//���������ұ���û�ϰ���������Ϊ�߽�
					else
					{
						break;
					}

				}
				//ˮƽ�����ܵ���������
				int totalCountx = leftcount + rightcount;
				float rightcount1, leftcount1;
				//���߶�û����
				if (rightcount < borderLineWidth && leftcount < borderLineWidth)
				{
					findLineFlag[robotID][i] = 1;
					rightcount1 = float(rightcount);
					leftcount1 = float(leftcount);
					roadWidth[robotID][i] = (rightcount + leftcount - 1) * 0.5;
					//�������е����ÿ��ǰ��
					if (roadDir[robotID][i] == 2)
					{
						midPoint[robotID][i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0.5;
						midPoint[robotID][i][0] = (rightcount1 - leftcount1) / 2.0 * 0.5 + temGridList[robotID][i][1] * 0.5 + 0.25;
					}
					else
					{
						midPoint[robotID][i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0;
						midPoint[robotID][i][0] = (rightcount1 - leftcount1) / 2.0 * 0.5 + temGridList[robotID][i][1] * 0.5 + 0.25;
					}
				}
				//���߶����ߣ�����Ϊ·����
				else if (rightcount == borderLineWidth && leftcount == borderLineWidth)
				{
					findLineFlag[robotID][i] = 0;
					//�������е����ÿ��ǰ��
					if (roadDir[robotID][i] == 2)
					{
						midPoint[robotID][i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0.5;
						midPoint[robotID][i][0] = temGridList[robotID][i][1] * 0.5 + 0.25;
					}
					else
					{
						midPoint[robotID][i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0;
						midPoint[robotID][i][0] = temGridList[robotID][i][1] * 0.5 + 0.25;
					}
					roadWidth[robotID][i] = (rightcount + leftcount - 1) * 0.5;
				}
				//ֻ��һ�߶�����
				else {
					if (leftcount < rightcount)
					{
						findLineFlag[robotID][i] = 2;//�ұ߶���
						if (leftcount <= 2)
						{
							leftcount1 = leftcount;
							rightcount1 = leftcount + (2 - leftcount) * 0.5 + 1;//������΢ƫ��һ��
						}
						else
						{
							leftcount1 = leftcount;
							rightcount1 = leftcount;
						}
					}
					else if (rightcount < leftcount)
					{
						findLineFlag[robotID][i] = 4;//��߶���
						if (rightcount<=2)
						{
							leftcount1 = rightcount+(2- rightcount)*0.5+1;
							rightcount = rightcount;//������΢ƫ��һ��
						}
						else
						{
							leftcount1 = rightcount;
							rightcount1 = rightcount;
						}
					}
					roadWidth[robotID][i] = (rightcount + leftcount - 1) * 0.5;
					//�������е����ÿ��ǰ��
					if (roadDir[robotID][i] == 2)
					{
						midPoint[robotID][i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0.5;
						midPoint[robotID][i][0] = (rightcount1 - leftcount1) / 2.0 * 0.5 + temGridList[robotID][i][1] * 0.5 + 0.25;
					}
					else
					{
						midPoint[robotID][i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0;
						midPoint[robotID][i][0] = (rightcount1 - leftcount1) / 2.0 * 0.5 + temGridList[robotID][i][1] * 0.5 + 0.25;
					}
				}
			}
			//���������ٶ�
			else if (roadDir[robotID][i] == 5 || roadDir[robotID][i] == 7)
			{
				//���������±���
				int leftupcount = 1;//�����������
				for (; leftupcount < borderLineWidth; leftupcount++)
				{
					if (temGridList[robotID][i][1] - leftupcount >= 0 && temGridList[robotID][i][0] - leftupcount >= 0)
					{
						if (map_data[temGridList[robotID][i][0] - leftupcount][temGridList[robotID][i][1] - leftupcount] == '#')
						{
							break;
						}
					}
					//�������������û�ϰ���������Ϊ�߽�
					else
					{
						break;
					}

				}
				int rightdowncount = 1;//�ұ���������
				for (; rightdowncount < borderLineWidth; rightdowncount++)
				{
					if (temGridList[robotID][i][1] + rightdowncount <= 99 && temGridList[robotID][i][0] + rightdowncount <= 99)
					{
						if (map_data[temGridList[robotID][i][0] + rightdowncount][temGridList[robotID][i][1] + rightdowncount] == '#')
						{
							break;
						}
					}
					//���������ұ���û�ϰ���������Ϊ�߽�
					else
					{
						break;
					}

				}
				//ˮƽ�����ܵ���������
				int totalCount57 = leftupcount + rightdowncount;
				float rightdowncount1, leftupcount1;
				//���߶�û����
				if (leftupcount < borderLineWidth && rightdowncount < borderLineWidth)
				{
					findLineFlag[robotID][i] = 1;
					rightdowncount1 = float(rightdowncount);
					leftupcount1 = float(leftupcount);
					roadWidth[robotID][i] = (rightdowncount1 + leftupcount1 - 1) * 0.5*sqrt(2);
					if (roadDir[robotID][i] == 5)
					{

						midPoint[robotID][i][1] = (-rightdowncount1 + leftupcount1) / 2.0 * 0.5 + (99 - temGridList[robotID][i][0]) * 0.5 + 0.5;
						midPoint[robotID][i][0] = (rightdowncount1 - leftupcount1) / 2.0 * 0.5 + temGridList[robotID][i][1] * 0.5 + 0.5;
					}
					else
					{
						midPoint[robotID][i][1] = (-rightdowncount1 + leftupcount1) / 2.0 * 0.5 + (99 - temGridList[robotID][i][0]) * 0.5 + 0;
						midPoint[robotID][i][0] = (rightdowncount1 - leftupcount1) / 2.0 * 0.5 + temGridList[robotID][i][1] * 0.5 + 0;
					}
				}
				//���߶����ߣ�����Ϊ·����
				else if (rightdowncount == borderLineWidth && leftupcount == borderLineWidth)
				{
					findLineFlag[robotID][i] = 0;
					if (roadDir[robotID][i] == 5)
					{

						midPoint[robotID][i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0.5;
						midPoint[robotID][i][0] = temGridList[robotID][i][1] * 0.5 + 0.5;
					}
					else
					{
						midPoint[robotID][i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0;
						midPoint[robotID][i][0] = temGridList[robotID][i][1] * 0.5 + 0;
					}
					roadWidth[robotID][i] = (leftupcount + rightdowncount - 1) * 0.5* sqrt(2);
				}
				//ֻ��һ�߶�����
				else {
					if (leftupcount < rightdowncount)
					{
						findLineFlag[robotID][i] = 6;//���¶���
						leftupcount1 = leftupcount;
						rightdowncount1 = rightdowncount;
					}
					else if (rightdowncount < leftupcount)
					{
						findLineFlag[robotID][i] = 8;//���϶���
						rightdowncount1 = rightdowncount;
						leftupcount1 = leftupcount;
					}
					roadWidth[robotID][i] = (leftupcount + rightdowncount - 1) * 0.5 * sqrt(2);
					if (roadDir[robotID][i] == 5)
					{

						midPoint[robotID][i][1] = (-rightdowncount1 + leftupcount1) / 2.0 * 0.5 + (99 - temGridList[robotID][i][0]) * 0.5 + 0.5;
						midPoint[robotID][i][0] = (rightdowncount1 - leftupcount1) / 2.0 * 0.5 + temGridList[robotID][i][1] * 0.5 + 0.5;
					}
					else
					{
						midPoint[robotID][i][1] = (-rightdowncount1 + leftupcount1) / 2.0 * 0.5 + (99 - temGridList[robotID][i][0]) * 0.5 + 0;
						midPoint[robotID][i][0] = (rightdowncount1 - leftupcount1) / 2.0 * 0.5 + temGridList[robotID][i][1] * 0.5 + 0;
					}
				}
			}
			//���������ٶ�
			else if (roadDir[robotID][i] == 6 || roadDir[robotID][i] == 8)
			{
				//���������±���
				int leftdowncount = 1;//�����������
				for (; leftdowncount < borderLineWidth; leftdowncount++)
				{
					if (temGridList[robotID][i][1] + leftdowncount <= 99 && temGridList[robotID][i][0] - leftdowncount >= 0)
					{
						if (map_data[temGridList[robotID][i][0] - leftdowncount][temGridList[robotID][i][1] + leftdowncount] == '#')
						{
							break;
						}
					}
					//�������������û�ϰ���������Ϊ�߽�
					else
					{
						break;
					}

				}
				int rightupcount = 1;//�ұ���������
				for (; rightupcount < borderLineWidth; rightupcount++)
				{
					if (temGridList[robotID][i][1] - rightupcount >= 0 && temGridList[robotID][i][0] + rightupcount <= 99)
					{
						if (map_data[temGridList[robotID][i][0] + rightupcount][temGridList[robotID][i][1] - rightupcount] == '#')
						{
							break;
						}
					}
					//���������ұ���û�ϰ���������Ϊ�߽�
					else
					{
						break;
					}

				}
				//ˮƽ�����ܵ���������
				int totalCount68 = leftdowncount + rightupcount;
				float rightupcount1, leftdowncount1;
				//���߶�û����
				if (leftdowncount < borderLineWidth && rightupcount < borderLineWidth)
				{
					findLineFlag[robotID][i] = 1;
					rightupcount1 = float(rightupcount);
					leftdowncount1 = float(leftdowncount);
					roadWidth[robotID][i] = (rightupcount1 + leftdowncount1 - 1) * sqrt(2) * 0.5;
					if (roadDir[robotID][i] == 6)
					{
						midPoint[robotID][i][1] = (rightupcount1 - leftdowncount1) / 2.0 * 0.5 + (99 - temGridList[robotID][i][0]) * 0.5 + 0.25;
						midPoint[robotID][i][0] = (rightupcount1 - leftdowncount1) / 2.0 * 0.5 + temGridList[robotID][i][1] * 0.5 - 0.25;
					}
					else
					{
						midPoint[robotID][i][1] = (rightupcount1 - leftdowncount1) / 2.0 * 0.5 + (99 - temGridList[robotID][i][0]) * 0.5 - 0.25;
						midPoint[robotID][i][0] = (rightupcount1 - leftdowncount1) / 2.0 * 0.5 + temGridList[robotID][i][1] * 0.5 + 0.25;
					}
				}
				//���߶����ߣ�����Ϊ·����
				else if (rightupcount == borderLineWidth && leftdowncount == borderLineWidth)
				{
					findLineFlag[robotID][i] = 0;
					if (roadDir[robotID][i] == 6)
					{
						midPoint[robotID][i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0.25;
						midPoint[robotID][i][0] = temGridList[robotID][i][1] * 0.5 - 0.25;
					}
					else
					{
						midPoint[robotID][i][1] = (99 - temGridList[robotID][i][0]) * 0.5 - 0.25;
						midPoint[robotID][i][0] = temGridList[robotID][i][1] * 0.5 + 0.25;
					}
					roadWidth[robotID][i] = (rightupcount + leftdowncount - 1) * sqrt(2) * 0.5;
				}
				//ֻ��һ�߶�����
				else {
					if (leftdowncount < rightupcount)
					{
						findLineFlag[robotID][i] = 7;//���϶���
						leftdowncount1 = leftdowncount;
						rightupcount1 = rightupcount;
					}
					else if (rightupcount < leftdowncount)
					{
						findLineFlag[robotID][i] = 9;//���¶���
						rightupcount1 = rightupcount;
						leftdowncount1 = leftdowncount;
					}
					roadWidth[robotID][i] = (rightupcount + leftdowncount - 1) * sqrt(2) * 0.5;
					if (roadDir[robotID][i] == 6)
					{
						midPoint[robotID][i][1] = (rightupcount1 - leftdowncount1) / 2.0 * 0.5 + (99 - temGridList[robotID][i][0]) * 0.5 + 0.25;
						midPoint[robotID][i][0] = (rightupcount1 - leftdowncount1) / 2.0 * 0.5 + temGridList[robotID][i][1] * 0.5 - 0.25;
					}
					else
					{
						midPoint[robotID][i][1] = (rightupcount1 - leftdowncount1) / 2.0 * 0.5 + (99 - temGridList[robotID][i][0]) * 0.5 - 0.25;
						midPoint[robotID][i][0] = (rightupcount1 - leftdowncount1) / 2.0 * 0.5 + temGridList[robotID][i][1] * 0.5 + 0.25;
					}
				}
			}
		}
	}
}
