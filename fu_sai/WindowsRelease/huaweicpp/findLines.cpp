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
 * Input: 机器人序号,搜索格子个数半径：2，代表2.5X2.5范围
 * Return: 0,代表没找到，视为周围无障碍
 * Description: 判断机器人周围是否存在障碍
 * Author: 余游,2023/3/29
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
 * Input: 机器人序号
 * Return:
 * Description: 找左边线，右边线，和中线
 * Author: 余游,2023/3/31
 */
void findBorderLine(int robotID)
{
	int  deskid = aimDesk[robotID];
	//获得机器人到目标工作台路径列表
	std::list<Node_c> temlist;

	//判断一下路径冲突 
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
	//if(temlist.size() > 0) temlist.pop_front(); //将第一个点 也就是当前在的点pop掉
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
	//将每个路径点格子坐标存到temGridList
	int count = 0;
	//获得路径点数组
	for (Node_c test : temlist)
	{
		temGridList[robotID][count][0] = test.point.x;
		temGridList[robotID][count][1] = test.point.y;
		count++;
		if (count == realBoderGridNum[robotID])
			break;
	}


	//超过2的时候起点和终点不连通
	if (realBoderGridNum[robotID] > 2)
	{
		for (int i = 0; i < realBoderGridNum[robotID]; i++)
		{
			temGridPoint[i][0] = temGridList[robotID][i][1] * 0.5 + 0.25;
			temGridPoint[i][1] = (99 - temGridList[robotID][i][0]) * 0.5 + 0.25;
			//判断路径方向
			//往右
			if (temGridList[robotID][i][0] == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] + 1 == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 1;
			//往左
			else if (temGridList[robotID][i][0] == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] - 1 == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 3;
			//往上
			else if (temGridList[robotID][i][0] - 1 == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 2;
			//往下
			else if (temGridList[robotID][i][0] + 1 == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 4;
			//右上
			else if (temGridList[robotID][i][0] - 1 == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] + 1 == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 5;
			//左上
			else if (temGridList[robotID][i][0] - 1 == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] - 1 == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 6;
			//左下
			else if (temGridList[robotID][i][0] + 1 == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] - 1 == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 7;
			//右下
			else if (temGridList[robotID][i][0] + 1 == temGridList[robotID][i + 1][0] && (temGridList[robotID][i][1] + 1 == temGridList[robotID][i + 1][1]))
				roadDir[robotID][i] = 8;
			else
				roadDir[robotID][i] = 0;


			//水平方向宽度较大，把中点往竖直方向调整,水平速度
			//if (totalCountx >= totalCounty)
			/*if((absF(traceAngleAV)<0.25*PI|| absF(traceAngleAV)>0.75*PI)&&
				(absF(robot.towardAngle[robotID])<=0.25*PI || robot.towardAngle[robotID] + PI<= 0.25 * PI|| robot.towardAngle[robotID] - PI >= -0.25 * PI))*/
			if (roadDir[robotID][i] == 1 || roadDir[robotID][i] == 3)
			{
				//找上下边线
				int downcount = 1;//下边搜索步数
				for (; downcount < borderLineWidth; downcount++)
				{
					if (temGridList[robotID][i][0] + downcount <= 99)
					{
						if (map_data[temGridList[robotID][i][0] + downcount][temGridList[robotID][i][1]] == '#')
						{
							break;
						}
					}
					//搜索到最下边了没障碍物以最下为边界
					else
					{
						break;
					}

				}
				int upcount = 1;//上搜索步数
				for (; upcount < borderLineWidth; upcount++)
				{
					if (temGridList[robotID][i][0] - upcount >= 0)
					{
						if (map_data[temGridList[robotID][i][0] - upcount][temGridList[robotID][i][1]] == '#')
						{
							break;
						}
					}
					//搜索到最右边了没障碍物以最右为边界
					else
					{
						break;
					}
				}
				//竖直方向总的搜索步长
				int totalCounty = downcount + upcount;

				float upcount1, downcount1;
				//两边都没丢线
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
				//两边都丢线，中线为路径点
				else if (upcount == borderLineWidth && downcount == borderLineWidth)
				{
					findLineFlag[robotID][i] = 0;
					//尽量把中点放在每格前面
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
				//只有一边丢线了
				else {
					if (downcount < upcount)
					{
						findLineFlag[robotID][i] = 3;//上边丢线
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
						findLineFlag[robotID][i] = 5;//下边丢线
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
					//尽量把中点放在每格前面
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
			//竖直方向宽度较大，把中点往水平方向调整
			else if (roadDir[robotID][i] == 2 || roadDir[robotID][i] == 4)
			{
				//找左右边线
				int leftcount = 1;//左边搜索步数
				for (; leftcount < borderLineWidth; leftcount++)
				{
					if (temGridList[robotID][i][1] - leftcount >= 0)
					{
						if (map_data[temGridList[robotID][i][0]][temGridList[robotID][i][1] - leftcount] == '#')
						{
							break;
						}
					}
					//搜索到最左边了没障碍物以最左为边界
					else
					{
						break;
					}

				}
				int rightcount = 1;//右边搜索步数
				for (; rightcount < borderLineWidth; rightcount++)
				{
					if (temGridList[robotID][i][1] + rightcount <= 99)
					{
						if (map_data[temGridList[robotID][i][0]][temGridList[robotID][i][1] + rightcount] == '#')
						{
							break;
						}
					}
					//搜索到最右边了没障碍物以最右为边界
					else
					{
						break;
					}

				}
				//水平方向总的搜索步长
				int totalCountx = leftcount + rightcount;
				float rightcount1, leftcount1;
				//两边都没丢线
				if (rightcount < borderLineWidth && leftcount < borderLineWidth)
				{
					findLineFlag[robotID][i] = 1;
					rightcount1 = float(rightcount);
					leftcount1 = float(leftcount);
					roadWidth[robotID][i] = (rightcount + leftcount - 1) * 0.5;
					//尽量把中点放在每格前面
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
				//两边都丢线，中线为路径点
				else if (rightcount == borderLineWidth && leftcount == borderLineWidth)
				{
					findLineFlag[robotID][i] = 0;
					//尽量把中点放在每格前面
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
				//只有一边丢线了
				else {
					if (leftcount < rightcount)
					{
						findLineFlag[robotID][i] = 2;//右边丢线
						if (leftcount <= 2)
						{
							leftcount1 = leftcount;
							rightcount1 = leftcount + (2 - leftcount) * 0.5 + 1;//过近稍微偏移一点
						}
						else
						{
							leftcount1 = leftcount;
							rightcount1 = leftcount;
						}
					}
					else if (rightcount < leftcount)
					{
						findLineFlag[robotID][i] = 4;//左边丢线
						if (rightcount<=2)
						{
							leftcount1 = rightcount+(2- rightcount)*0.5+1;
							rightcount = rightcount;//过近稍微偏移一点
						}
						else
						{
							leftcount1 = rightcount;
							rightcount1 = rightcount;
						}
					}
					roadWidth[robotID][i] = (rightcount + leftcount - 1) * 0.5;
					//尽量把中点放在每格前面
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
			//右上左下速度
			else if (roadDir[robotID][i] == 5 || roadDir[robotID][i] == 7)
			{
				//找左上右下边线
				int leftupcount = 1;//左边搜索步数
				for (; leftupcount < borderLineWidth; leftupcount++)
				{
					if (temGridList[robotID][i][1] - leftupcount >= 0 && temGridList[robotID][i][0] - leftupcount >= 0)
					{
						if (map_data[temGridList[robotID][i][0] - leftupcount][temGridList[robotID][i][1] - leftupcount] == '#')
						{
							break;
						}
					}
					//搜索到最左边了没障碍物以最左为边界
					else
					{
						break;
					}

				}
				int rightdowncount = 1;//右边搜索步数
				for (; rightdowncount < borderLineWidth; rightdowncount++)
				{
					if (temGridList[robotID][i][1] + rightdowncount <= 99 && temGridList[robotID][i][0] + rightdowncount <= 99)
					{
						if (map_data[temGridList[robotID][i][0] + rightdowncount][temGridList[robotID][i][1] + rightdowncount] == '#')
						{
							break;
						}
					}
					//搜索到最右边了没障碍物以最右为边界
					else
					{
						break;
					}

				}
				//水平方向总的搜索步长
				int totalCount57 = leftupcount + rightdowncount;
				float rightdowncount1, leftupcount1;
				//两边都没丢线
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
				//两边都丢线，中线为路径点
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
				//只有一边丢线了
				else {
					if (leftupcount < rightdowncount)
					{
						findLineFlag[robotID][i] = 6;//右下丢线
						leftupcount1 = leftupcount;
						rightdowncount1 = rightdowncount;
					}
					else if (rightdowncount < leftupcount)
					{
						findLineFlag[robotID][i] = 8;//左上丢线
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
			//右上左下速度
			else if (roadDir[robotID][i] == 6 || roadDir[robotID][i] == 8)
			{
				//找右上左下边线
				int leftdowncount = 1;//左边搜索步数
				for (; leftdowncount < borderLineWidth; leftdowncount++)
				{
					if (temGridList[robotID][i][1] + leftdowncount <= 99 && temGridList[robotID][i][0] - leftdowncount >= 0)
					{
						if (map_data[temGridList[robotID][i][0] - leftdowncount][temGridList[robotID][i][1] + leftdowncount] == '#')
						{
							break;
						}
					}
					//搜索到最左边了没障碍物以最左为边界
					else
					{
						break;
					}

				}
				int rightupcount = 1;//右边搜索步数
				for (; rightupcount < borderLineWidth; rightupcount++)
				{
					if (temGridList[robotID][i][1] - rightupcount >= 0 && temGridList[robotID][i][0] + rightupcount <= 99)
					{
						if (map_data[temGridList[robotID][i][0] + rightupcount][temGridList[robotID][i][1] - rightupcount] == '#')
						{
							break;
						}
					}
					//搜索到最右边了没障碍物以最右为边界
					else
					{
						break;
					}

				}
				//水平方向总的搜索步长
				int totalCount68 = leftdowncount + rightupcount;
				float rightupcount1, leftdowncount1;
				//两边都没丢线
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
				//两边都丢线，中线为路径点
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
				//只有一边丢线了
				else {
					if (leftdowncount < rightupcount)
					{
						findLineFlag[robotID][i] = 7;//右上丢线
						leftdowncount1 = leftdowncount;
						rightupcount1 = rightupcount;
					}
					else if (rightupcount < leftdowncount)
					{
						findLineFlag[robotID][i] = 9;//左下丢线
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
