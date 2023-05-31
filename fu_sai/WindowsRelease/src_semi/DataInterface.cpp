#define  _CRT_SECURE_NO_WARNINGS
#include"globalVariables.h"
char map_data[100][100];
char map_data_ob_expand_sell[100][100];
char map_data_ob_expand_buy[100][100];
char map_data_to_look_path[100][100];
bool is_expand_or_set_sell[100][100] = { false };//当前位置是障碍物 将其周围四个格子都扩展成障碍物
bool is_expand_or_set_buy[100][100] = { false };
double map_data_cost[100][100] = { 0 }; //距离障碍物越近 代价越大
double map_coor_axis_revised[100][100][2] = { 0 }; //grid格子中心坐标矫正

DeskData workDesk;
RobotData robot;
int frameID; 
int hasMoney;
int deskNum=0;
int robotNum=4;


void expand_ob_sell(int i, int j) {

	//for (int row = i - 1; row <= i + 1; row++) {
	//	for (int col = j - 1; col <= j + 1; col++) {
	//		if (row < 0 || row > 99 || col < 0 || col > 100) continue;
	//		if (row == i && col == j) continue;
	//		if (!is_expand_or_set_sell[row][col] && map_data[row][col] == '.') { //有一种情况 卖东西的工作台在墙角 有两个障碍物夹着他 他就不能去被卖东西
	//			map_data_ob_expand_sell[row][col] = '#';
	//			is_expand_or_set_sell[row][col] = true;
	//		}
	//	}
	//}

	if (i - 1 >= 0 && !is_expand_or_set_sell[i - 1][j] && map_data[i - 1][j] == '.') {
		map_data_ob_expand_sell[i - 1][j] = '#';
		is_expand_or_set_sell[i - 1][j] = true;
	}
	if (i+ 1 < 100 && !is_expand_or_set_sell[i + 1][j] && map_data[i + 1][j] == '.') {
		map_data_ob_expand_sell[i + 1][j] = '#';
		is_expand_or_set_sell[i + 1][j] = true;
	}
	if (j - 1 >= 0 && !is_expand_or_set_sell[i][j - 1] && map_data[i][j - 1] == '.') {
		map_data_ob_expand_sell[i][j - 1] = '#';
		is_expand_or_set_sell[i][j - 1] = true;
	}
	if(j + 1 < 100 && !is_expand_or_set_sell[i][j + 1] && map_data[i][j + 1] == '.') {
		map_data_ob_expand_sell[i][j + 1] = '#';
		is_expand_or_set_sell[i][j + 1] = true;
	}
}

void expand_ob_buy(int i, int j) {

	/*if (i - 3 >= 0 && !is_expand_or_set_buy[i - 1][j] && map_data[i - 1][j] == '.'&& !(map_data[i - 2][j] == '.'&& map_data[i - 3][j] == '#')) {
		map_data_ob_expand_buy[i - 1][j] = '#';
		is_expand_or_set_buy[i - 1][j] = true;
	}
	if (i + 3 < 100 && !is_expand_or_set_buy[i + 1][j] && map_data[i + 1][j] == '.' &&!(map_data[i +2][j] == '.' && map_data[i +3][j] == '#')) {
		map_data_ob_expand_buy[i + 1][j] = '#';
		is_expand_or_set_buy[i + 1][j] = true;
	}
	if (j - 3 >= 0 && !is_expand_or_set_buy[i][j - 1] && map_data[i][j - 1] == '.' && !(map_data[i][j-2] == '.' && map_data[i][j-3] == '#')) {
		map_data_ob_expand_buy[i][j - 1] = '#';
		is_expand_or_set_buy[i][j - 1] = true;
	}
	if (j + 3< 100 && !is_expand_or_set_buy[i][j + 1] && map_data[i][j + 1] == '.' && !(map_data[i][j +2] == '.' && map_data[i][j +3] == '#')) {
		map_data_ob_expand_buy[i][j + 1] = '#';
		is_expand_or_set_buy[i][j + 1] = true;
	}*/
	if (j + 1 < 100 && !is_expand_or_set_buy[i][j + 1] && map_data[i][j + 1] == '.') {
		map_data_ob_expand_buy[i][j + 1] = '#';
		is_expand_or_set_buy[i][j + 1] = true;
	}
	if (i - 1 >= 0 && !is_expand_or_set_buy[i - 1][j] && map_data[i - 1][j] == '.') {
		map_data_ob_expand_buy[i - 1][j] = '#';
		is_expand_or_set_buy[i - 1][j] = true;
	}
	if (i - 1 >= 0 && j + 1 < 100 && !is_expand_or_set_buy[i - 1][j + 1] && map_data[i - 1][j + 1] == '.') {
		map_data_ob_expand_buy[i - 1][j + 1] = '#';
		is_expand_or_set_buy[i - 1][j + 1] = true;
	}

	/*
	if (i - 1 >= 0 && !is_expand_or_set_buy[i - 1][j] && map_data[i - 1][j] == '.') {
		map_data_ob_expand_buy[i - 1][j] = '#';
		is_expand_or_set_buy[i - 1][j] = true;
	}
	if (i+ 1 < 100 && !is_expand_or_set_buy[i + 1][j] && map_data[i + 1][j] == '.') {
		map_data_ob_expand_buy[i + 1][j] = '#';
		is_expand_or_set_buy[i + 1][j] = true;
	}
	if (j - 1 >= 0 && !is_expand_or_set_buy[i][j - 1] && map_data[i][j - 1] == '.') {
		map_data_ob_expand_buy[i][j - 1] = '#';
		is_expand_or_set_buy[i][j - 1] = true;
	}
	if(j + 1 < 100 && !is_expand_or_set_buy[i][j + 1] && map_data[i][j + 1] == '.') {
		map_data_ob_expand_buy[i][j + 1] = '#';
		is_expand_or_set_buy[i][j + 1] = true;
	}*/
}


/**
 * FunctionName: bool readMap()
 * Input: 无
 * Return: 读取OK返回true
 * Description: 读取地图
 * Author: 余游,2023/3/11
 */
bool readMap() {

	for (int i = 0; i < 100; i++) {
		memset(is_expand_or_set_sell[i], false, sizeof is_expand_or_set_sell[i]);
		memset(is_expand_or_set_buy[i], false, sizeof is_expand_or_set_buy[i]);
		memset(map_data_cost[i], 1, sizeof map_data_cost[i]);
	}


	char line[1024];
	int j = 0;

	while (fgets(line, sizeof line, stdin)) {
		if (line[0] == 'O' && line[1] == 'K') {
			break;
		}
		else
		{
			for (int i = 0; i < 100; i++) {
				bench_seq_map[j][i] = -1;
				map_data[j][i] = line[i];
				map_data_ob_expand_sell[j][i] = line[i];
				map_data_ob_expand_buy[j][i] = line[i];
			}
		}
		j++;
	}

	for (int i = 0; i < 100; i++) {
		for (int j = 0; j < 100; j++) {

			map_coor_axis_revised[i][j][0] = j * 0.5 + 0.25;
			map_coor_axis_revised[i][j][1] = (99 - i) * 0.5 + 0.25;


			if (!is_expand_or_set_sell[i][j] && map_data[i][j] == '#') {//障碍物地图扩展
				is_expand_or_set_sell[i][j] = true;
				expand_ob_sell(i, j);
			}

			if (!is_expand_or_set_buy[i][j] && map_data[i][j] == '#') {//障碍物地图扩展
				is_expand_or_set_buy[i][j] = true;
				expand_ob_buy(i, j);
			}

			if (map_data[i][j] == '#') { //将障碍物周围2格代价设大点
				for (int row = i - 2; row < i + 2; row++) {
					for (int col = j - 2; col < j + 2; col++) {
						if (row < 0 || row > 99 || col < 0 || col > 99) continue;
						int dist2 = absI(row - i) + absI(col - j);
						if (dist2 <= 1)
							map_data_cost[row][col] = maxI(map_data_cost[row][col], 3); //先设置个5看看 15目前效果比较好
						else if (dist2 <= 2)
							map_data_cost[row][col] = maxI(map_data_cost[row][col], 2);
						else if (dist2 <= 3)
							map_data_cost[row][col] = maxI(map_data_cost[row][col], 1);
					}
				}

				//校正障碍物周边空地实际坐标
				for (int row = -1; row <= 1; row++) {
					for (int col = -1; col <= 1; col++) {
						if (i + row < 0 || i + row > 99 || j + col < 0 || j + col > 99) continue;
						if (map_data[i + row][j + col] >= '1' && map_data[i + row][j + col] <= '9') continue;
						map_coor_axis_revised[i + row][j + col][0] += col * 0.25;
						map_coor_axis_revised[i + row][j + col][1] += -row * 0.25;
					}
				}
			}
		}
	}

	//std::fstream f;
	//f.open("data.txt", std::ios::out);

	//for (int i = 0; i < 100; i++) {
	//	for (int j = 0; j < 100; j++) {
	//		f << char(obstacles[i][j]);
	//	}
	//	f << "\n";
	//}
	//f.close();

	return true;
}
/**
 * FunctionName: bool readMap()
 * Input: 无
 * Return: 读取到OK返回true
 * Description: 读取状态信息写入workDesk、robot结构体
 * Author: 余游,2023/3/11
 */
bool readState() {
    int j = 0;
  //读取前两行
    if (j == 0)
    {
        scanf("%d", &hasMoney);
        j++;
    }
     if (j == 1)
    { 
        scanf("%d", &deskNum);
        j++;
    }
 
    //读取OK之前的行
    while (j < deskNum + 2+ robotNum) {
        if (j < 2 + deskNum)
        {
            scanf("%d%lf%lf%d%d%d", &workDesk.deskType[j - 2], 
                &workDesk.local[j - 2][0],
                &workDesk.local[j - 2][1]
                , &workDesk.waitTime[j - 2],
                &workDesk.materialState[j - 2], 
                &workDesk.productState[j - 2]);
        }
        else {
            scanf("%d%d%lf%lf%lf%lf%lf%lf%lf%lf", &robot.deskID[j - 2 - deskNum]
            , &robot.productType[j - 2 - deskNum]
            , &robot.timeC[j - 2 - deskNum]
            , &robot.collC[j - 2 - deskNum]
            , &robot.angleSpeed[j - 2 - deskNum]
            , &robot.lineSpeed[j - 2 - deskNum][0]
            , &robot.lineSpeed[j - 2 - deskNum][1]
            , &robot.towardAngle[j - 2 - deskNum]
            , &robot.local[j - 2 - deskNum][0]
            , &robot.local[j - 2 - deskNum][1]);

			robot.grid[j - 2 - deskNum][0] = int(robot.local[j - 2 - deskNum][0] - 0.25) * 2;
			robot.grid[j - 2 - deskNum][1] = 99 - int(robot.local[j - 2 - deskNum][1] - 0.25) * 2;
        }
        j++;
    }

	if (frameID == 1) { 
		//根据场上工作台数量构造材料需求表
		material_demand[4] =1;
		material_demand[5] = 1;
		material_demand[6] = 1;
		material_demand[1] =  1;
		material_demand[2] = 1;
		material_demand[3] =  1;

		for (int seq = 0; seq < deskNum; seq++) {
			if (roborts_can_go_bench_sell_array[0][seq]) {
				int type = workDesk.deskType[seq];
				switch (type)
				{
				case 4:
					material_demand[1] += 1;
					material_demand[2] += 1;
					break;
				case 5:
					material_demand[1] += 1;
					material_demand[3] += 1;
					break;
				case 6:
					material_demand[2] += 1;
					material_demand[3] += 1;
					break;
				case 7:
					material_demand[4] += 1;
					material_demand[5] += 1;
					material_demand[5] += 1;
					break;
				default:
					break;
				}
			}
		}

		//构造工作台之间 连接数量关系
		for (int i = 0; i < deskNum; i++) {
			int type = workDesk.deskType[i];
			if (type == 4) {
				bench_in_edge_num[i] = infor_working_benchs_dict[1].size() + infor_working_benchs_dict[2].size();
				if (infor_working_benchs_dict[7].size() > 0)
					bench_out_edge_num[i] = infor_working_benchs_dict[7].size();
				else
					bench_out_edge_num[i] = infor_working_benchs_dict[9].size();
			}
			else if (type == 5) {
				bench_in_edge_num[i] = infor_working_benchs_dict[1].size() + infor_working_benchs_dict[3].size();
				if (infor_working_benchs_dict[7].size() > 0)
					bench_out_edge_num[i] = infor_working_benchs_dict[7].size();
				else
					bench_out_edge_num[i] = infor_working_benchs_dict[9].size();
			}
			else if (type == 6) {
				bench_in_edge_num[i] = infor_working_benchs_dict[2].size() + infor_working_benchs_dict[3].size();
				if (infor_working_benchs_dict[7].size() > 0)
					bench_out_edge_num[i] = infor_working_benchs_dict[7].size();
				else
					bench_out_edge_num[i] = infor_working_benchs_dict[9].size();
			}
			else if (type == 7) {
				bench_in_edge_num[i] = infor_working_benchs_dict[4].size() + infor_working_benchs_dict[5].size() + infor_working_benchs_dict[6].size();
				bench_out_edge_num[i] = infor_working_benchs_dict[8].size() + infor_working_benchs_dict[9].size();
			}
			else if (type == 1) {
				bench_out_edge_num[i] = infor_working_benchs_dict[4].size() + infor_working_benchs_dict[5].size();
			}
			else if (type == 2) {
				bench_out_edge_num[i] = infor_working_benchs_dict[4].size() + infor_working_benchs_dict[6].size();
			}
			else if (type == 3) {
				bench_out_edge_num[i] = infor_working_benchs_dict[5].size() + infor_working_benchs_dict[6].size();
			}
		}
	}

    while (j >=deskNum + 2+ robotNum) {
      char line[10];
      fgets(line, sizeof line, stdin);
      if (line[0] == 'O' && line[1] == 'K') {
          //workDesk.local[DeskMaxNum-1][0] = 25;
          //workDesk.local[DeskMaxNum-1][1] = 25;
        return true;
        }
       j++;
    }
    return false;
}
/**
 * FunctionName: void outCMD(void);
 * Input: 无
 * Return: 无
 * Description: 输出控制指令
 * Author: 余游,2023/3/12
 */
void outCMD(void)
{
    for (int i = 0; i < RobotNum; i++)
    {
        //速度控制
        lastSetSpeed[i] = setLineSpeed[i];
        lastSetAngleSpeed[i] = setAngleSpeed[i];
        printf("forward %d %lf\n", i, setLineSpeed[i]);
        printf("rotate %d %lf\n", i, setAngleSpeed[i]);
        //动作控制
        ;
    }
    printf("OK\n");
    fflush(stdout);
}
