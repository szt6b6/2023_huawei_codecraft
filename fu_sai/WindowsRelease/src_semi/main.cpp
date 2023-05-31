#define _CRT_SECURE_NO_WARNINGS
#include"globalVariables.h"
#include<Windows.h>

int main(int argc, char** argv) {
	//初始化

	LPCTSTR lpszUnicode = "Test String";
	MessageBox(NULL, lpszUnicode, lpszUnicode, MB_YESNO);

	//time_t start_t, end_t;
	//double diff_t;
	//start_t = clock();

	for (int i = 0; i < 50; i++)
		memset(sell_benchs_in_queue[i], false, sizeof(sell_benchs_in_queue[i]));
	memset(buy_benchs_in_queue, 0, sizeof(buy_benchs_in_queue));

	//to_sell_need_frames_error = atof(argv[0]); // -10, 40 s = 2
	//to_buy_need_frames_error = atof(argv[1]); // -10, 30 s=2
	//sell_bench_logical_dist_estimate = atof(argv[3]); // -150 0 150
	//producing_status_weight = atof(argv[4]); // -50 50 20
	//limit_bench_to_buy = atof(argv[5]); // 7\8
	//average_speed = atof(argv[6]); //0.10 0.11 0.12
	//bench_out_edge_num_weight = atof(argv[7]); //-1 1 s=0.1
	//bench_in_edge_num_weight = atof(argv[8]); //-1 1 s=0.1
	//material_demand_weight = atof(argv[9]); //-10 10 2

	//进行工作台之间路径搜索 机器人可达工作台收集 附加到进程进行调试
	readMap();
	roborts_can_go_bench_map.clear();
	init_bench_robort_seq_map();
	BFSTrave();
	for (int i = 0; i < 4; i++) {
		for (int bench_seq : roborts_can_go_bench_map[i]) {
			roborts_can_go_bench_buy_array[i][bench_seq] = true;
			roborts_can_go_bench_sell_array[i][bench_seq] = true;
		}
	}

	//让夹在墙中间的工作台不能去卖东西
	for (int seq = 0; seq < init_bench_sum; seq++) {
		int row = bench_seq_dist[seq][0], col = bench_seq_dist[seq][1];
		int count_have_ob = 0;
		if (row - 1 >= 0 && map_data[row - 1][col] == '#')count_have_ob++;
		if (row + 1 >= 0 && map_data[row + 1][col] == '#')count_have_ob++;
		if (col - 1 >= 0 && map_data[row][col - 1] == '#')count_have_ob++;
		if (col + 1 >= 0 && map_data[row][col + 1] == '#')count_have_ob++;
		if (count_have_ob > 1)
			for (int i = 0; i < 4; i++)
				roborts_can_go_bench_sell_array[i][seq] = false;
	}
	init_bench2bench_paths(); //bench2bench_paths_buy中收集工作台之间的路径

	//end_t = clock();
	//diff_t = difftime(end_t, start_t);
	//fprintf(stderr, "执行时间 = %f\n", diff_t);
	//update_queue_init();

	puts("OK");
	fflush(stdout);

	while (scanf("%d", &frameID) != EOF) {
		//读取地图信息
		readState();


		//计算机器人状态参数
		calculateRobotState();
		//eventDeal();
		//获得目标工作台和动作序列
		update_queue();

		printf("%d\n", frameID);

		run_task();

		//全局速度控制
		speedControl();
		//避障算法
		avoidOb();
		//输出指令
		outCMD();
	}
	return 0;
}
