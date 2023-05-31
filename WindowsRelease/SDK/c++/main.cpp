#define _CRT_SECURE_NO_WARNINGS
#include"globalVariables.h"
#include<Windows.h>

int main(int argc, char** argv) {
    //初始化

	//LPCTSTR lpszUnicode = "Test String";
	//MessageBox(NULL, lpszUnicode, lpszUnicode, MB_YESNO);

	memset(sell_benchs_in_queue, false, sizeof(sell_benchs_in_queue));
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

    readMap();
    puts("OK");
    fflush(stdout);

    while (scanf("%d", &frameID) != EOF) {
        //读取地图信息
        readState();
		////根据不同的地图赋予参数
		//if (deskNum == 18) { 
		//	count_have_materials_weight = -10; 
		//	to_sell_need_frames_error = 0; 
		//	to_buy_need_frames_error = 0;

		//}//map4
		//else if (deskNum == 50) { 
		//	count_have_materials_weight = 0;
		//}//map3
		//else if (deskNum == 25) { 
		//	count_have_materials_weight = -24; 
		//	map_2_flag = true;
		//}//map2
		//else { 
		//	//SDK\c++\buidl\Release\main.exe 10 5 150.0 - 50 0 8 0.1 - 0.5 - 0.7 0
		//	//{"status":"Successful", "score" : 673194}
		//	count_have_materials_weight = -10;  
		//	map_2_flag = true;
		//}//map1
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
