//#include"globalVariables.h"
////#include"headfile.h"
//using namespace std;
//
//
////----------调参区域-----------//
///*
//if (res_sell_seq != -1 && (calculateDist(robot.local[task.robotID], workDesk.local[res_sell_seq]) / 0.10 + frameID + to_buy_need_frames_error) < 9001)
//
//if (to_buy_need_frames + frameID < task.predictFrameDoneTask + to_buy_need_frames_error)
//	continue;
//
//double finish_t_need_frames = total_dist / 0.10 + to_sell_need_frames_error;
//if (finish_t_need_frames > 9001 - frameID)
//	continue;
//用来保证机器人到达工作台之前 产品已经生产完成 与能在时间结束之前卖出 单位帧数
//*/
//int to_sell_need_frames_error = 32;
//int to_buy_need_frames_error = 25;
//
///*
//if (sell_bench_type == 9)
//	sell_bench_logical_dist = 200; //尽量不要去9卖东西，如果实在没地方去卖
//保证机器人尽量不要去9买物品
//*/
//double sell_bench_logical_dist_estimate = 150; //
//
///*
//double c_dist = calculateDist(workDesk.local[buy_bench_seq], workDesk.local[sell_bench_seq]) + sell_bench_logical_dist - count_have_materials * count_have_materials_weight;
//使机器人优先去已经存在某种材料的工作台去卖产品 让其能尽快进行生产
//*/
//double count_have_materials_weight = 100; //1最好100
//
//
///*
//double logical_dist = calculateDist(workDesk.local[buy_bench_seq], workDesk.local[sell_bench_seq]) + sell_bench_logical_dist + produce_done_status * produce_done_status_weight - count_have_materials * count_have_materials_weight;
//对于已经有产品的高级工作台 我要去它那卖产品的代价权重
//*/
//double produce_done_status_weight = 150; //似乎也是最优
//
//double limit_bench_to_buy = 8;
//
//
//double average_speed = 0.10;
//
//
//
////----------调参区域-----------//
//
//std::list<Task> taskQueue;//任务队列
//std::list<WorkingRobot> workingRobotsQueue;//工作中机器人队列
//std::map<int, std::list<int>> where_to_sell = { {1 , {4, 5, 9}}, {2 , {4, 6, 9}}, {3 , {5, 6, 9}}, {4 , {7, 9}}, {5 , {7, 9}}, {6 , {7, 9}}, {7 , {8, 9}} };
//double product_profit_dict[8] = { 0,3000,3200,3400,7100,7800,8300,29000 };
//bool sell_benchs_in_queue[50][8] = { false };//表示是否有人要去某个工作台卖某种产品 当考虑8，9类型工作台时，由于只有1帧的冷却，故不必考虑占位问题
//int buy_benchs_in_queue[50] = { 0 };//表示有几将人要去某个工作台买东西 限制1，2，3低级工作台最多有2人可以去买东西 3以上高级工作台只能有一个人去买东西
//bool free_robots[4] = { true, true, true, true };
//std::map<int, std::list<int>> infor_working_benchs_dict = { {1 , {}}, {2 , {}}, {3 , {}}, {4 , {}}, { 5 , {} }, { 6 , {} }, { 7 , {} },{ 8, {} },{ 9, {} } };//存储对应类型的工作台
//
//double aimDist[RobotNum] = { 0 };
///**
// * FunctionName: getAim()
// * Input: 无
// * Return: 无
// * Description: 得到机器人目标工作台和动作序列
// * Author: 余游,2023/3/11
// */
//int aimDesk[RobotNum] = { 0 };
//int aimAction[RobotNum][actionNum] = { 0 };
//void getAim()
//{
//	for (int i = 0; i < RobotNum; i++) {
//		if (aimAction[i][0] != 0) {
//			continue;
//		}
//	}
//}
//
//
///**
//input 产品类型，买工作台序号
//return 返回一个能过去卖东西的工作台
//*/
//
//int find_a_bench_to_sell(int product_type, int buy_bench_seq) {
//	double dist = INFINITY;
//	int res_sell_seq = -1;
//
//	for (list<int>::iterator p1 = where_to_sell[product_type].begin(); p1 != where_to_sell[product_type].end(); p1++) {
//		int sell_bench_type = *p1;
//		double sell_bench_logical_dist = 0;
//		int produce_done_status = 0;
//		for (list<int>::iterator p2 = infor_working_benchs_dict[sell_bench_type].begin(); p2 != infor_working_benchs_dict[sell_bench_type].end(); p2++) {
//			int sell_bench_seq = *p2;
//
//			//如果去8 9类型去卖产品 就不考虑冷却和占位
//			if (sell_bench_type >= 8 || (!sell_benchs_in_queue[sell_bench_seq][product_type]) && ((workDesk.materialState[sell_bench_seq] & (1 << product_type)) == 0)) {
//				//if ((!sell_benchs_in_queue[sell_bench_seq][product_type]) && ((workDesk.materialState[sell_bench_seq] & (1 << product_type)) == 0)) {
//
//				if (sell_bench_type == 9)
//					sell_bench_logical_dist = sell_bench_logical_dist_estimate; //尽量不要去9卖东西，如果实在没地方去卖
//				int count_have_materials = 0;
//				for (int i = 1; i < 8; i++) {
//					if (sell_benchs_in_queue[sell_bench_seq][i]) count_have_materials++;
//				}
//				if (workDesk.productState[sell_bench_seq] == 1)
//					produce_done_status = 1;
//
//				double logical_dist = calculateDist(workDesk.local[buy_bench_seq], workDesk.local[sell_bench_seq]) + sell_bench_logical_dist + produce_done_status * produce_done_status_weight - count_have_materials * count_have_materials_weight;
//
//				if (logical_dist < dist) {
//					dist = logical_dist;
//					res_sell_seq = sell_bench_seq;
//				}
//			}
//		}
//	}
//	return res_sell_seq;
//}
//
////有个想法 可以根据场上产品数量动态变化产品优先级
//double getPriority(int bench_type) {
//	switch (bench_type)
//	{
//	case(1):
//		return 20;
//	case(2):
//		return 20;
//	case(3):
//		return 20;
//	case(4):
//		return 5;
//	case(5):
//		return 5;
//	case(6):
//		return 5;
//	case(7):
//		return 1;
//	default:
//		return 0;
//	}
//	//return round(bench_type / 3);
//}
//void update_queue() {
//
//	for (int bench_type = 1; bench_type < limit_bench_to_buy; bench_type++) {
//		for (list<int>::iterator p1 = infor_working_benchs_dict[bench_type].begin(); p1 != infor_working_benchs_dict[bench_type].end(); p1++) {
//			int to_buy_bench_seq = *p1;
//			//fprintf(stderr, "判断条件 buy_benchs_in_queue benchseq: %d ,contition: %d\n", to_buy_bench_seq, (buy_benchs_in_queue[to_buy_bench_seq]));
//			if ((buy_benchs_in_queue[to_buy_bench_seq] == 0) && (workDesk.productState[to_buy_bench_seq] == 1 || workDesk.waitTime[to_buy_bench_seq] > 0)) {
//				int res_sell_seq = find_a_bench_to_sell(bench_type, to_buy_bench_seq);
//				//fprintf(stderr, "try ot add task, found sell bench seq %d\n", res_sell_seq);
//				if (res_sell_seq != -1) {
//					buy_benchs_in_queue[to_buy_bench_seq] += 1;
//					sell_benchs_in_queue[res_sell_seq][bench_type] = true;
//
//					Task task;
//					task.isInit = true;
//					task.priority = getPriority(bench_type);
//					task.predictFrameDoneTask = workDesk.waitTime[to_buy_bench_seq] + frameID;
//					task.buyBenchSeq = to_buy_bench_seq;
//					task.sellBenchSeq = res_sell_seq;
//					task.productType = bench_type;
//
//					taskQueue.push_back(task);
//				}
//			}
//		}
//	}
//
//	////以单位时间内获得最大收益为目标思路 待改进 参数调整
//	//for (int r_id = 0; r_id < 4; r_id++) {
//	//	if (free_robots[r_id] && taskQueue.size() > 0) {
//	//		double can_get_profit = 0;
//	//		Task t; //isinit 默认为false
//	//		list<Task>::iterator position;
//	//		for (list<Task>::iterator p1 = taskQueue.begin(); p1 != taskQueue.end(); p1++) {
//	//			Task task = *p1;
//
//	//			double c_dist = calculateDist(robot.local[r_id], workDesk.local[task.buyBenchSeq]);
//	//			double to_buy_need_frames = c_dist / 0.10 + to_buy_need_frames_error;
//	//			double total_dist = c_dist + calculateDist(workDesk.local[task.buyBenchSeq], workDesk.local[task.sellBenchSeq]);
//
//	//			if (to_buy_need_frames + frameID < task.predictFrameDoneTask)
//	//				continue;
//
//	//			double finish_t_need_frames = total_dist / 0.10 + to_sell_need_frames_error;
//	//			if (finish_t_need_frames > 9001 - frameID)
//	//				continue;
//
//	//			double product_profit = product_profit_dict[task.productType];
//	//			double profit_per_frame = product_profit / finish_t_need_frames *task.priority;// * 系数
//
//
//	//			if (can_get_profit < profit_per_frame) {
//	//				can_get_profit = profit_per_frame;
//	//				t = task;
//	//				position = p1;
//	//			}
//	//		}
//
//	//		if (t.isInit) {
//
//	//			free_robots[r_id] = false;
//	//			taskQueue.erase(position);
//	//			WorkingRobot workingRobot;
//	//			workingRobot.robotID = r_id;
//	//			workingRobot.buyBenchSeq = t.buyBenchSeq;
//	//			workingRobot.sellBenchSeq = t.sellBenchSeq;
//	//			workingRobot.productType = t.productType;
//	//			workingRobot.buyOrSell = true; //buy->true; sell->false
//
//	//			workingRobotsQueue.push_back(workingRobot);
//	//		}
//	//	}
//	//}
//
//
//	//以最近距离取买为目标思路
//	for (int r_id = 0; r_id < 4; r_id++) {
//		if (free_robots[r_id] && taskQueue.size() > 0) {
//			double dist = INFINITY;
//			Task t; //isinit 默认为false
//			list<Task>::iterator position;
//			for (list<Task>::iterator p1 = taskQueue.begin(); p1 != taskQueue.end(); p1++) {
//				Task task = *p1;
//
//				double c_dist = calculateDist(robot.local[r_id], workDesk.local[task.buyBenchSeq]);
//				double to_buy_need_frames = c_dist / average_speed;
//				double total_dist = c_dist + calculateDist(workDesk.local[task.buyBenchSeq], workDesk.local[task.sellBenchSeq]);
//
//				if (to_buy_need_frames + frameID < task.predictFrameDoneTask + to_buy_need_frames_error)
//					continue;
//
//				double finish_t_need_frames = total_dist / average_speed + to_sell_need_frames_error;
//				if (finish_t_need_frames > 9001 - frameID) {
//					continue;
//				}
//
//
//				//double to_buy_need_frames = estiamteTime(robot.local[r_id], workDesk.local[task.buyBenchSeq], robot.towardAngle[r_id]) * 50;
//				//if (to_buy_need_frames + frameID + to_buy_need_frames_error < task.predictFrameDoneTask)
//				//	continue;
//
//				//double c_dist = calculateDist(robot.local[r_id], workDesk.local[task.buyBenchSeq]);
//
//				//double finish_t_need_frames = estiamteTime2(robot.local[r_id], workDesk.local[task.buyBenchSeq], workDesk.local[task.sellBenchSeq], robot.towardAngle[r_id]) * 50;
//				//if (finish_t_need_frames + to_sell_need_frames_error > 9001 - frameID)
//				//	continue;
//
//				if (c_dist < dist) { //尝试加入逻辑距离 应该要优先把有原材料的工作台填满
//					dist = c_dist;
//					t = task;
//					position = p1;
//				}
//			}
//
//			if (t.isInit) {
//
//				free_robots[r_id] = false;
//				taskQueue.erase(position);
//				WorkingRobot workingRobot;
//				workingRobot.robotID = r_id;
//				workingRobot.buyBenchSeq = t.buyBenchSeq;
//				workingRobot.sellBenchSeq = t.sellBenchSeq;
//				workingRobot.productType = t.productType;
//				workingRobot.buyOrSell = true; //buy->true; sell->false
//				workingRobot.predictFrameDoneTask = t.predictFrameDoneTask;
//
//				workingRobotsQueue.push_back(workingRobot);
//			}
//		}
//		else if (free_robots[r_id] && taskQueue.size() == 0) { //机器人没任务接 出现这种情况只可能是买工作台都有人要去了 给机器人设置个目标工作台 让它赶过去
//			double cost = INFINITY;
//			for (list<WorkingRobot>::iterator p = workingRobotsQueue.begin(); p != workingRobotsQueue.end(); p++) {
//				WorkingRobot task = *p;
//				if (task.productType > 3) continue;
//
//				//计算下一轮生产出产品最快时间 根据这个时间赶过去工作台
//				int buyBenchSeq = task.buyBenchSeq;
//				double predict_next_product_generate_done = task.predictFrameDoneTask + 50;
//
//				if (predict_next_product_generate_done < cost) {
//					cost = predict_next_product_generate_done;
//					aimDesk[r_id] = buyBenchSeq;
//				}
//			}
//		}
//	}
//}
//
//
//void run_task() {
//
//	//fprintf(stderr, "len(workingRobotsQueue): %d  len(task_queue): %d\n", workingRobotsQueue.size(), taskQueue.size());
//
//	for (list<WorkingRobot>::iterator p = workingRobotsQueue.begin(); p != workingRobotsQueue.end(); p++) {
//		WorkingRobot task = *p;
//
//		//if(frameID % 89 == 0) fprintf(stderr, "robotid: %d,  buy_bench_seq: %d,  sell_bench_seq: %d,  buyOrSell:1_0 : %d  speed: (%f, %f) \n", task.robotID, task.buyBenchSeq, task.sellBenchSeq, task.buyOrSell, robot.lineSpeed[task.robotID][0], robot.lineSpeed[task.robotID][1]);
//
//		if (task.buyOrSell) { //买任务中
//
//			//fprintf(stderr, "frame_id: %d buy task robotid: %d from buy %d to sell %d buyORsell flag %d\n", frameID, task.robotID, task.buyBenchSeq, task.sellBenchSeq, task.buyOrSell);
//			if (robot.deskID[task.robotID] == task.buyBenchSeq && workDesk.productState[task.buyBenchSeq] == 1) {
//				printf("buy %d\n", task.robotID);
//				buy_benchs_in_queue[task.buyBenchSeq] -= 1;
//				(*p).buyOrSell = false; //要*p方式才能改掉这个值 吐了
//				aimDesk[task.robotID] = task.sellBenchSeq;
//
//			}
//			else {
//				aimDesk[task.robotID] = task.buyBenchSeq;
//			}
//		}
//		else { //卖任务中
//
//			//fprintf(stderr, "frame_id: %d sell task %d\n", frameID, task.robotID);
//			if (robot.deskID[task.robotID] == task.sellBenchSeq) {
//				printf("sell %d\n", task.robotID);
//				sell_benchs_in_queue[task.sellBenchSeq][task.productType] = false;
//				p = workingRobotsQueue.erase(p);
//				free_robots[task.robotID] = true;
//
//				//在当前工作台卖出 若有产品则立即买入 这段代码似乎没卵用 用和不用一个分数
//				if (workDesk.productState[task.sellBenchSeq] == 1 && buy_benchs_in_queue[task.sellBenchSeq] == 0) {
//					int res_sell_seq = find_a_bench_to_sell(workDesk.deskType[task.sellBenchSeq], task.sellBenchSeq);
//					if (res_sell_seq != -1 && (calculateDist(robot.local[task.robotID], workDesk.local[res_sell_seq]) / average_speed + frameID + to_buy_need_frames_error) < 9001) {
//						printf("buy %d\n", task.robotID);
//						free_robots[task.robotID] = false;
//						sell_benchs_in_queue[res_sell_seq][workDesk.deskType[task.sellBenchSeq]] = true;
//
//						WorkingRobot workingRobot;
//						workingRobot.robotID = task.robotID;
//						workingRobot.buyBenchSeq = task.sellBenchSeq;
//						workingRobot.sellBenchSeq = res_sell_seq;
//						workingRobot.productType = workDesk.deskType[task.sellBenchSeq];
//						workingRobot.buyOrSell = false; //buy->true; sell->false
//
//						workingRobotsQueue.push_back(workingRobot);
//
//						aimDesk[task.robotID] = res_sell_seq;
//					}
//				}
//			}
//			else {
//				aimDesk[task.robotID] = task.sellBenchSeq;
//			}
//		}
//
//	}
//}