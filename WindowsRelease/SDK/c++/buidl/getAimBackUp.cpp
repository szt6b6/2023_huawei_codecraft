//#include"globalVariables.h"
////#include"headfile.h"
//using namespace std;
//
//
////----------��������-----------//
///*
//if (res_sell_seq != -1 && (calculateDist(robot.local[task.robotID], workDesk.local[res_sell_seq]) / 0.10 + frameID + to_buy_need_frames_error) < 9001)
//
//if (to_buy_need_frames + frameID < task.predictFrameDoneTask + to_buy_need_frames_error)
//	continue;
//
//double finish_t_need_frames = total_dist / 0.10 + to_sell_need_frames_error;
//if (finish_t_need_frames > 9001 - frameID)
//	continue;
//������֤�����˵��﹤��̨֮ǰ ��Ʒ�Ѿ�������� ������ʱ�����֮ǰ���� ��λ֡��
//*/
//int to_sell_need_frames_error = 32;
//int to_buy_need_frames_error = 25;
//
///*
//if (sell_bench_type == 9)
//	sell_bench_logical_dist = 200; //������Ҫȥ9�����������ʵ��û�ط�ȥ��
//��֤�����˾�����Ҫȥ9����Ʒ
//*/
//double sell_bench_logical_dist_estimate = 150; //
//
///*
//double c_dist = calculateDist(workDesk.local[buy_bench_seq], workDesk.local[sell_bench_seq]) + sell_bench_logical_dist - count_have_materials * count_have_materials_weight;
//ʹ����������ȥ�Ѿ�����ĳ�ֲ��ϵĹ���̨ȥ����Ʒ �����ܾ����������
//*/
//double count_have_materials_weight = 100; //1���100
//
//
///*
//double logical_dist = calculateDist(workDesk.local[buy_bench_seq], workDesk.local[sell_bench_seq]) + sell_bench_logical_dist + produce_done_status * produce_done_status_weight - count_have_materials * count_have_materials_weight;
//�����Ѿ��в�Ʒ�ĸ߼�����̨ ��Ҫȥ��������Ʒ�Ĵ���Ȩ��
//*/
//double produce_done_status_weight = 150; //�ƺ�Ҳ������
//
//double limit_bench_to_buy = 8;
//
//
//double average_speed = 0.10;
//
//
//
////----------��������-----------//
//
//std::list<Task> taskQueue;//�������
//std::list<WorkingRobot> workingRobotsQueue;//�����л����˶���
//std::map<int, std::list<int>> where_to_sell = { {1 , {4, 5, 9}}, {2 , {4, 6, 9}}, {3 , {5, 6, 9}}, {4 , {7, 9}}, {5 , {7, 9}}, {6 , {7, 9}}, {7 , {8, 9}} };
//double product_profit_dict[8] = { 0,3000,3200,3400,7100,7800,8300,29000 };
//bool sell_benchs_in_queue[50][8] = { false };//��ʾ�Ƿ�����Ҫȥĳ������̨��ĳ�ֲ�Ʒ ������8��9���͹���̨ʱ������ֻ��1֡����ȴ���ʲ��ؿ���ռλ����
//int buy_benchs_in_queue[50] = { 0 };//��ʾ�м�����Ҫȥĳ������̨���� ����1��2��3�ͼ�����̨�����2�˿���ȥ���� 3���ϸ߼�����ֻ̨����һ����ȥ����
//bool free_robots[4] = { true, true, true, true };
//std::map<int, std::list<int>> infor_working_benchs_dict = { {1 , {}}, {2 , {}}, {3 , {}}, {4 , {}}, { 5 , {} }, { 6 , {} }, { 7 , {} },{ 8, {} },{ 9, {} } };//�洢��Ӧ���͵Ĺ���̨
//
//double aimDist[RobotNum] = { 0 };
///**
// * FunctionName: getAim()
// * Input: ��
// * Return: ��
// * Description: �õ�������Ŀ�깤��̨�Ͷ�������
// * Author: ����,2023/3/11
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
//input ��Ʒ���ͣ�����̨���
//return ����һ���ܹ�ȥ�������Ĺ���̨
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
//			//���ȥ8 9����ȥ����Ʒ �Ͳ�������ȴ��ռλ
//			if (sell_bench_type >= 8 || (!sell_benchs_in_queue[sell_bench_seq][product_type]) && ((workDesk.materialState[sell_bench_seq] & (1 << product_type)) == 0)) {
//				//if ((!sell_benchs_in_queue[sell_bench_seq][product_type]) && ((workDesk.materialState[sell_bench_seq] & (1 << product_type)) == 0)) {
//
//				if (sell_bench_type == 9)
//					sell_bench_logical_dist = sell_bench_logical_dist_estimate; //������Ҫȥ9�����������ʵ��û�ط�ȥ��
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
////�и��뷨 ���Ը��ݳ��ϲ�Ʒ������̬�仯��Ʒ���ȼ�
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
//			//fprintf(stderr, "�ж����� buy_benchs_in_queue benchseq: %d ,contition: %d\n", to_buy_bench_seq, (buy_benchs_in_queue[to_buy_bench_seq]));
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
//	////�Ե�λʱ���ڻ���������ΪĿ��˼· ���Ľ� ��������
//	//for (int r_id = 0; r_id < 4; r_id++) {
//	//	if (free_robots[r_id] && taskQueue.size() > 0) {
//	//		double can_get_profit = 0;
//	//		Task t; //isinit Ĭ��Ϊfalse
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
//	//			double profit_per_frame = product_profit / finish_t_need_frames *task.priority;// * ϵ��
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
//	//���������ȡ��ΪĿ��˼·
//	for (int r_id = 0; r_id < 4; r_id++) {
//		if (free_robots[r_id] && taskQueue.size() > 0) {
//			double dist = INFINITY;
//			Task t; //isinit Ĭ��Ϊfalse
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
//				if (c_dist < dist) { //���Լ����߼����� Ӧ��Ҫ���Ȱ���ԭ���ϵĹ���̨����
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
//		else if (free_robots[r_id] && taskQueue.size() == 0) { //������û����� �����������ֻ����������̨������Ҫȥ�� �����������ø�Ŀ�깤��̨ �����Ϲ�ȥ
//			double cost = INFINITY;
//			for (list<WorkingRobot>::iterator p = workingRobotsQueue.begin(); p != workingRobotsQueue.end(); p++) {
//				WorkingRobot task = *p;
//				if (task.productType > 3) continue;
//
//				//������һ����������Ʒ���ʱ�� �������ʱ��Ϲ�ȥ����̨
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
//		if (task.buyOrSell) { //��������
//
//			//fprintf(stderr, "frame_id: %d buy task robotid: %d from buy %d to sell %d buyORsell flag %d\n", frameID, task.robotID, task.buyBenchSeq, task.sellBenchSeq, task.buyOrSell);
//			if (robot.deskID[task.robotID] == task.buyBenchSeq && workDesk.productState[task.buyBenchSeq] == 1) {
//				printf("buy %d\n", task.robotID);
//				buy_benchs_in_queue[task.buyBenchSeq] -= 1;
//				(*p).buyOrSell = false; //Ҫ*p��ʽ���ܸĵ����ֵ ����
//				aimDesk[task.robotID] = task.sellBenchSeq;
//
//			}
//			else {
//				aimDesk[task.robotID] = task.buyBenchSeq;
//			}
//		}
//		else { //��������
//
//			//fprintf(stderr, "frame_id: %d sell task %d\n", frameID, task.robotID);
//			if (robot.deskID[task.robotID] == task.sellBenchSeq) {
//				printf("sell %d\n", task.robotID);
//				sell_benchs_in_queue[task.sellBenchSeq][task.productType] = false;
//				p = workingRobotsQueue.erase(p);
//				free_robots[task.robotID] = true;
//
//				//�ڵ�ǰ����̨���� ���в�Ʒ���������� ��δ����ƺ�û���� �úͲ���һ������
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