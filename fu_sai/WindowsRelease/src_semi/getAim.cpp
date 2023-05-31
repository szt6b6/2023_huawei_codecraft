#include"globalVariables.h"
//#include"headfile.h"
using namespace std;


//----------��������-----------//
/*
������֤�����˵��﹤��̨֮ǰ ��Ʒ�Ѿ�������� ������ʱ�����֮ǰ���� ��λ֡��
*/
int to_sell_need_frames_error = 10;
int to_buy_need_frames_error = 10;

/*
��֤�����˾�����Ҫȥ9����Ʒ
*/
double sell_bench_logical_dist_estimate = 500;
/*
�����Ѿ��ڵĸ߼�����̨ ����ȥ��������Ʒ
*/
double producing_status_weight = -100;
/*
��������������Ĺ���̨���� ��ʱ������������7������ Ч����úܶ�
*/
double limit_bench_to_buy = 4;
/*
���ƻ�����ƽ���ٶ�
*/
double average_speed = 0.10;
/*
����̨����Ȩ�� ���ȱ�ʾ��ǰ����̨����Ʒ������ȥ��Ӧ����̨������
*/
double bench_out_edge_num_weight = -0.1;
/*
��ǰ����̨����� ��ȱ�ʾ��ǰ����̨�ܻ��ն��ٸ�����̨����Ʒ
*/
double bench_in_edge_num_weight = -0.7;
/*
���ƻ�������ִ��������û����Ʒʱ ����������и����ȵ����� ȥ���Ǹ������ȵ�����
*/
bool map_2_flag = false;

double frames_estimate_weight = 2;
/*
ԭ���������Ȩ��
*/
double priority_sell_weight = -25;

double priority_buy_weight = -25;

double bench_7_priority = 0.0;


//----------��������-----------//

std::list<int> buy_task_seq_queue;//�������
WorkingRobot working_robots_queue[4];//�����л����˶���
std::map<int, std::list<int>> where_to_sell = { {1 , {4, 5, 9}}, {2 , {4, 6, 9}}, {3 , {5, 6, 9}}, {4 , {7, 9}}, {5 , {7, 9}}, {6 , {7, 9}}, {7 , {8, 9}} };
double product_profit_dict[8] = { 0,3000,3200,3400,7100,7800,8300,29000 };
bool sell_benchs_in_queue[DeskMaxNum][8] = { false };//��ʾ�Ƿ�����Ҫȥĳ������̨��ĳ�ֲ�Ʒ ������8��9���͹���̨ʱ������ֻ��1֡����ȴ���ʲ��ؿ���ռλ����
int buy_benchs_in_queue[DeskMaxNum] = { 0 };//��ʾ�м�����Ҫȥĳ������̨���� ����1��2��3�ͼ�����̨�����2�˿���ȥ���� 3���ϸ߼�����ֻ̨����һ����ȥ����
//bool free_robots[4] = { true, false, false, false };

bool free_robots[4] = { true, true, true, true };
//bool free_robots[4] = { false, false, false, true };
std::map<int, std::list<int>> infor_working_benchs_dict = { {1 , {}}, {2 , {}}, {3 , {}}, {4 , {}}, { 5 , {} }, { 6 , {} }, { 7 , {} },{ 8, {} },{ 9, {} } };//�洢��Ӧ���͵Ĺ���̨
int full_dict[8] = { 0,0,0,0,6,10,12,112 };
int material_demand[7] = { 0 }; //��ʾ��ǰʱ��1-6���ϵ����� //
int bench_in_edge_num[DeskMaxNum] = { 0 };
int bench_out_edge_num[DeskMaxNum] = { 0 };
double aimDist[RobotNum] = { 0 };
int min_material_demand_num_456;
int min_material_demand_num_123;
/**
 * FunctionName: getAim()
 * Input: ��
 * Return: ��
 * Description: �õ�������Ŀ�깤��̨�Ͷ�������
 * Author: ����,2023/3/11
 */
int aimDesk[RobotNum] = { 0 };
int aimAction[RobotNum][actionNum] = { 0 };
void getAim()
{
	for (int i = 0; i < RobotNum; i++) {
		if (aimAction[i][0] != 0) {
			continue;
		}
	}
}


/**
input ��Ʒ���ͣ�����̨���
return ����һ���ܹ�ȥ�������Ĺ���̨
*/

//�и��뷨 ���Ը��ݳ��ϲ�Ʒ������̬�仯��Ʒ���ȼ�
double getPriority(int bench_type) {
	if (bench_type >= 8) return bench_7_priority;
	
	if (bench_type > 3)
		return (material_demand[bench_type] - min_material_demand_num_456);

	else
		return (material_demand[bench_type] - min_material_demand_num_123);
}

int find_a_bench_to_sell(int product_type, int buy_bench_seq, int r_id) {
	double dist = INFINITY;
	int res_sell_seq = -1;

	for (list<int>::iterator p1 = where_to_sell[product_type].begin(); p1 != where_to_sell[product_type].end(); p1++) {
		int sell_bench_type = *p1;
		double sell_bench_logical_dist = 0;
		int producing_status = 0;
		for (list<int>::iterator p2 = infor_working_benchs_dict[sell_bench_type].begin(); p2 != infor_working_benchs_dict[sell_bench_type].end(); p2++) {
			int sell_bench_seq = *p2;
			if (!roborts_can_go_bench_sell_array[r_id][sell_bench_seq] || bench2bench_paths_sell[buy_bench_seq][sell_bench_seq].size() == 0) continue;

			double to_sell_frames_estimate = bench2bench_paths_sell[buy_bench_seq][sell_bench_seq].size() / 2 / average_speed + to_sell_need_frames_error;
			if (to_sell_frames_estimate + frameID > 15000) continue;

			//���ȥ8 9����ȥ����Ʒ �Ͳ�������ȴ��ռλ
			if (sell_bench_type >= 8 || (!sell_benchs_in_queue[sell_bench_seq][product_type]) && ((workDesk.materialState[sell_bench_seq] & (1 << product_type)) == 0)) {
				//if ((!sell_benchs_in_queue[sell_bench_seq][product_type]) && ((workDesk.materialState[sell_bench_seq] & (1 << product_type)) == 0)) {

				if (sell_bench_type == 9)
					sell_bench_logical_dist = sell_bench_logical_dist_estimate; //������Ҫȥ9�����������ʵ��û�ط�ȥ��
				int count_have_materials = 0;

				for (int i = 1; i < 8; i++) {
					if ((workDesk.materialState[sell_bench_seq] & (1 << i)) != 0) count_have_materials++;
				}

				if (workDesk.productState[sell_bench_seq] == 1 || workDesk.waitTime[sell_bench_seq] > 0 || count_have_materials > 0)
					producing_status = 1;

				//����ȥ�Ѿ��ڹ����Ĺ���̨������
				double logical_dist = bench2bench_paths_sell[buy_bench_seq][sell_bench_seq].size()
					+ sell_bench_logical_dist
					+ producing_status * producing_status_weight
					+ priority_sell_weight * getPriority(sell_bench_type);

				if (logical_dist < dist) {
					dist = logical_dist;
					res_sell_seq = sell_bench_seq;
				}
			}
		}
	}
	return res_sell_seq;
}

void update_queue() {
	if (!(free_robots[0] || free_robots[1] || free_robots[2] || free_robots[3])) return;
	if (frameID > 14500) { limit_bench_to_buy = 8; bench_7_priority = 999; }
	//�ռ����п���Ĺ���̨
	for (int bench_type = 1; bench_type < limit_bench_to_buy; bench_type++) {

		for (list<int>::iterator p1 = infor_working_benchs_dict[bench_type].begin(); p1 != infor_working_benchs_dict[bench_type].end(); p1++) {
			int to_buy_bench_seq = *p1;

			if ((buy_benchs_in_queue[to_buy_bench_seq] == 0) && (workDesk.productState[to_buy_bench_seq] == 1 || workDesk.waitTime[to_buy_bench_seq] > 0)) {

				buy_benchs_in_queue[to_buy_bench_seq] += 1;

				buy_task_seq_queue.push_back(to_buy_bench_seq);
			
			}
		}
	}

	//����һ�³��ϲ�����������
	min_material_demand_num_456 = 999;
	min_material_demand_num_123 = 999;
	for (int i = 4; i < 7; i++) {
		min_material_demand_num_456 = std::min(material_demand[i], min_material_demand_num_456);
	}
	for (int i = 1; i < 4; i++) {
		min_material_demand_num_123 = std::min(material_demand[i], min_material_demand_num_123);
	}

	//�����л����˷�������
	for (int r_id = 0; r_id < 4; r_id++) {
		if (free_robots[r_id] && buy_task_seq_queue.size() > 0) {
			double dist = INFINITY;
			int des_to_buy = -1, des_to_sell = -1;
			list<int>::iterator position;
			for (list<int>::iterator p1 = buy_task_seq_queue.begin(); p1 != buy_task_seq_queue.end(); p1++) {
				int buy_bench_seq = *p1;

				if (!roborts_can_go_bench_buy_array[r_id][buy_bench_seq]) continue;

				double buy_frames_estimate;
				if (robot.deskID[r_id] > -1 && bench2bench_paths_buy[robot.deskID[r_id]][buy_bench_seq].size() > 0)
					buy_frames_estimate = bench2bench_paths_buy[robot.deskID[r_id]][buy_bench_seq].size() / 2.0 / average_speed;
				else
					buy_frames_estimate = calculateDist(robot.local[r_id], workDesk.local[buy_bench_seq]) / average_speed;

				if (buy_frames_estimate + to_buy_need_frames_error < workDesk.waitTime[buy_bench_seq]) continue;

				int sell_bench_seq = find_a_bench_to_sell(workDesk.deskType[buy_bench_seq], buy_bench_seq, r_id);

				if (sell_bench_seq == -1) continue;

				double sell_frames_estimate = bench2bench_paths_sell[buy_bench_seq][sell_bench_seq].size() / 2.0 / average_speed;

				if (sell_frames_estimate + buy_frames_estimate + to_sell_need_frames_error + frameID > 15000) continue;


				double logical_dist = (buy_frames_estimate + sell_frames_estimate) * average_speed * frames_estimate_weight + bench_out_edge_num_weight * bench_in_edge_num[sell_bench_seq] +
					priority_sell_weight * getPriority(workDesk.deskType[sell_bench_seq]) + priority_buy_weight * getPriority(workDesk.deskType[buy_bench_seq]) + 100*(workDesk.deskType[sell_bench_seq] == 9);
				if (logical_dist < dist) { //���Լ����߼����� Ӧ��Ҫ���Ȱ���ԭ���ϵĹ���̨����
					dist = logical_dist;
					des_to_buy = buy_bench_seq;
					des_to_sell = sell_bench_seq;
					position = p1;
				}
			}

			if (des_to_buy != -1 && des_to_sell != -1) {

				free_robots[r_id] = false;
				buy_task_seq_queue.erase(position);
				WorkingRobot workingRobot;
				workingRobot.robotID = r_id;
				workingRobot.buyBenchSeq = des_to_buy;
				workingRobot.sellBenchSeq = des_to_sell;
				workingRobot.productType = workDesk.deskType[des_to_buy];
				workingRobot.buyOrSell = 1; //buy->true; sell->false
				//workingRobot.path = search_Astar_buy(robot.grid[r_id], bench_seq_dist[des_to_buy]); //���빤������ʱ ִ����������һ�µ�����̨��·��
				//workingRobot.path = search_dijkstra_buy(robot.grid[r_id], bench_seq_dist[des_to_buy]);
				working_robots_queue[r_id] = workingRobot;

				sell_benchs_in_queue[des_to_sell][workDesk.deskType[des_to_buy]] = true;
			}
		}
	}

	//������û����� �����������ֻ����������̨������Ҫȥ�� ֱ��������1-3���͹���̨��
	for (int r_id = 0; r_id < robotNum; r_id++) {
		if (free_robots[r_id]) {
			int res_buy = -1, res_sell = -1;
			double dist = INFINITY;
			for (int type = 1; type <= 3; type++) {
				for (auto buy_bench_123_seq : infor_working_benchs_dict[type]) {
					if (!roborts_can_go_bench_buy_array[r_id][buy_bench_123_seq]) continue;
					int sell_bench_seq = find_a_bench_to_sell(type, buy_bench_123_seq, r_id);
					//if (sell_bench_seq == -1) continue;

					double logical_dist = -getPriority(type); //���ȼ���Խ��Խ��
					if (logical_dist < dist) {
						dist = logical_dist;
						res_buy = buy_bench_123_seq;
						res_sell = sell_bench_seq;
					}
				}
			}


			if (res_buy != -1 ) {
				//w.path = search_Astar_buy(robot.grid[r_id], bench_seq_dist[t.buyBenchSeq]); //���빤������ʱ ִ����������һ�µ�����̨��·��
				//w.path = search_dijkstra_buy(robot.grid[r_id], bench_seq_dist[t.buyBenchSeq]);
				working_robots_queue[r_id].buyBenchSeq = res_buy;
				working_robots_queue[r_id].sellBenchSeq = res_sell;
				if (res_sell == -1) {
					working_robots_queue[r_id].buyOrSell = 2;
				}
				else {
					working_robots_queue[r_id].buyOrSell = 1;
					free_robots[r_id] = false;
					sell_benchs_in_queue[res_sell][workDesk.deskType[res_buy]] = true;
					buy_benchs_in_queue[res_buy] += 1;
				}
				working_robots_queue[r_id].productType = workDesk.deskType[res_buy];
				working_robots_queue[r_id].robotID = r_id;

			}
		}
	}

	for (int t : buy_task_seq_queue) {
		buy_benchs_in_queue[t] -= 1;	
	}

	buy_task_seq_queue.clear();
}


void run_task() {

	for (int p = 0; p < 4; p++) {
		if (free_robots[p]) {
			aimDesk[working_robots_queue[p].robotID] = working_robots_queue[p].buyBenchSeq;
		}

		else if (working_robots_queue[p].buyOrSell == 1) { //��������
			if (robot.deskID[p] == working_robots_queue[p].buyBenchSeq && workDesk.productState[working_robots_queue[p].buyBenchSeq] == 1) {
				// ����֮ǰ�ٴ�ִ��ʱ���ж� �������ڽ���ǰ���� �ͷ����������
				double sell_frames_estimate = bench2bench_paths_sell[working_robots_queue[p].buyBenchSeq][working_robots_queue[p].sellBenchSeq].size() / 2.0 / average_speed;
				if (sell_frames_estimate + frameID + to_sell_need_frames_error > 15000) {//�����ڽ���ǰ���� ��������
					free_robots[p] = true;
					buy_benchs_in_queue[working_robots_queue[p].buyBenchSeq] -= 1;
					sell_benchs_in_queue[working_robots_queue[p].sellBenchSeq][workDesk.deskType[working_robots_queue[p].sellBenchSeq]] = true;
					continue;
				}
				printf("buy %d\n", p);
				buy_benchs_in_queue[working_robots_queue[p].buyBenchSeq] -= 1;
				working_robots_queue[p].buyOrSell = 0;
				aimDesk[p] = working_robots_queue[p].sellBenchSeq;

				int p_type = working_robots_queue[p].productType;
				if (workDesk.deskType[working_robots_queue[p].sellBenchSeq] != 9) material_demand[p_type] -= 1;
				if (p_type == 7) { material_demand[4] += 1;  material_demand[5] += 1; material_demand[6] += 1; }
				else if (p_type == 4) { material_demand[1] += 1;  material_demand[2] += 1; }
				else if (p_type == 5) { material_demand[1] += 1;  material_demand[3] += 1; }
				else if (p_type == 6) { material_demand[2] += 1;  material_demand[3] += 1; }

			}
			else {
				aimDesk[p] = working_robots_queue[p].buyBenchSeq;
			}
		}
		else if(working_robots_queue[p].buyOrSell == 0){ //��������
			if (robot.deskID[p] == working_robots_queue[p].sellBenchSeq && (workDesk.materialState[working_robots_queue[p].sellBenchSeq] & (1 << working_robots_queue[p].productType)) == 0) {
				printf("sell %d\n", p);
				sell_benchs_in_queue[working_robots_queue[p].sellBenchSeq][working_robots_queue[p].productType] = false;
				free_robots[p] = true;

				int count_have_materials = 0;
				for (int i = 1; i < 8; i++) {
					if ((workDesk.materialState[working_robots_queue[p].sellBenchSeq] & (1 << i)) != 0) count_have_materials++;
				}
				if (count_have_materials == 1) material_demand[workDesk.deskType[working_robots_queue[p].sellBenchSeq]] -= 1; // 4 5 6 ����̨����
				else if (count_have_materials == 2) { material_demand[4] += 1;  material_demand[5] += 1; material_demand[6] += 1; } // 7����̨����


				//�ڵ�ǰ����̨���� ���в�Ʒ����������
				if (workDesk.productState[working_robots_queue[p].sellBenchSeq] == 1 && buy_benchs_in_queue[working_robots_queue[p].sellBenchSeq] == 0) {
					int res_sell_seq = find_a_bench_to_sell(workDesk.deskType[working_robots_queue[p].sellBenchSeq], working_robots_queue[p].sellBenchSeq, working_robots_queue[p].robotID);
					if (res_sell_seq != -1) {
						printf("buy %d\n", p);

						int p_type = workDesk.deskType[working_robots_queue[p].sellBenchSeq];
						if (workDesk.deskType[working_robots_queue[p].sellBenchSeq] != 9) material_demand[p_type] -= 1;

						if(p_type == 7) { material_demand[4] += 1;  material_demand[5] += 1; material_demand[6] += 1;}
						else if (p_type == 4) { material_demand[1] += 1;  material_demand[2] += 1; }
						else if (p_type == 5) { material_demand[1] += 1;  material_demand[3] += 1; }
						else if (p_type == 6) { material_demand[2] += 1;  material_demand[3] += 1; }
						
						free_robots[p] = false;
						sell_benchs_in_queue[res_sell_seq][workDesk.deskType[working_robots_queue[p].sellBenchSeq]] = true;

						WorkingRobot workingRobot;
						workingRobot.robotID = p;
						workingRobot.buyBenchSeq = working_robots_queue[p].sellBenchSeq;
						workingRobot.sellBenchSeq = res_sell_seq;
						workingRobot.productType = workDesk.deskType[working_robots_queue[p].sellBenchSeq];
						workingRobot.buyOrSell = 0; //buyworking_robots_queue[p].true; sellworking_robots_queue[p].false
						//workingRobot.path = bench2bench_paths_sell[taskworking_robots_queue[p].sellBenchSeq][res_sell_seq]; //���빤������ʱ ִ����������һ�µ�����̨��·��
						working_robots_queue[p] = workingRobot;

						aimDesk[p] = res_sell_seq;
					}
				}
			}
			else {
				aimDesk[p] = working_robots_queue[p].sellBenchSeq;
			}
		}
	}
}
