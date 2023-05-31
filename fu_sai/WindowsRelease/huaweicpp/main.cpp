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

	for(int i=0; i<50; i++)
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
	//priority_sell_weight = atof(argv[9]); //-10 10 2
	
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
		if (deskNum==13)
			mapFlag = 3;
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


////look map_data
//int main() {
//
//	for (int i = 0; i < 100; i++) {
//		memset(is_expand_or_set_sell[i], false, sizeof is_expand_or_set_sell[i]);
//		memset(map_data_cost[i], 1, sizeof map_data_cost[i]);
//		memset(obstacles[i], 0, sizeof obstacles[i]);
//	}
//
//
//	FILE *f;
//	f = fopen("data.txt", "r");
//
//	char line[1024];
//	int j = 0;
//	while (fgets(line, sizeof line, f)) {
//		if (line[0] == 'O' && line[1] == 'K') {
//			break;
//		}
//		else
//		{
//			for (int i = 0; i < 100; i++) {
//				map_data[j][i] = line[i];
//			}
//		}
//		j++;
//	}
//
//	for (int i = 0; i < 100; i++) {
//		for (int j = 0; j < 100; j++) {
//
//			if (map_data[i][j] == '#') { //将障碍物周围2格代价设大点
//				obstacles[i][j] = 1;
//			}
//		}
//	}
//	Point start = { 48, 33 };
//	Point goal = { 46, 49 };
//	std::list<Node_c> res = find_path(start, goal);
//	
//	return 0;
//
//}
//
//
//////check 小根堆
//int main(int argc, char** argv) {
//	Node* source = new Node(0, 0);
//	Node* target = new Node(5, 5);
//
//	//每新开始找一对路径 将nodeAdded，res_path, source, target, pq_astar, pointer重置 
//	for (int i = 0; i < 100; i++) {
//		memset(node_added[i], false, sizeof node_added[i]);
//		memset(node_isSearched[i], false, sizeof node_isSearched[i]);
//		memset(map_data_cost[i], 1, sizeof map_data_cost[i]);
//	}
//	while (!pq_astar.empty()) pq_astar.pop();
//	res_path.clear();
//	sourceNode = new  Node(0, 0);
//	targetNode = new Node(5, 5);
//
//	pq_astar.push(source);
//	node_added[sourceNode->row][sourceNode->col] = true; //表示已经加入了堆中
//	all_node_pointers[sourceNode->row][sourceNode->col] = sourceNode; //收集所有点的指针
//
//	while (res_path.size() == 0 && pq_astar.size() > 0) {
//
//		std::priority_queue<Node*, std::vector<Node*>, cmp_a_star> a = pq_astar;
//		
//		while (!a.empty()) {
//			std::cout << a.top()->real_cost << " ";
//			a.pop();
//		}
//		std::cout << std::endl;
//
//		if (res_path.size() > 0 || pq_astar.size() == 0) break; //找到了路或者最后没找到路 直接返回
//	//sort_heap(const_cast<Node*>(pq_astar.top()), const_cast<Node*>(pq_astar.top() + pq_astar.size()), cmp_a_star());
//		Node* node = pq_astar.top();
//		pq_astar.pop();
//
//		node_isSearched[node->row][node->col] = true;
//
//
//		if (node->row == targetNode->row && node->col == targetNode->col) { //找到终点
//
//			Node* current_node = node;
//
//			while (current_node->row != sourceNode->row || current_node->col != sourceNode->col) { //回溯父亲 返回整条路经
//				res_path.push_front(*current_node);
//				current_node->parent->next = current_node; //更新next指针
//				current_node = current_node->parent;
//			}
//			res_path.push_front(*current_node);
//			break;
//		}
//
//
//		//八方向
//	//上
//		if (node->row - 1 >= 0 && map_data_ob_expand_sell[node->row - 1][node->col] != '#') {
//			if (node_isSearched[node->row - 1][node->col]) { //已经遍历过了 判断距离 更新父亲
//				Node *up = all_node_pointers[node->row - 1][node->col];
//				if (up->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col]) {
//					up->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col];
//					up->parent = node;
//				}
//			}
//			else if (!node_added[node->row - 1][node->col]) {
//				Node* up = new Node(node->row - 1, node->col, node);
//				up->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col];
//				all_node_pointers[node->row - 1][node->col] = up;
//				node_added[node->row - 1][node->col] = true;
//				pq_astar.push(up);
//			}
//
//		}
//		////左上
//		//if (node->row - 1 >= 0 && node->col - 1 >= 0 && map_data_ob_expand_sell[node->row - 1][node->col - 1] != '#') {
//		//	if (node_isSearched[node->row - 1][node->col - 1]) { //已经遍历过了 判断距离 更新父亲
//		//		Node *up_left = all_node_pointers[node->row - 1][node->col - 1];
//		//		if (up_left->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col - 1]) {
//		//			up_left->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col - 1];
//		//			up_left->parent = node;
//		//		}
//		//	}
//		//	else if (!node_added[node->row - 1][node->col - 1]) {
//		//		Node* up_left = new Node(node->row - 1, node->col - 1, node);
//		//		all_node_pointers[node->row - 1][node->col - 1] = up_left;
//		//		up_left->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col - 1];
//		//		node_added[node->row - 1][node->col - 1] = true;
//		//		pq_astar.push(up_left);
//		//	}
//		//}
//		////右上
//		//if (node->row - 1 >= 0 && node->col + 1 < 100 && map_data_ob_expand_sell[node->row - 1][node->col + 1] != '#') {
//		//	if (node_isSearched[node->row - 1][node->col + 1]) { //已经遍历过了 判断距离 更新父亲
//		//		Node *up_right = all_node_pointers[node->row - 1][node->col + 1];
//		//		if (up_right->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col + 1]) {
//		//			up_right->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col + 1];
//		//			up_right->parent = node;
//		//		}
//		//	}
//		//	else if (!node_added[node->row - 1][node->col + 1]) {
//		//		Node* up_right = new Node(node->row - 1, node->col + 1, node);
//		//		all_node_pointers[node->row - 1][node->col + 1] = up_right;
//		//		up_right->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col + 1];
//		//		node_added[node->row - 1][node->col + 1] = true;
//		//		pq_astar.push(up_right);
//		//	}
//		//}
//		//下
//		if (node->row + 1 < 100 && map_data_ob_expand_sell[node->row + 1][node->col] != '#') {
//			if (node_isSearched[node->row + 1][node->col]) { //已经遍历过了 判断距离 更新父亲
//				Node *botton = all_node_pointers[node->row + 1][node->col];
//				if (botton->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col]) {
//					botton->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col];
//					botton->parent = node;
//				}
//			}
//			else if (!node_added[node->row + 1][node->col]) {
//				Node* botton = new Node(node->row + 1, node->col, node);
//				all_node_pointers[node->row + 1][node->col] = botton;
//				botton->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col];
//				node_added[node->row + 1][node->col] = true;
//				pq_astar.push(botton);
//			}
//		}
//		////左下
//		//if (node->row + 1 < 100 && node->col - 1 >= 0 && map_data_ob_expand_sell[node->row + 1][node->col - 1] != '#') {
//		//	if (node_isSearched[node->row + 1][node->col - 1]) { //已经遍历过了 判断距离 更新父亲
//		//		Node *botton_left = all_node_pointers[node->row + 1][node->col - 1];
//		//		if (botton_left->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col - 1]) {
//		//			botton_left->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col - 1];
//		//			botton_left->parent = node;
//		//		}
//		//	}
//		//	else if (!node_added[node->row + 1][node->col - 1]) {
//		//		Node* botton_left = new Node(node->row + 1, node->col - 1, node);
//		//		all_node_pointers[node->row + 1][node->col - 1] = botton_left;
//		//		botton_left->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col - 1];
//		//		node_added[node->row + 1][node->col - 1] = true;
//		//		pq_astar.push(botton_left);
//		//	}
//		//}
//		////右下
//		//if (node->row + 1 < 100 && node->col + 1 < 100 && map_data_ob_expand_sell[node->row + 1][node->col + 1] != '#') {
//		//	if (node_isSearched[node->row + 1][node->col + 1]) { //已经遍历过了 判断距离 更新父亲
//		//		Node *botton_right = all_node_pointers[node->row + 1][node->col + 1];
//		//		if (botton_right->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col + 1]) {
//		//			botton_right->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col + 1];
//		//			botton_right->parent = node;
//		//		}
//		//	}
//		//	else if (!node_added[node->row + 1][node->col] + 1) {
//		//		Node* botton_right = new Node(node->row + 1, node->col + 1, node);
//		//		all_node_pointers[node->row + 1][node->col + 1] = botton_right;
//		//		botton_right->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col + 1];
//		//		node_added[node->row + 1][node->col + 1] = true;
//		//		pq_astar.push(botton_right);
//		//	}
//		//}
//		//左
//		if (node->col - 1 >= 0 && map_data_ob_expand_sell[node->row][node->col - 1] != '#') {
//			if (node_isSearched[node->row][node->col - 1]) { //已经遍历过了 判断距离 更新父亲
//				Node *left = all_node_pointers[node->row][node->col - 1];
//				if (left->real_cost > node->real_cost + map_data_cost[node->row][node->col - 1]) {
//					left->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
//					left->parent = node;
//				}
//			}
//			else if (!node_added[node->row][node->col] - 1) {
//				Node* left = new Node(node->row, node->col - 1, node);
//				all_node_pointers[node->row][node->col - 1] = left;
//				left->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
//				node_added[node->row][node->col - 1] = true;
//				pq_astar.push(left);
//			}
//		}
//		//右
//		if (node->col + 1 < 100 && map_data_ob_expand_sell[node->row][node->col + 1] != '#') {
//			if (node_isSearched[node->row][node->col + 1]) { //已经遍历过了 判断距离 更新父亲
//				Node *right = all_node_pointers[node->row][node->col + 1];
//				if (right->real_cost > node->real_cost + map_data_cost[node->row][node->col + 1]) {
//					right->real_cost = node->real_cost + map_data_cost[node->row][node->col + 1];
//					right->parent = node;
//				}
//			}
//			else if (!node_added[node->row][node->col + 1]) {
//				Node* right = new Node(node->row, node->col + 1, node);
//				all_node_pointers[node->row][node->col + 1] = right;
//				right->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
//				node_added[node->row][node->col + 1] = true;
//				pq_astar.push(right);
//			}
//		}
//	}
//	
//	return 0;
//}