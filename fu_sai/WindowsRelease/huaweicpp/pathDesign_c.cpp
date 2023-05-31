#include "globalVariables.h"

//#include<time.h>
#define ROWS 100
#define COLS 100

int hasin[ROWS][COLS] = { 0 };//�ڵ��Ƿ��ѱ���Ӷ�
Node_c* small_top_heap[ROWS * COLS];
Node_c* node_pointers[100][100]; //�ռ����е��ָ��
int heap_size = 0;


int obstacles[100][100];

double heuristic_dist(Point c_p, Point goal) {
	double dx = c_p.x - goal.x;
	double dy = c_p.y - goal.y;
	return sqrt(dx * dx + dy * dy);
}

int check_is_valid(Point p, bool buy_or_sell) {
	if (p.x < 0 || p.x >= ROWS || p.y < 0 || p.y >= COLS) {
		return 0;
	}
	if (!buy_or_sell && map_data_ob_expand_sell[p.x][p.y] == '#') {
		return 0;
	}
	if (buy_or_sell && map_data_ob_expand_buy[p.x][p.y] == '#') {
		return 0;
	}
	//for (int i = 0; i < 4; i++) {
	//	if (p.x == robot.grid[i][0] && p.y == robot.grid[i][1]) return 0;
	//}
	return 1;
}

void add_into_heap(Node_c* node) {
	int index = heap_size;
	heap_size++;
	while (index > 0) {
		int parent_index = (index - 1) / 2;
		//��С����ѣ�
		if (small_top_heap[parent_index]->f <= node->f) {
			break;
		}

		small_top_heap[index] = small_top_heap[parent_index];
		index = parent_index;
	}
	small_top_heap[index] = node;
}

Node_c* pop_from_heap() {
	Node_c* node = small_top_heap[0];
	heap_size--;

	Node_c* last_node = small_top_heap[heap_size];
	small_top_heap[0] = last_node;

	int index = 0;
	while (1) {
		int left_child_index = 2 * index + 1;
		int right_child_index = 2 * index + 2;
		int min_child_index = index;

		if (left_child_index < heap_size && small_top_heap[left_child_index]->f < small_top_heap[min_child_index]->f)
			min_child_index = left_child_index;

		if (right_child_index < heap_size && small_top_heap[right_child_index]->f < small_top_heap[min_child_index]->f)
			min_child_index = right_child_index;
		if (min_child_index == index) {
			break;
		}

		Node_c* min_child_node = small_top_heap[min_child_index];
		small_top_heap[min_child_index] = small_top_heap[index];
		small_top_heap[index] = min_child_node;

		index = min_child_index;
	}

	return node;
}


std::list<Node_c> find_path(Point start, Point goal, bool buy_or_sell) {

	for (int i = 0; i < 100; i++) {
		memset(node_isSearched[i], false, sizeof node_isSearched[i]);
		memset(hasin[i], 0, sizeof hasin[i]);
	}

	while (heap_size > 0) {
		pop_from_heap();
	}

	//�ͷ��ڴ�
	for (int i = 0; i < 100; i++)
		for (int j = 0; j < 100; j++)
			if (node_pointers[i][j]) {
				free(node_pointers[i][j]); 
				node_pointers[i][j] = NULL;
			};

	Node_c* start_node = (Node_c*)malloc(sizeof(Node_c));
	start_node->point = start;
	start_node->f = heuristic_dist(start, goal);
	start_node->g = 0;
	start_node->c = 0;
	start_node->dir = -1;
	start_node->parent = NULL; add_into_heap(start_node);
	hasin[start_node->point.x][start_node->point.y] = 1;
	node_pointers[start_node->point.x][start_node->point.y] = start_node;



	std::list<Node_c> res;
	while (heap_size > 0) {
		//����Ȩֵ��С�ڵ�
		Node_c* current_node = pop_from_heap();
		int x = current_node->point.x;
		int y = current_node->point.y;
		node_isSearched[x][y] = true;

		if (current_node->point.x == goal.x && current_node->point.y == goal.y) {
			while ((current_node->point.x != start.x || current_node->point.y != start.y) && current_node->parent) { //���ݸ��� ��������·��
				res.push_front(*current_node);
				current_node->parent->next = current_node; //����nextָ��
				current_node = current_node->parent;
			}
			res.push_front(*current_node);
			return res;
		}

		Point next_points[8] = {
			{x + 1, y},
			{x - 1, y},
			{x, y + 1},
			{x, y - 1},
			{x + 1, y + 1},
			{x + 1, y - 1},
			{x - 1, y + 1},
			{x - 1, y - 1},
		};
		//����8������ڵ㣬�������ϰ���Ľڵ���ӽ������
		//for (int i = 0; i < 8; i++)
		for (int i = 0; i < 4; i++) {
			Point next_point = next_points[i];
			double c = i > 3 ? 1.42 : 1;
			if (!check_is_valid(next_point, buy_or_sell)) {
				continue;
			}
			if (node_isSearched[next_point.x][next_point.y] && node_pointers[next_point.x][next_point.y]->c > current_node->c + c + map_data_cost[next_point.x][next_point.y]) {
				node_pointers[next_point.x][next_point.y]->c = current_node->c + c + map_data_cost[next_point.x][next_point.y];
				node_pointers[next_point.x][next_point.y]->parent = node_pointers[x][y];
			}
			if (hasin[next_point.x][next_point.y] == 1)
				continue;

			Node_c* next_node = (Node_c*)malloc(sizeof(Node_c));
			next_node->point = next_point;
			//��¼�ڵ㷽��
			next_node->dir = i;
			next_node->g = current_node->g + c + map_data_cost[next_point.x][next_point.y];
			next_node->c = next_node->g;
			next_node->f = next_node->g + heuristic_dist(next_point, goal)+4*(next_node->dir!= current_node->dir);
			next_node->parent = current_node;
			hasin[next_node->point.x][next_node->point.y] = 1;
			node_pointers[next_node->point.x][next_node->point.y] = next_node;
			add_into_heap(next_node);
		}
	}
	return res;
}



//��ʼ������̨֮���·�� �������bench2bench_paths_buy��
void init_bench2bench_paths() {
	for (int i = 0; i < init_bench_sum; i++)
	{
		for (int j = 0; j < init_bench_sum; j++)
		{
			if (i == j) continue;

			int b_x0 = bench_seq_dist[i][0];
			int b_y0 = bench_seq_dist[i][1];

			int b_x1 = bench_seq_dist[j][0];
			int b_y1 = bench_seq_dist[j][1];

			Point source = { b_x0, b_y0 };
			Point target = { b_x1, b_y1 };

			int buy_bench_type = bench_seq_type_dist[i];
			int sell_bench_type = bench_seq_type_dist[j];
			//source�ǵ͵ȼ���target�Ǹߵȼ��Ļ�������target��sourceҪȥ���Ĺ���̨
			if (buy_bench_type < sell_bench_type)
			{
				bool source_can_to_target_sell = false;
				for (auto bench_type : where_to_sell[buy_bench_type])
					if (sell_bench_type == bench_type) { source_can_to_target_sell = true; break; }

				if (!source_can_to_target_sell) continue;//j��ŵĹ���̨����Ҫȥ���Ĺ���̨������

				std::list<Node_c> list_sell = find_path(source, target, false);
				//std::list<Node> list_sell = search_dijkstra_sell(source, target);

				bench2bench_paths_sell[i][j] = list_sell;
				//fprintf(stderr, "sell!!!source bench seq:%d,source bench type:%d,target bench seq:%d, target bench type:%d,path length:%d \n", i, buy_bench_type, j, sell_bench_type, list_sell.size());
			}
		}
	}
}