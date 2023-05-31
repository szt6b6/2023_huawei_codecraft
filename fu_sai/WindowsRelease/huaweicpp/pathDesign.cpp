#include "globalVariables.h"


Node* sourceNode = new Node(49, 50); //������
Node* targetNode = new Node(39, 67); //Ŀ���
std::list<Node> res_path; 
Node* all_node_pointers[100][100] = { NULL }; //�ռ����е��ָ��
std::list<Node> bench2bench_paths_buy[50][50] = {}; //�ռ�����̨֮���·�� ����Ѱ·�������������·���㼯
std::list<Node_c> bench2bench_paths_sell[50][50] = {};

std::priority_queue<Node*, std::vector<Node*>, cmp_a_star> pq_astar;//С����
std::priority_queue<Node*, std::vector<Node*>, cmp_dijkstra> pq_dijkstra;//С����

bool node_added[100][100] = { false }; //��ʾ���Ƿ��Ѿ������˶���
bool node_isSearched[100][100] = { false }; //��ʾ���Ƿ��Ѿ�������


Node::Node(int row, int col) {
	this->row = row;
	this->col = col;
	this->real_cost = 0;
	this->esti_cost = 0;
	this->parent = NULL;
	this->next = NULL;
}

Node::Node(int row, int col, Node *parent) {
	this->row = row;
	this->col = col;
	this->esti_cost = abs(this->row - targetNode->row) + abs(this->col - targetNode->col);
	this->parent = parent;
}

Node::Node(Node* p) {
	this->row = p->row;
	this->col = p->col;
	this->esti_cost = p->esti_cost;
	this->real_cost = p->real_cost;
	this->parent = p->parent;
	this->next = p->next;
}

int* Node::local() {
	int* p = new int[2];
	p[0] = this->row;
	p[1] = this->col;
	return p;
}

double Node::get_total_cost() {
	return this->real_cost + this->esti_cost;
}



/*
	A*�㷨�������� buy
*/
void iteration_astar_buy() {
	if (res_path.size() > 0 || pq_astar.size() == 0) return; //�ҵ���·�������û�ҵ�· ֱ�ӷ���
	//sort_heap(const_cast<Node*>(pq_astar.top()), const_cast<Node*>(pq_astar.top()) + pq_astar.size(), cmp_a_star());
	Node* node = pq_astar.top(); 
	pq_astar.pop();

	node_isSearched[node->row][node->col] = true;


	if (node->row == targetNode->row && node->col == targetNode->col) { //�ҵ��յ�

		Node* current_node = node;

		while (current_node->row != sourceNode->row || current_node->col != sourceNode->col) { //���ݸ��� ��������·��
			res_path.push_front(*current_node);
			current_node->parent->next = current_node; //����nextָ��
			current_node = current_node->parent;
		}
		res_path.push_front(*current_node);
		return;
	}
	
	//�˷���
	//��
	if (node->row - 1 >= 0 && map_data_ob_expand_buy[node->row - 1][node->col] != '#') {
		if (node_isSearched[node->row - 1][node->col]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *up = all_node_pointers[node->row - 1][node->col];
			if (up->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col]) {
				up->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col];
				up->parent = node;
			}
		}
		else if(!node_added[node->row - 1][node->col]) {
			Node* up = new Node(node->row - 1, node->col, node);
			up->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col];
			all_node_pointers[node->row - 1][node->col] = up;
			node_added[node->row - 1][node->col] = true;
			pq_astar.push(up);
		}

	}
	////����
	//if (node->row - 1 >= 0 && node->col - 1 >= 0 && map_data_ob_expand_buy[node->row - 1][node->col - 1] != '#') {
	//	if (node_isSearched[node->row - 1][node->col - 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *up_left = all_node_pointers[node->row - 1][node->col - 1];
	//		if (up_left->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col - 1]) {
	//			up_left->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col - 1];
	//			up_left->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row - 1][node->col - 1]) {
	//		Node* up_left = new Node(node->row - 1, node->col - 1, node);
	//		all_node_pointers[node->row - 1][node->col - 1] = up_left;
	//		up_left->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col - 1];
	//		node_added[node->row - 1][node->col - 1] = true;
	//		pq_astar.push(up_left);
	//	}
	//}
	////����
	//if (node->row - 1 >= 0 && node->col + 1 < 100 && map_data_ob_expand_buy[node->row - 1][node->col + 1] != '#') {
	//	if (node_isSearched[node->row - 1][node->col + 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *up_right = all_node_pointers[node->row - 1][node->col + 1];
	//		if (up_right->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col + 1]) {
	//			up_right->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col + 1];
	//			up_right->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row - 1][node->col + 1]) {
	//		Node* up_right = new Node(node->row - 1, node->col + 1, node);
	//		all_node_pointers[node->row - 1][node->col + 1] = up_right;
	//		up_right->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col + 1];
	//		node_added[node->row - 1][node->col + 1] = true;
	//		pq_astar.push(up_right);
	//	}
	//}
	//��
	if (node->row + 1 < 100 && map_data_ob_expand_buy[node->row + 1][node->col] != '#') {
		if (node_isSearched[node->row + 1][node->col]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *botton = all_node_pointers[node->row + 1][node->col];
			if (botton->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col]) {
				botton->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col];
				botton->parent = node;
			}
		}
		else if (!node_added[node->row + 1][node->col]) {
			Node* botton = new Node(node->row + 1, node->col, node);
			all_node_pointers[node->row + 1][node->col] = botton;
			botton->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col];
			node_added[node->row + 1][node->col] = true;
			pq_astar.push(botton);
		}
	}
	////����
	//if (node->row + 1 < 100 && node->col - 1 >= 0&& map_data_ob_expand_buy[node->row + 1][node->col - 1] != '#') {
	//	if (node_isSearched[node->row + 1][node->col - 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *botton_left = all_node_pointers[node->row + 1][node->col - 1];
	//		if (botton_left->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col - 1]) {
	//			botton_left->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col - 1];
	//			botton_left->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row + 1][node->col - 1]) {
	//		Node* botton_left = new Node(node->row + 1, node->col - 1, node);
	//		all_node_pointers[node->row + 1][node->col - 1] = botton_left;
	//		botton_left->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col - 1];
	//		node_added[node->row + 1][node->col - 1] = true;
	//		pq_astar.push(botton_left);
	//	}
	//}
	////����
	//if (node->row + 1 < 100 && node->col + 1 < 100 && map_data_ob_expand_buy[node->row + 1][node->col + 1] != '#') {
	//	if (node_isSearched[node->row + 1][node->col + 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *botton_right = all_node_pointers[node->row + 1][node->col + 1];
	//		if (botton_right->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col + 1]) {
	//			botton_right->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col + 1];
	//			botton_right->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row + 1][node->col] + 1) {
	//		Node* botton_right = new Node(node->row + 1, node->col + 1, node);
	//		all_node_pointers[node->row + 1][node->col + 1] = botton_right;
	//		botton_right->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col + 1];
	//		node_added[node->row + 1][node->col + 1] = true;
	//		pq_astar.push(botton_right);
	//	}
	//}
	//��
	if (node->col - 1 >= 0 && map_data_ob_expand_buy[node->row][node->col - 1] != '#') {
		if (node_isSearched[node->row][node->col - 1]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *left = all_node_pointers[node->row][node->col - 1];
			if (left->real_cost > node->real_cost + map_data_cost[node->row][node->col - 1]) {
				left->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
				left->parent = node;
			}
		}
		else if (!node_added[node->row][node->col] - 1) {
			Node* left =new Node(node->row, node->col - 1, node);
			all_node_pointers[node->row][node->col - 1] = left;
			left->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
			node_added[node->row][node->col - 1] = true;
			pq_astar.push(left);
		}
	}
	//��
	if (node->col + 1 < 100 && map_data_ob_expand_buy[node->row][node->col + 1] != '#') {
		if (node_isSearched[node->row][node->col + 1]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *right = all_node_pointers[node->row][node->col + 1];
			if (right->real_cost > node->real_cost + map_data_cost[node->row][node->col + 1]) {
				right->real_cost = node->real_cost + map_data_cost[node->row][node->col + 1];
				right->parent = node;
			}
		}
		else if (!node_added[node->row][node->col + 1]) {
			Node* right = new Node(node->row, node->col + 1, node);
			all_node_pointers[node->row][node->col + 1] = right;
			right->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
			node_added[node->row][node->col + 1] = true;
			pq_astar.push(right);
		}
	}
}

/*
input: ������꣬�յ�����
output: ·���㼯��
*/
std::list<Node> search_Astar_buy(int* source, int* target) {

	//ÿ�¿�ʼ��һ��·�� ��nodeAdded��res_path, source, target, pq_astar, pointer���� 
	for (int i = 0; i < 100; i++) {
		memset(node_added[i], false, sizeof node_added[i]);
		memset(node_isSearched[i], false, sizeof node_isSearched[i]);
		/*memset(all_node_pointers[i], NULL, sizeof all_node_pointers[i]);*/
	}
	
	while (!pq_astar.empty()) pq_astar.pop();
	res_path.clear();
	sourceNode = new  Node(source[0], source[1]);
	targetNode = new Node(target[0], target[1]);

	pq_astar.push(sourceNode);
	node_added[sourceNode->row][sourceNode->col] = true; //��ʾ�Ѿ������˶���
	all_node_pointers[sourceNode->row][sourceNode->col] = sourceNode; //�ռ����е��ָ��

	while ((res_path.size() == 0 && pq_astar.size() > 0)) {
		iteration_astar_buy();
	}
	//��� res_path.size() Ϊ0��ʾû�ҵ���ͨ·��

	//for (Node t : res_path) {
	//	map_data_to_look_path[t.local()[0]][t.local()[1]] = '+';
	//}

	//std::fstream f;
	//f.open(".\\data.txt", std::ios::out);
	//
	//for (int i = 0; i < 100; i++) {
	//	for (int j = 0; j < 100; j++) {
	//		f << char(map_data_to_look_path[i][j]);
	//	}
	//	f << "\n";
	//}
	//f.close();
	//for (int i = 0; i < 100; i++) {
	//	for (int j = 0; j < 100; j++) {
	//		map_data_to_look_path[i][j] = map_data_ob_expand_sell[i][j];
	//	}
	//}
	return res_path;
}

/*
	A*�㷨�������� sell
*/
void iteration_astar_sell() {
	if (res_path.size() > 0 || pq_astar.size() == 0) return; //�ҵ���·�������û�ҵ�· ֱ�ӷ���
	//sort_heap(const_cast<Node*>(pq_astar.top()), const_cast<Node*>(pq_astar.top() + pq_astar.size()), cmp_a_star());
	Node* node = pq_astar.top();
	pq_astar.pop();

	node_isSearched[node->row][node->col] = true;


	if (node->row == targetNode->row && node->col == targetNode->col) { //�ҵ��յ�

		Node* current_node = node;

		while (current_node->row != sourceNode->row || current_node->col != sourceNode->col) { //���ݸ��� ��������·��
			res_path.push_front(*current_node);
			current_node->parent->next = current_node; //����nextָ��
			current_node = current_node->parent;
		}
		res_path.push_front(*current_node);
		return;
	}


	//�˷���
//��
	if (node->row - 1 >= 0 && map_data_ob_expand_sell[node->row - 1][node->col] != '#') {
		if (node_isSearched[node->row - 1][node->col]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *up = all_node_pointers[node->row - 1][node->col];
			if (up->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col]) {
				up->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col];
				up->parent = node;
			}
		}
		else if (!node_added[node->row - 1][node->col]) {
			Node* up = new Node(node->row - 1, node->col, node);
			up->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col];
			all_node_pointers[node->row - 1][node->col] = up;
			node_added[node->row - 1][node->col] = true;
			pq_astar.push(up);
		}

	}
	////����
	//if (node->row - 1 >= 0 && node->col - 1 >= 0 && map_data_ob_expand_sell[node->row - 1][node->col - 1] != '#') {
	//	if (node_isSearched[node->row - 1][node->col - 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *up_left = all_node_pointers[node->row - 1][node->col - 1];
	//		if (up_left->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col - 1]) {
	//			up_left->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col - 1];
	//			up_left->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row - 1][node->col - 1]) {
	//		Node* up_left = new Node(node->row - 1, node->col - 1, node);
	//		all_node_pointers[node->row - 1][node->col - 1] = up_left;
	//		up_left->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col - 1];
	//		node_added[node->row - 1][node->col - 1] = true;
	//		pq_astar.push(up_left);
	//	}
	//}
	////����
	//if (node->row - 1 >= 0 && node->col + 1 < 100 && map_data_ob_expand_sell[node->row - 1][node->col + 1] != '#') {
	//	if (node_isSearched[node->row - 1][node->col + 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *up_right = all_node_pointers[node->row - 1][node->col + 1];
	//		if (up_right->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col + 1]) {
	//			up_right->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col + 1];
	//			up_right->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row - 1][node->col + 1]) {
	//		Node* up_right = new Node(node->row - 1, node->col + 1, node);
	//		all_node_pointers[node->row - 1][node->col + 1] = up_right;
	//		up_right->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col + 1];
	//		node_added[node->row - 1][node->col + 1] = true;
	//		pq_astar.push(up_right);
	//	}
	//}
	//��
	if (node->row + 1 < 100 && map_data_ob_expand_sell[node->row + 1][node->col] != '#') {
		if (node_isSearched[node->row + 1][node->col]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *botton = all_node_pointers[node->row + 1][node->col];
			if (botton->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col]) {
				botton->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col];
				botton->parent = node;
			}
		}
		else if (!node_added[node->row + 1][node->col]) {
			Node* botton = new Node(node->row + 1, node->col, node);
			all_node_pointers[node->row + 1][node->col] = botton;
			botton->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col];
			node_added[node->row + 1][node->col] = true;
			pq_astar.push(botton);
		}
	}
	////����
	//if (node->row + 1 < 100 && node->col - 1 >= 0 && map_data_ob_expand_sell[node->row + 1][node->col - 1] != '#') {
	//	if (node_isSearched[node->row + 1][node->col - 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *botton_left = all_node_pointers[node->row + 1][node->col - 1];
	//		if (botton_left->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col - 1]) {
	//			botton_left->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col - 1];
	//			botton_left->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row + 1][node->col - 1]) {
	//		Node* botton_left = new Node(node->row + 1, node->col - 1, node);
	//		all_node_pointers[node->row + 1][node->col - 1] = botton_left;
	//		botton_left->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col - 1];
	//		node_added[node->row + 1][node->col - 1] = true;
	//		pq_astar.push(botton_left);
	//	}
	//}
	////����
	//if (node->row + 1 < 100 && node->col + 1 < 100 && map_data_ob_expand_sell[node->row + 1][node->col + 1] != '#') {
	//	if (node_isSearched[node->row + 1][node->col + 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *botton_right = all_node_pointers[node->row + 1][node->col + 1];
	//		if (botton_right->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col + 1]) {
	//			botton_right->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col + 1];
	//			botton_right->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row + 1][node->col] + 1) {
	//		Node* botton_right = new Node(node->row + 1, node->col + 1, node);
	//		all_node_pointers[node->row + 1][node->col + 1] = botton_right;
	//		botton_right->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col + 1];
	//		node_added[node->row + 1][node->col + 1] = true;
	//		pq_astar.push(botton_right);
	//	}
	//}
	//��
	if (node->col - 1 >= 0 && map_data_ob_expand_sell[node->row][node->col - 1] != '#') {
		if (node_isSearched[node->row][node->col - 1]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *left = all_node_pointers[node->row][node->col - 1];
			if (left->real_cost > node->real_cost + map_data_cost[node->row][node->col - 1]) {
				left->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
				left->parent = node;
			}
		}
		else if (!node_added[node->row][node->col] - 1) {
			Node* left = new Node(node->row, node->col - 1, node);
			all_node_pointers[node->row][node->col - 1] = left;
			left->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
			node_added[node->row][node->col - 1] = true;
			pq_astar.push(left);
		}
	}
	//��
	if (node->col + 1 < 100 && map_data_ob_expand_sell[node->row][node->col + 1] != '#') {
		if (node_isSearched[node->row][node->col + 1]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *right = all_node_pointers[node->row][node->col + 1];
			if (right->real_cost > node->real_cost + map_data_cost[node->row][node->col + 1]) {
				right->real_cost = node->real_cost + map_data_cost[node->row][node->col + 1];
				right->parent = node;
			}
		}
		else if (!node_added[node->row][node->col + 1]) {
			Node* right = new Node(node->row, node->col + 1, node);
			all_node_pointers[node->row][node->col + 1] = right;
			right->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
			node_added[node->row][node->col + 1] = true;
			pq_astar.push(right);
		}
	}
}

/*
input: ������꣬�յ�����
output: ·���㼯��
*/
std::list<Node> search_Astar_sell(int* source, int* target) {

	//ÿ�¿�ʼ��һ��·�� ��nodeAdded��res_path, source, target, pq_astar, pointer���� 
	for (int i = 0; i < 100; i++) {
		memset(node_added[i], false, sizeof node_added[i]);
		memset(node_isSearched[i], false, sizeof node_isSearched[i]);
		/*memset(all_node_pointers[i], NULL, sizeof all_node_pointers[i]);*/
	}

	while (!pq_astar.empty()) pq_astar.pop();
	res_path.clear();
	sourceNode = new  Node(source[0], source[1]);
	targetNode = new Node(target[0], target[1]);

	pq_astar.push(sourceNode);
	node_added[sourceNode->row][sourceNode->col] = true; //��ʾ�Ѿ������˶���
	all_node_pointers[sourceNode->row][sourceNode->col] = sourceNode; //�ռ����е��ָ��

	while ((res_path.size() == 0 && pq_astar.size() > 0)) {
		iteration_astar_sell();
	}
	//��� res_path.size() Ϊ0��ʾû�ҵ���ͨ·��

	//for (Node t : res_path) {
	//	map_data_to_look_path[t.local()[0]][t.local()[1]] = '+';
	//}

	//std::fstream f;
	//f.open(".\\data.txt", std::ios::out);
	//
	//for (int i = 0; i < 100; i++) {
	//	for (int j = 0; j < 100; j++) {
	//		f << char(map_data_to_look_path[i][j]);
	//	}
	//	f << "\n";
	//}
	//f.close();
	//for (int i = 0; i < 100; i++) {
	//	for (int j = 0; j < 100; j++) {
	//		map_data_to_look_path[i][j] = map_data[i][j];
	//	}
	//}
	return res_path;
}


/*
	dijkstra�㷨�������� buy
*/
void iteration_dijkstra_buy() {
	if (res_path.size() > 0 || pq_dijkstra.size() == 0) return; //�ҵ���·�������û�ҵ�· ֱ�ӷ���
	//sort_heap(const_cast<Node*>(pq_dijkstra.top()), const_cast<Node*>(pq_dijkstra.top()) + pq_dijkstra.size(), cmp_a_star());
	Node* node = pq_dijkstra.top();
	pq_dijkstra.pop();

	node_isSearched[node->row][node->col] = true;


	if (node->row == targetNode->row && node->col == targetNode->col) { //�ҵ��յ�

		Node* current_node = node;

		while (current_node->row != sourceNode->row || current_node->col != sourceNode->col) { //���ݸ��� ��������·��
			res_path.push_front(*current_node);
			current_node->parent->next = current_node; //����nextָ��
			current_node = current_node->parent;
		}
		res_path.push_front(*current_node);
		return;
	}

	//�˷���
	//��
	if (node->row - 1 >= 0 && map_data_ob_expand_buy[node->row - 1][node->col] != '#') {
		if (node_isSearched[node->row - 1][node->col]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *up = all_node_pointers[node->row - 1][node->col];
			if (up->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col]) {
				up->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col];
				up->parent = node;
			}
		}
		else if (!node_added[node->row - 1][node->col]) {
			Node* up = new Node(node->row - 1, node->col, node);
			up->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col];
			all_node_pointers[node->row - 1][node->col] = up;
			node_added[node->row - 1][node->col] = true;
			pq_dijkstra.push(up);
		}

	}
	////����
	//if (node->row - 1 >= 0 && node->col - 1 >= 0 && map_data_ob_expand_buy[node->row - 1][node->col - 1] != '#') {
	//	if (node_isSearched[node->row - 1][node->col - 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *up_left = all_node_pointers[node->row - 1][node->col - 1];
	//		if (up_left->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col - 1]) {
	//			up_left->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col - 1];
	//			up_left->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row - 1][node->col - 1]) {
	//		Node* up_left = new Node(node->row - 1, node->col - 1, node);
	//		all_node_pointers[node->row - 1][node->col - 1] = up_left;
	//		up_left->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col - 1];
	//		node_added[node->row - 1][node->col - 1] = true;
	//		pq_dijkstra.push(up_left);
	//	}
	//}
	////����
	//if (node->row - 1 >= 0 && node->col + 1 < 100 && map_data_ob_expand_buy[node->row - 1][node->col + 1] != '#') {
	//	if (node_isSearched[node->row - 1][node->col + 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *up_right = all_node_pointers[node->row - 1][node->col + 1];
	//		if (up_right->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col + 1]) {
	//			up_right->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col + 1];
	//			up_right->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row - 1][node->col + 1]) {
	//		Node* up_right = new Node(node->row - 1, node->col + 1, node);
	//		all_node_pointers[node->row - 1][node->col + 1] = up_right;
	//		up_right->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col + 1];
	//		node_added[node->row - 1][node->col + 1] = true;
	//		pq_dijkstra.push(up_right);
	//	}
	//}
	//��
	if (node->row + 1 < 100 && map_data_ob_expand_buy[node->row + 1][node->col] != '#') {
		if (node_isSearched[node->row + 1][node->col]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *botton = all_node_pointers[node->row + 1][node->col];
			if (botton->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col]) {
				botton->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col];
				botton->parent = node;
			}
		}
		else if (!node_added[node->row + 1][node->col]) {
			Node* botton = new Node(node->row + 1, node->col, node);
			all_node_pointers[node->row + 1][node->col] = botton;
			botton->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col];
			node_added[node->row + 1][node->col] = true;
			pq_dijkstra.push(botton);
		}
	}
	////����
	//if (node->row + 1 < 100 && node->col - 1 >= 0 && map_data_ob_expand_buy[node->row + 1][node->col - 1] != '#') {
	//	if (node_isSearched[node->row + 1][node->col - 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *botton_left = all_node_pointers[node->row + 1][node->col - 1];
	//		if (botton_left->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col - 1]) {
	//			botton_left->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col - 1];
	//			botton_left->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row + 1][node->col - 1]) {
	//		Node* botton_left = new Node(node->row + 1, node->col - 1, node);
	//		all_node_pointers[node->row + 1][node->col - 1] = botton_left;
	//		botton_left->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col - 1];
	//		node_added[node->row + 1][node->col - 1] = true;
	//		pq_dijkstra.push(botton_left);
	//	}
	//}
	////����
	//if (node->row + 1 < 100 && node->col + 1 < 100 && map_data_ob_expand_buy[node->row + 1][node->col + 1] != '#') {
	//	if (node_isSearched[node->row + 1][node->col + 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *botton_right = all_node_pointers[node->row + 1][node->col + 1];
	//		if (botton_right->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col + 1]) {
	//			botton_right->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col + 1];
	//			botton_right->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row + 1][node->col] + 1) {
	//		Node* botton_right = new Node(node->row + 1, node->col + 1, node);
	//		all_node_pointers[node->row + 1][node->col + 1] = botton_right;
	//		botton_right->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col + 1];
	//		node_added[node->row + 1][node->col + 1] = true;
	//		pq_dijkstra.push(botton_right);
	//	}
	//}
	//��
	if (node->col - 1 >= 0 && map_data_ob_expand_buy[node->row][node->col - 1] != '#') {
		if (node_isSearched[node->row][node->col - 1]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *left = all_node_pointers[node->row][node->col - 1];
			if (left->real_cost > node->real_cost + map_data_cost[node->row][node->col - 1]) {
				left->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
				left->parent = node;
			}
		}
		else if (!node_added[node->row][node->col] - 1) {
			Node* left = new Node(node->row, node->col - 1, node);
			all_node_pointers[node->row][node->col - 1] = left;
			left->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
			node_added[node->row][node->col - 1] = true;
			pq_dijkstra.push(left);
		}
	}
	//��
	if (node->col + 1 < 100 && map_data_ob_expand_buy[node->row][node->col + 1] != '#') {
		if (node_isSearched[node->row][node->col + 1]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *right = all_node_pointers[node->row][node->col + 1];
			if (right->real_cost > node->real_cost + map_data_cost[node->row][node->col + 1]) {
				right->real_cost = node->real_cost + map_data_cost[node->row][node->col + 1];
				right->parent = node;
			}
		}
		else if (!node_added[node->row][node->col + 1]) {
			Node* right = new Node(node->row, node->col + 1, node);
			all_node_pointers[node->row][node->col + 1] = right;
			right->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
			node_added[node->row][node->col + 1] = true;
			pq_dijkstra.push(right);
		}
	}
}

/*
input: ������꣬�յ�����
output: ·���㼯��
*/
std::list<Node> search_dijkstra_buy(int* source, int* target) {

	//ÿ�¿�ʼ��һ��·�� ��nodeAdded��res_path, source, target, pq_astar, pointer���� 
	for (int i = 0; i < 100; i++) {
		memset(node_added[i], false, sizeof node_added[i]);
		memset(node_isSearched[i], false, sizeof node_isSearched[i]);
		//memset(all_node_pointers[i], NULL, sizeof all_node_pointers[i]);
	}

	while (!pq_dijkstra.empty()) pq_dijkstra.pop();
	res_path.clear();
	sourceNode = new  Node(source[0], source[1]);
	targetNode = new Node(target[0], target[1]);

	pq_dijkstra.push(sourceNode);
	node_added[sourceNode->row][sourceNode->col] = true; //��ʾ�Ѿ������˶���
	all_node_pointers[sourceNode->row][sourceNode->col] = sourceNode; //�ռ����е��ָ��

	while ((res_path.size() == 0 && pq_dijkstra.size() > 0)) {
		iteration_dijkstra_buy();
	}
	//��� res_path.size() Ϊ0��ʾû�ҵ���ͨ·��

	//for (Node t : res_path) {
	//	map_data_to_look_path[t.local()[0]][t.local()[1]] = '+';
	//}

	//std::fstream f;
	//f.open(".\\data.txt", std::ios::out);

	//for (int i = 0; i < 100; i++) {
	//	for (int j = 0; j < 100; j++) {
	//		f << char(map_data_to_look_path[i][j]);
	//	}
	//	f << "\n";
	//}
	//f.close();
	//for (int i = 0; i < 100; i++) {
	//	for (int j = 0; j < 100; j++) {
	//		map_data_to_look_path[i][j] = map_data_ob_expand_sell[i][j];
	//	}
	//}
	return res_path;
}

/*
	dijkstra�㷨�������� sell
*/
void iteration_dijkstra_sell() {
	if (res_path.size() > 0 || pq_dijkstra.size() == 0) return; //�ҵ���·�������û�ҵ�· ֱ�ӷ���
	//sort_heap(const_cast<Node*>(pq_astar.top()), const_cast<Node*>(pq_astar.top() + pq_astar.size()), cmp_a_star());
	Node* node = pq_dijkstra.top();
	pq_dijkstra.pop();

	node_isSearched[node->row][node->col] = true;


	if (node->row == targetNode->row && node->col == targetNode->col) { //�ҵ��յ�

		Node* current_node = node;

		while (current_node->row != sourceNode->row || current_node->col != sourceNode->col) { //���ݸ��� ��������·��
			res_path.push_front(*current_node);
			current_node->parent->next = current_node; //����nextָ��
			current_node = current_node->parent;
		}
		res_path.push_front(*current_node);
		return;
	}


	//�˷���
//��
	if (node->row - 1 >= 0 && map_data_ob_expand_sell[node->row - 1][node->col] != '#') {
		if (node_isSearched[node->row - 1][node->col]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *up = all_node_pointers[node->row - 1][node->col];
			if (up->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col]) {
				up->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col];
				up->parent = node;
			}
		}
		else if (!node_added[node->row - 1][node->col]) {
			Node* up = new Node(node->row - 1, node->col, node);
			up->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col];
			all_node_pointers[node->row - 1][node->col] = up;
			node_added[node->row - 1][node->col] = true;
			pq_dijkstra.push(up);
		}

	}
	////����
	//if (node->row - 1 >= 0 && node->col - 1 >= 0 && map_data_ob_expand_sell[node->row - 1][node->col - 1] != '#') {
	//	if (node_isSearched[node->row - 1][node->col - 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *up_left = all_node_pointers[node->row - 1][node->col - 1];
	//		if (up_left->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col - 1]) {
	//			up_left->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col - 1];
	//			up_left->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row - 1][node->col - 1]) {
	//		Node* up_left = new Node(node->row - 1, node->col - 1, node);
	//		all_node_pointers[node->row - 1][node->col - 1] = up_left;
	//		up_left->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col - 1];
	//		node_added[node->row - 1][node->col - 1] = true;
	//		pq_dijkstra.push(up_left);
	//	}
	//}
	////����
	//if (node->row - 1 >= 0 && node->col + 1 < 100 && map_data_ob_expand_sell[node->row - 1][node->col + 1] != '#') {
	//	if (node_isSearched[node->row - 1][node->col + 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *up_right = all_node_pointers[node->row - 1][node->col + 1];
	//		if (up_right->real_cost > node->real_cost + map_data_cost[node->row - 1][node->col + 1]) {
	//			up_right->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col + 1];
	//			up_right->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row - 1][node->col + 1]) {
	//		Node* up_right = new Node(node->row - 1, node->col + 1, node);
	//		all_node_pointers[node->row - 1][node->col + 1] = up_right;
	//		up_right->real_cost = node->real_cost + map_data_cost[node->row - 1][node->col + 1];
	//		node_added[node->row - 1][node->col + 1] = true;
	//		pq_dijkstra.push(up_right);
	//	}
	//}
	//��
	if (node->row + 1 < 100 && map_data_ob_expand_sell[node->row + 1][node->col] != '#') {
		if (node_isSearched[node->row + 1][node->col]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *botton = all_node_pointers[node->row + 1][node->col];
			if (botton->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col]) {
				botton->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col];
				botton->parent = node;
			}
		}
		else if (!node_added[node->row + 1][node->col]) {
			Node* botton = new Node(node->row + 1, node->col, node);
			all_node_pointers[node->row + 1][node->col] = botton;
			botton->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col];
			node_added[node->row + 1][node->col] = true;
			pq_dijkstra.push(botton);
		}
	}
	////����
	//if (node->row + 1 < 100 && node->col - 1 >= 0 && map_data_ob_expand_sell[node->row + 1][node->col - 1] != '#') {
	//	if (node_isSearched[node->row + 1][node->col - 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *botton_left = all_node_pointers[node->row + 1][node->col - 1];
	//		if (botton_left->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col - 1]) {
	//			botton_left->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col - 1];
	//			botton_left->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row + 1][node->col - 1]) {
	//		Node* botton_left = new Node(node->row + 1, node->col - 1, node);
	//		all_node_pointers[node->row + 1][node->col - 1] = botton_left;
	//		botton_left->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col - 1];
	//		node_added[node->row + 1][node->col - 1] = true;
	//		pq_dijkstra.push(botton_left);
	//	}
	//}
	////����
	//if (node->row + 1 < 100 && node->col + 1 < 100 && map_data_ob_expand_sell[node->row + 1][node->col + 1] != '#') {
	//	if (node_isSearched[node->row + 1][node->col + 1]) { //�Ѿ��������� �жϾ��� ���¸���
	//		Node *botton_right = all_node_pointers[node->row + 1][node->col + 1];
	//		if (botton_right->real_cost > node->real_cost + map_data_cost[node->row + 1][node->col + 1]) {
	//			botton_right->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col + 1];
	//			botton_right->parent = node;
	//		}
	//	}
	//	else if (!node_added[node->row + 1][node->col] + 1) {
	//		Node* botton_right = new Node(node->row + 1, node->col + 1, node);
	//		all_node_pointers[node->row + 1][node->col + 1] = botton_right;
	//		botton_right->real_cost = node->real_cost + map_data_cost[node->row + 1][node->col + 1];
	//		node_added[node->row + 1][node->col + 1] = true;
	//		pq_dijkstra.push(botton_right);
	//	}
	//}
	//��
	if (node->col - 1 >= 0 && map_data_ob_expand_sell[node->row][node->col - 1] != '#') {
		if (node_isSearched[node->row][node->col - 1]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *left = all_node_pointers[node->row][node->col - 1];
			if (left->real_cost > node->real_cost + map_data_cost[node->row][node->col - 1]) {
				left->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
				left->parent = node;
			}
		}
		else if (!node_added[node->row][node->col] - 1) {
			Node* left = new Node(node->row, node->col - 1, node);
			all_node_pointers[node->row][node->col - 1] = left;
			left->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
			node_added[node->row][node->col - 1] = true;
			pq_dijkstra.push(left);
		}
	}
	//��
	if (node->col + 1 < 100 && map_data_ob_expand_sell[node->row][node->col + 1] != '#') {
		if (node_isSearched[node->row][node->col + 1]) { //�Ѿ��������� �жϾ��� ���¸���
			Node *right = all_node_pointers[node->row][node->col + 1];
			if (right->real_cost > node->real_cost + map_data_cost[node->row][node->col + 1]) {
				right->real_cost = node->real_cost + map_data_cost[node->row][node->col + 1];
				right->parent = node;
			}
		}
		else if (!node_added[node->row][node->col + 1]) {
			Node* right = new Node(node->row, node->col + 1, node);
			all_node_pointers[node->row][node->col + 1] = right;
			right->real_cost = node->real_cost + map_data_cost[node->row][node->col - 1];
			node_added[node->row][node->col + 1] = true;
			pq_dijkstra.push(right);
		}
	}
}

/*
input: ������꣬�յ�����
output: ·���㼯��
*/
std::list<Node> search_dijkstra_sell(int* source, int* target) {

	//ÿ�¿�ʼ��һ��·�� ��nodeAdded��res_path, source, target, pq_astar, pointer���� 
	for (int i = 0; i < 100; i++) {
		memset(node_added[i], false, sizeof node_added[i]);
		memset(node_isSearched[i], false, sizeof node_isSearched[i]);
		//memset(all_node_pointers[i], NULL, sizeof all_node_pointers[i]);
	}

	while (!pq_dijkstra.empty()) pq_dijkstra.pop();
	res_path.clear();
	sourceNode = new  Node(source[0], source[1]);
	targetNode = new Node(target[0], target[1]);

	pq_dijkstra.push(sourceNode);
	node_added[sourceNode->row][sourceNode->col] = true; //��ʾ�Ѿ������˶���
	all_node_pointers[sourceNode->row][sourceNode->col] = sourceNode; //�ռ����е��ָ��

	while ((res_path.size() == 0 && pq_dijkstra.size() > 0)) {
		iteration_dijkstra_sell();
	}
	//��� res_path.size() Ϊ0��ʾû�ҵ���ͨ·��

	//for (Node t : res_path) {
	//	map_data_to_look_path[t.local()[0]][t.local()[1]] = '+';
	//}

	//std::fstream f;
	//f.open(".\\data.txt", std::ios::out);

	//for (int i = 0; i < 100; i++) {
	//	for (int j = 0; j < 100; j++) {
	//		f << char(map_data_to_look_path[i][j]);
	//	}
	//	f << "\n";
	//}
	//f.close();
	//for (int i = 0; i < 100; i++) {
	//	for (int j = 0; j < 100; j++) {
	//		map_data_to_look_path[i][j] = map_data[i][j];
	//	}
	//}
	return res_path;
}

////��ʼ������̨֮���·�� �������bench2bench_paths_buy��
//void init_bench2bench_paths() {
//	for (int i = 0; i < init_bench_sum; i++)
//	{
//		for (int j = 0; j < init_bench_sum; j++)
//		{
//			//if (i > j && bench2bench_paths_buy[j][i].size() > 0)
//			//{
//			//	std::list<Node> temp = bench2bench_paths_buy[j][i]; //����·��֮�� �Ͳ��ù�parentָ��
//			//	temp.reverse();
//			//	std::list<Node> new_temp;
//			//	for (auto p : temp) {
//			//		p.next = p.parent;
//			//		new_temp.push_back(p);
//			//	}
//			//	bench2bench_paths_buy[i][j] = new_temp;
//			//	continue;
//			//}
//
//			if (i == j) continue;
//
//			int b_x0 = bench_seq_dist[i][0];
//			int b_y0 = bench_seq_dist[i][1];
//
//			int b_x1 = bench_seq_dist[j][0];
//			int b_y1 = bench_seq_dist[j][1];
//
//			int source[2] = { b_x0, b_y0 };
//			int target[2] = { b_x1, b_y1 };
//
//			int buy_bench_type = bench_seq_type_dist[i];
//			int sell_bench_type = bench_seq_type_dist[j];
//			//source�ǵ͵ȼ���target�Ǹߵȼ��Ļ�������target��sourceҪȥ���Ĺ���̨
//			if (buy_bench_type < sell_bench_type)
//			{
//
//
//				bool source_can_to_target_sell = false;
//				for (auto bench_type : where_to_sell[buy_bench_type])
//					if (sell_bench_type == bench_type) { source_can_to_target_sell = true; break; }
//
//				if (!source_can_to_target_sell) continue;//j��ŵĹ���̨����Ҫȥ���Ĺ���̨������
//
//
//				std::list<Node> list_sell = search_Astar_sell(source, target);
//				//std::list<Node> list_sell = search_dijkstra_sell(source, target);
//				if (list_sell.size() != 0)
//				{
//					bench2bench_paths_sell[i][j] = list_sell;
//					//fprintf(stderr, "sell!!!source bench seq:%d,source bench type:%d,target bench seq:%d, target bench type:%d,path length:%d \n", i, buy_bench_type, j, sell_bench_type, list_sell.size());
//				}
//
//				//std::list<Node> list_buy = search_Astar_buy(source, target);
//				////std::list<Node> list_buy = search_dijkstra_buy(source, target);
//				//if (list_buy.size() != 0)
//				//{
//				//	bench2bench_paths_buy[i][j] = list_buy;
//				//	//fprintf(stderr, "buy!!!source bench seq:%d,source bench type:%d,target bench seq:%d, target bench type:%d,path length:%d \n", i, buy_bench_type, j, sell_bench_type, list_buy.size());
//				//}
//			}
//		}
//	}
//}

