#include "globalVariables.h"

int robot_priority[4] = { 0 };

void set_robot_priority() {
	for (int i = 0; i < RobotNum; i++) {
		if (free_robots[i]) {
			robot_priority[i] = -10 + i; continue;
		}

		//保证携带高级产品的机器人更为优先
		if (working_robots_queue[i].buyOrSell == 1) {
			//买
			robot_priority[i] = working_robots_queue[i].productType * 5 + i;
		}
		else {
			robot_priority[i] = working_robots_queue[i].productType * 10 + 100 + i; 
		}

	}
}

//检测自己一定距离内是否有优先级更高的机器人
int return_highter_robot(int robotID) {
	set_robot_priority();
	int res = -1, priority = 999;
	Point c_position = { robot.grid[robotID][0], robot.grid[robotID][1] };

	for (int r_id = 0; r_id < RobotNum; r_id++) {
		if (r_id == robotID) continue;
		if (robot_priority[robotID] > robot_priority[r_id]) continue;
		Point o_position = { robot.grid[r_id][0], robot.grid[r_id][1] };
		int dist = absI(c_position.x - o_position.x) + absI(c_position.y - o_position.y);
		if (dist > 15) continue; //当前机器人距离其他机器人在一定的距离之外 不做判断

		//可能存在多个机器人挤在一起的情况 暂时判断两个机器人路径冲突在一起
		if (priority > robot_priority[r_id]) {
			priority = robot_priority[r_id];
			res = r_id;
		} //返回当前机器人周围一定距离内的更高级的机器人
	}
	return res;
}


bool check_in_path(int c_robot_id, int o_robot_id) {
	if (o_robot_id == -1) return false;
	std::list<Node_c> path = working_robots_queue[o_robot_id].path;

	//判断一下高级机器人周围是否有障碍物 没有则不进行避免路径
	Point o_position = { robot.grid[o_robot_id][0], robot.grid[o_robot_id][1] };
	bool judge_have_ob = false;
	for (int row = o_position.x - 2; row <= o_position.x + 2; row++) {
		for (int col = o_position.y - 2; col <= o_position.y + 2; col++) {
			if (row < 0 || row > 99 || col < 0 || col > 99) continue;
			if (map_data[row][col] == '#') {
				judge_have_ob = true; break;
			}
		}
	}
	if (!judge_have_ob) return false;
	Point c_position = { robot.grid[c_robot_id][0], robot.grid[c_robot_id][1] };

	for (Node_c p : path) {
		double dist = (c_position.x - p.point.x) * (c_position.x - p.point.x) + (c_position.y - p.point.y) * (c_position.y - p.point.y);
		if (dist <= 8) return true;
	}
	return false;
}

//如果当前就机器人附近有优先级更高的机器人 并且自己在他的路径当中 自己就找一个不在路径点周围的点进行让路
void out_of_way_robot(int r_id, int higher_r_id) {

	//上方2格条件满足 当前机器人进行让路 将路径点往周围扩散2格
	bool path_point[100][100] = { false }; //表示是否是路径点或其周围
	for (Node_c p : working_robots_queue[higher_r_id].path) {
		for (int i = p.point.x - 2; i <= p.point.x + 2; i++) {
			for (int j = p.point.y - 2; j <= p.point.y + 2; j++) {
				if (i < 0 || i > 99 || j < 0 || j>99) continue;
				path_point[i][j] = true; 
			}
		}
	}

	//当前机器人位置蔓延 知道蔓延到一个合适的地方
	bool is_added[100][100] = { false };
	std::queue<Point> search_position;
	std::list<Point> added_node;
	std::list<Node_c> paths = working_robots_queue[higher_r_id].path;

	Point robot_position = { robot.grid[r_id][0], robot.grid[r_id][1] };
	Point higher_r_position = { robot.grid[higher_r_id][0], robot.grid[higher_r_id][1] };
	int limited_spred_num = 100;

	added_node.push_back(robot_position);
	is_added[robot_position.x][robot_position.y] = true;
	while (limited_spred_num > 0 && added_node.size() > 0) {

		Point p = added_node.front();
		added_node.pop_front();

		Point next_points[4] = {
			{p.x + 1, p.y},
			{p.x - 1, p.y},
			{p.x, p.y + 1},
			{p.x, p.y - 1},
		};
		//如果这个位置离高优先级机器人距离大于2 并且不是墙 可以加入added_node中
		for (int i = 0; i < 4; i++) {
			limited_spred_num--;
			int dis = (next_points[i].x - higher_r_position.x) * (next_points[i].x - higher_r_position.x) + (next_points[i].y - higher_r_position.y) * (next_points[i].y - higher_r_position.y);
			if (dis <= 8 || map_data[next_points[i].x][next_points[i].y] == '#' || is_added[next_points[i].x][next_points[i].y]) continue;
			search_position.push(next_points[i]);
			added_node.push_back(next_points[i]);
			is_added[next_points[i].x][next_points[i].y] = true;
		}
	}
	//防止暂时没有搜到合适的点
	if (search_position.size() > 0) {
		Point farest_point = search_position.back();

		std::list<Node_c> new_path = find_path({ robot.grid[r_id][0], robot.grid[r_id][1] }, { farest_point.x, farest_point.y }, false);
		working_robots_queue[r_id].path = new_path; 
	}


	while (search_position.size() > 0) {
		Point current_p = search_position.front();
		search_position.pop();
		if (!path_point[current_p.x][current_p.y]) {
			std::list<Node_c> new_path = find_path({ robot.grid[r_id][0], robot.grid[r_id][1] }, { current_p.x, current_p.y }, false);

			//for (int i = 0; i < 100; i++)
			//	for (int j = 0; j < 100; j++)
			//		map_data_to_look_path[i][j] = map_data[i][j];

			//std::list<Node_c> tem = new_path;
			//if (tem.size() >= 2) {
			//	tem.pop_front();
			//	tem.pop_back();
			//}
			//for (Node_c t : tem) {
			//	map_data_to_look_path[t.point.x][t.point.y] = '*';
			//}

			//std::fstream f;
			//f.open(".\\data.txt", std::ios::out);

			//for (int i = 0; i < 100; i++) {
			//	for (int j = 0; j < 100; j++) {
			//		f << map_data_to_look_path[i][j];
			//	}
			//	f << "\n";
			//}
			//f.close();
			

			if (new_path.size() > 0) {
				working_robots_queue[r_id].path = new_path; //在findlines或者speedcontrol中 可以使用这搜出来路径
				return;
			} //找到了一个不在路径点 并且可以过去的点
		}
	}
}