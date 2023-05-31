#include"globalVariables.h"


int bench_seq_map[row1][col1];//工作台和机器人编号的地图
int bench_seq_dist[50][2]; //第一维是工作台编号，第二维是工作台坐标
int robort_seq_map[4][2];//第一维是机器人编号，第二维是机器人初始坐标
std::map<int, std::vector<int>> roborts_can_go_bench_map;
bool roborts_can_go_bench_buy_array[4][50] = { false };
bool roborts_can_go_bench_sell_array[4][50] = { false };
std::vector<int> r_can_go_bench_vec;
std::vector<int> together_robort;
int init_bench_sum = -1;
int bench_seq_type_dist[50] = { 0 };


void BFS(int r_id, int x0,int y0)
{
	// 初始化标记矩阵
	bool inq[row1][col1] = { false };
	// 初始化队列
	std::queue<coordinate> que;

	coordinate begin;
	begin.x = x0;
	begin.y = y0;
	// 初始位置入队
	que.push(begin);
	inq[x0][y0] = true;
	while (!que.empty())
	{
		coordinate tmp = que.front();
		que.pop();
		int x = tmp.x;
		int y = tmp.y;
		// 遍历它可以到达的位置
		Point next_points[4] = {
			{x + 1, y},
			{x - 1, y},
			{x, y + 1},
			{x, y - 1},
		};
		for (int k = 0; k <= 3; k++)
		{
			int i = next_points[k].x, j = next_points[k].y;
			
			// 跳过非法位置
			if (i < 0 || i >= row1)
				continue;
			if (j < 0 || j >= col1)
				continue;
			// 如果没有遍历过
			if (!inq[i][j])
			{
				if (map_data[i][j] >= '1' && map_data[i][j] <= '9') // 是工作台
				{
					int bench_seq = getBenchSeq(i, j); // 根据坐标得到工作台编号的函数

					r_can_go_bench_vec.push_back(bench_seq);
						//将<i,j>入队
						coordinate cd;
					cd.x = i;
					cd.y = j;
					que.push(cd);
				}
				else if (map_data[i][j] == '#') // 是墙壁
				{
					// 如果是墙壁，就走不通，不需要放入队列
				}
				else if (map_data[i][j] == 'A') // 是别的机器人
				{
					// 将<i,j>入队
					coordinate cd;
					cd.x = i;
					cd.y = j;
					que.push(cd);
					int r_seq = getRobortSeq(i, j);
					together_robort.push_back(r_seq);
				}
				else // 空地
				{
					// 将<i,j>入队
					coordinate cd;
					cd.x = i;
					cd.y = j;
					que.push(cd);
				}
				// 标记该位置已遍历
				inq[i][j] = true;
			}
		}
	}
	roborts_can_go_bench_map[r_id] = r_can_go_bench_vec;
}

void BFSTrave()
{
	// 遍历四个机器人
	for (int r_id = 0; r_id < 4; r_id++)
	{
		// 如果已经有了该机器人的记录，则跳过
		if (roborts_can_go_bench_map.find(r_id) != roborts_can_go_bench_map.end())
			continue;

		together_robort.clear();
		r_can_go_bench_vec.clear();

		int X, Y;
		getRobortXY(r_id, X, Y);
		BFS(r_id, X, Y);
		// 如果机器人在同一块联通区域
		for (int i = 0; i < together_robort.size(); i++)
		{
			int r_id = together_robort[i];
			if (roborts_can_go_bench_map.find(r_id) == roborts_can_go_bench_map.end()) // 没有这个记录
				roborts_can_go_bench_map[r_id] = r_can_go_bench_vec;
		}
	}
}


int getBenchSeq(int i, int j)
{
	return bench_seq_map[i][j];
}

void getRobortXY(int r_seq, int &X, int &Y) // 通过引用回传
{
	X = robort_seq_map[r_seq][0];
	Y = robort_seq_map[r_seq][1];
}

int getRobortSeq(int i, int j)
{
	for (int r_id = 0; r_id < 4; r_id++)
	{
		int x = robort_seq_map[r_id][0];
		int y = robort_seq_map[r_id][1];
		if (i == x && j == y)
		{
			return r_id;
		}
	}
	return -1;
}

void init_bench_robort_seq_map()
{
	int bench_seq = 0;
	int robort_seq = 0;
	for (int i = 0; i < row1; i++)
	{
		for (int j = 0; j < col1; j++)
		{
			if (map_data[i][j] >= '1'&&map_data[i][j] <= '9')
			{
				bench_seq_map[i][j] = bench_seq;
				bench_seq_dist[bench_seq][0] = i;
				bench_seq_dist[bench_seq][1] = j;
				bench_seq_type_dist[bench_seq] = map_data[i][j] - '0';
				//将工作台按类型归好类别
				infor_working_benchs_dict[int(map_data[i][j]) - int('0')].push_back(bench_seq);
				bench_seq++;
			}
			if (map_data[i][j] == 'A')
			{
				robort_seq_map[robort_seq][0] = i;
					robort_seq_map[robort_seq][1] = j;
					robort_seq++;
			}
		}
	}
	init_bench_sum = bench_seq;
}

//void ReadTxtData(std::string filename, char a[row1][col1]) {
//	//filename为读取文件的地址；
//	//a为一个二维数组，将文本文件里面的内容读取到数组a中
//	std::ifstream readfile(filename);//打开文件夹
//	char* ptr = &a[0][0];
//	while (!readfile.eof())
//	{
//		readfile >> *ptr;//依次将数据读取
//		ptr++;
//	}
//	readfile.close();//关闭文件夹
//
//}

void coordinateTransf(int x0, int y0, double& x1, double& y1)//100*100坐标转换为50*50
{
	x1 = (double)(99 - x0)/2;
	y1 = (double)y0/2;
}

//int main()
//{
//	string filename = "C:\\Users\\14839\\Desktop\\WindowsRelease\\maps\\1.txt";//文件名
//	//char a[row][col] = { };//用于将文本文件中的数据读取到二维数组a中
//	ReadTxtData(filename, map_data);//调用函数
//
//	//初始化
//	roborts_can_go_bench_map.clear();
//	for (int i = 0; i < row1; i++)
//	{
//		for (int j = 0; j < col1; j++)
//		{
//			bench_seq_map[i][j] = -1;
//		}
//	}
//
//	init_bench_robort_seq_map();
//	BFSTrave();
//
//
//	//输出结果
//	for (int r_id = 0; r_id < 4; r_id++)
//	{
//		printf("robort:%d ", r_id);
//		map<int, vector<int>>::iterator it = roborts_can_go_bench_map.find(r_id);
//		vector<int> vec = it->second;
//		printf("size:%d ", vec.size());
//		for (int i = 0; i < vec.size(); i++)
//		{
//			printf("%d ", vec[i]);
//		}
//		printf("%\n");
//
//	}
//
//	
//
//
//}

