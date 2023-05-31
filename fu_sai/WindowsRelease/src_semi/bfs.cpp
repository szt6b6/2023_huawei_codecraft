#include"globalVariables.h"


int bench_seq_map[row1][col1];//����̨�ͻ����˱�ŵĵ�ͼ
int bench_seq_dist[50][2]; //��һά�ǹ���̨��ţ��ڶ�ά�ǹ���̨����
int robort_seq_map[4][2];//��һά�ǻ����˱�ţ��ڶ�ά�ǻ����˳�ʼ����
std::map<int, std::vector<int>> roborts_can_go_bench_map;
bool roborts_can_go_bench_buy_array[4][50] = { false };
bool roborts_can_go_bench_sell_array[4][50] = { false };
std::vector<int> r_can_go_bench_vec;
std::vector<int> together_robort;
int init_bench_sum = -1;
int bench_seq_type_dist[50] = { 0 };


void BFS(int r_id, int x0,int y0)
{
	// ��ʼ����Ǿ���
	bool inq[row1][col1] = { false };
	// ��ʼ������
	std::queue<coordinate> que;

	coordinate begin;
	begin.x = x0;
	begin.y = y0;
	// ��ʼλ�����
	que.push(begin);
	inq[x0][y0] = true;
	while (!que.empty())
	{
		coordinate tmp = que.front();
		que.pop();
		int x = tmp.x;
		int y = tmp.y;
		// ���������Ե����λ��
		Point next_points[4] = {
			{x + 1, y},
			{x - 1, y},
			{x, y + 1},
			{x, y - 1},
		};
		for (int k = 0; k <= 3; k++)
		{
			int i = next_points[k].x, j = next_points[k].y;
			
			// �����Ƿ�λ��
			if (i < 0 || i >= row1)
				continue;
			if (j < 0 || j >= col1)
				continue;
			// ���û�б�����
			if (!inq[i][j])
			{
				if (map_data[i][j] >= '1' && map_data[i][j] <= '9') // �ǹ���̨
				{
					int bench_seq = getBenchSeq(i, j); // ��������õ�����̨��ŵĺ���

					r_can_go_bench_vec.push_back(bench_seq);
						//��<i,j>���
						coordinate cd;
					cd.x = i;
					cd.y = j;
					que.push(cd);
				}
				else if (map_data[i][j] == '#') // ��ǽ��
				{
					// �����ǽ�ڣ����߲�ͨ������Ҫ�������
				}
				else if (map_data[i][j] == 'A') // �Ǳ�Ļ�����
				{
					// ��<i,j>���
					coordinate cd;
					cd.x = i;
					cd.y = j;
					que.push(cd);
					int r_seq = getRobortSeq(i, j);
					together_robort.push_back(r_seq);
				}
				else // �յ�
				{
					// ��<i,j>���
					coordinate cd;
					cd.x = i;
					cd.y = j;
					que.push(cd);
				}
				// ��Ǹ�λ���ѱ���
				inq[i][j] = true;
			}
		}
	}
	roborts_can_go_bench_map[r_id] = r_can_go_bench_vec;
}

void BFSTrave()
{
	// �����ĸ�������
	for (int r_id = 0; r_id < 4; r_id++)
	{
		// ����Ѿ����˸û����˵ļ�¼��������
		if (roborts_can_go_bench_map.find(r_id) != roborts_can_go_bench_map.end())
			continue;

		together_robort.clear();
		r_can_go_bench_vec.clear();

		int X, Y;
		getRobortXY(r_id, X, Y);
		BFS(r_id, X, Y);
		// �����������ͬһ����ͨ����
		for (int i = 0; i < together_robort.size(); i++)
		{
			int r_id = together_robort[i];
			if (roborts_can_go_bench_map.find(r_id) == roborts_can_go_bench_map.end()) // û�������¼
				roborts_can_go_bench_map[r_id] = r_can_go_bench_vec;
		}
	}
}


int getBenchSeq(int i, int j)
{
	return bench_seq_map[i][j];
}

void getRobortXY(int r_seq, int &X, int &Y) // ͨ�����ûش�
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
				//������̨�����͹�����
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
//	//filenameΪ��ȡ�ļ��ĵ�ַ��
//	//aΪһ����ά���飬���ı��ļ���������ݶ�ȡ������a��
//	std::ifstream readfile(filename);//���ļ���
//	char* ptr = &a[0][0];
//	while (!readfile.eof())
//	{
//		readfile >> *ptr;//���ν����ݶ�ȡ
//		ptr++;
//	}
//	readfile.close();//�ر��ļ���
//
//}

void coordinateTransf(int x0, int y0, double& x1, double& y1)//100*100����ת��Ϊ50*50
{
	x1 = (double)(99 - x0)/2;
	y1 = (double)y0/2;
}

//int main()
//{
//	string filename = "C:\\Users\\14839\\Desktop\\WindowsRelease\\maps\\1.txt";//�ļ���
//	//char a[row][col] = { };//���ڽ��ı��ļ��е����ݶ�ȡ����ά����a��
//	ReadTxtData(filename, map_data);//���ú���
//
//	//��ʼ��
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
//	//������
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

