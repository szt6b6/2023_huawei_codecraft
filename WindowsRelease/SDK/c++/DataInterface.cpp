#define  _CRT_SECURE_NO_WARNINGS
#include"globalVariables.h"
char map_data[100][100];
DeskData workDesk;
RobotData robot;
int frameID; 
int hasMoney;
int deskNum=0;
int robotNum=4;
/**
 * FunctionName: bool readMap()
 * Input: ��
 * Return: ��ȡOK����true
 * Description: ��ȡ��ͼ
 * Author: ����,2023/3/11
 */
bool readMap() {
    char line[1024];
    int j = 0;
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        else
        {
            for (int i = 0; i < 100; i++)
                map_data[j][i] = line[i];
        }
        j++;
    }
    return false;
}
/**
 * FunctionName: bool readMap()
 * Input: ��
 * Return: ��ȡ��OK����true
 * Description: ��ȡ״̬��Ϣд��workDesk��robot�ṹ��
 * Author: ����,2023/3/11
 */
bool readState() {
    int j = 0;
  //��ȡǰ����
    if (j == 0)
    {
        scanf("%d", &hasMoney);
        j++;
    }
     if (j == 1)
    { 
        scanf("%d", &deskNum);
        j++;
    }
 
    //��ȡOK֮ǰ����
    while (j < deskNum + 2+ robotNum) {
        if (j < 2 + deskNum)
        {
            scanf("%d%lf%lf%d%d%d", &workDesk.deskType[j - 2], 
                &workDesk.local[j - 2][0],
                &workDesk.local[j - 2][1]
                , &workDesk.waitTime[j - 2],
                &workDesk.materialState[j - 2], 
                &workDesk.productState[j - 2]);
			if (frameID == 1) {
				//�ڵ�һ֡��ʱ�� ������̨�����͹�����
				infor_working_benchs_dict[workDesk.deskType[j - 2]].push_back(j - 2);
			}
        }
        else {
            scanf("%d%d%lf%lf%lf%lf%lf%lf%lf%lf", &robot.deskID[j - 2 - deskNum]
            , &robot.productType[j - 2 - deskNum]
            , &robot.timeC[j - 2 - deskNum]
            , &robot.collC[j - 2 - deskNum]
            , &robot.angleSpeed[j - 2 - deskNum]
            , &robot.lineSpeed[j - 2 - deskNum][0]
            , &robot.lineSpeed[j - 2 - deskNum][1]
            , &robot.towardAngle[j - 2 - deskNum]
            , &robot.local[j - 2 - deskNum][0]
            , &robot.local[j - 2 - deskNum][1]);
        }
        j++;
    }

	if (frameID == 1) { 
		//���ݳ��Ϲ���̨����������������
		material_demand[4] = infor_working_benchs_dict[7].size() + 1;
		material_demand[5] = infor_working_benchs_dict[7].size() + 1;
		material_demand[6] = infor_working_benchs_dict[7].size() + 1;
		material_demand[1] = infor_working_benchs_dict[4].size() + infor_working_benchs_dict[5].size();
		material_demand[2] = infor_working_benchs_dict[4].size() + infor_working_benchs_dict[6].size();
		material_demand[3] = infor_working_benchs_dict[5].size() + infor_working_benchs_dict[6].size();

		//���칤��̨֮�� ����������ϵ
		for (int i = 0; i < deskNum; i++) {
			int type = workDesk.deskType[i];
			if (type == 4) {
				bench_in_edge_num[i] = infor_working_benchs_dict[1].size() + infor_working_benchs_dict[2].size();
				if (infor_working_benchs_dict[7].size() > 0)
					bench_out_edge_num[i] = infor_working_benchs_dict[7].size();
				else
					bench_out_edge_num[i] = infor_working_benchs_dict[9].size();
			}
			else if (type == 5) {
				bench_in_edge_num[i] = infor_working_benchs_dict[1].size() + infor_working_benchs_dict[3].size();
				if (infor_working_benchs_dict[7].size() > 0)
					bench_out_edge_num[i] = infor_working_benchs_dict[7].size();
				else
					bench_out_edge_num[i] = infor_working_benchs_dict[9].size();
			}
			else if (type == 6) {
				bench_in_edge_num[i] = infor_working_benchs_dict[2].size() + infor_working_benchs_dict[3].size();
				if (infor_working_benchs_dict[7].size() > 0)
					bench_out_edge_num[i] = infor_working_benchs_dict[7].size();
				else
					bench_out_edge_num[i] = infor_working_benchs_dict[9].size();
			}
			else if (type == 7) {
				bench_in_edge_num[i] = infor_working_benchs_dict[4].size() + infor_working_benchs_dict[5].size() + infor_working_benchs_dict[6].size();
				bench_out_edge_num[i] = infor_working_benchs_dict[8].size() + infor_working_benchs_dict[9].size();
			}
			else if (type == 1) {
				bench_out_edge_num[i] = infor_working_benchs_dict[4].size() + infor_working_benchs_dict[5].size();
			}
			else if (type == 2) {
				bench_out_edge_num[i] = infor_working_benchs_dict[4].size() + infor_working_benchs_dict[6].size();
			}
			else {
				bench_out_edge_num[i] = infor_working_benchs_dict[5].size() + infor_working_benchs_dict[6].size();
			}
		}
	}

    while (j >=deskNum + 2+ robotNum) {
      char line[10];
      fgets(line, sizeof line, stdin);
      if (line[0] == 'O' && line[1] == 'K') {
          //workDesk.local[DeskMaxNum-1][0] = 25;
          //workDesk.local[DeskMaxNum-1][1] = 25;
        return true;
        }
       j++;
    }
    return false;
}
/**
 * FunctionName: void outCMD(void);
 * Input: ��
 * Return: ��
 * Description: �������ָ��
 * Author: ����,2023/3/12
 */
void outCMD(void)
{
    for (int i = 0; i < RobotNum; i++)
    {
        //�ٶȿ���
        lastSetSpeed[i] = setLineSpeed[i];
        lastSetAngleSpeed[i] = setAngleSpeed[i];
        printf("forward %d %lf\n", i, setLineSpeed[i]);
        printf("rotate %d %lf\n", i, setAngleSpeed[i]);
        //��������
        ;
    }
    printf("OK\n");
    fflush(stdout);
}