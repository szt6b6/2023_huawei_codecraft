#include "globalVariables.h"
/**
 * FunctionName:  calculateAngle(double* startPoint, double* endPoint)
 * Input: ����������
 * Return: ʸ���Ļ���(-pi,pi]
 * Description: ���㻡��
 * Author: ����,2023/3/11
 */
double calculateAngle(double* startPoint, double* endPoint)
{
	double y = (endPoint[1] - startPoint[1]);
	double x = (endPoint[0] - startPoint[0]);
	double Angle = atan2(y, x);
	return Angle;
}
/**
 * FunctionName:calculateAngle()
 * Input: ��ά����
 * Return:������
 * Description: ����2����
 * Author: ����,2023/3/12
 */
double calculateL2(double x, double y) {
	return sqrt(x * x + y * y);
}
/**
 * FunctionName: calculateDist
 * Input:  ����������
 * Return:���������
 * Description: ����2����
 * Author: ����,2023/3/12
 */
double calculateDist(double* startPoint, double* endPoint)
{
	double y = (endPoint[1] - startPoint[1]);
	double x = (endPoint[0] - startPoint[0]);
	return calculateL2(x, y);
}
/**
 * FunctionName:minF
 * Input:  ������������Сֵ
 * Return:��������Сֵ
 * Description: ��������Сֵ
 * Author: ����,2023/3/16
 */
double minF(double x, double y) {
	if (x >= y)
		return y;
	else
		return x;
}
int minI(int x, int y) {
	if (x >= y)
		return y;
	else
		return x;
}
int maxI(int x, int y) {
	if (x >= y)
		return x;
	else
		return y;
}
/**
 * FunctionName:absF
 * Input:  ������
 * Return:����������ֵ
 * Description: ����������ֵ
 * Author: ����,2023/3/16
 */
double absF(double x)
{
	if (x >= 0)
		return  x;
	else
		return -x;
}
int absI(int x)
{
	if (x >= 0)
		return  x;
	else
		return -x;
}
/**
 * FunctionName:absF
 * Input: ��������
 * Return:�����
 * Description: sum����
 * Author: ����,2023/3/17
 */
 // int sumIntArray(int a[])
 //{
 //	int sum = 0;
 //	int length = sizeof(a) / sizeof(int);
 //	for (int i = 0;i < length;i++)
 //		sum += a[i];
 //	return sum;
 //}