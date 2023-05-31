#include "globalVariables.h"
/**
 * FunctionName:  calculateAngle(double* startPoint, double* endPoint)
 * Input: 两个点坐标
 * Return: 矢量的弧度(-pi,pi]
 * Description: 计算弧度
 * Author: 余游,2023/3/11
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
 * Input: 二维向量
 * Return:二范数
 * Description: 计算2范数
 * Author: 余游,2023/3/12
 */
double calculateL2(double x, double y) {
	return sqrt(x * x + y * y);
}
/**
 * FunctionName: calculateDist
 * Input:  两个点坐标
 * Return:两个点距离
 * Description: 计算2范数
 * Author: 余游,2023/3/12
 */
double calculateDist(double* startPoint, double* endPoint)
{
	double y = (endPoint[1] - startPoint[1]);
	double x = (endPoint[0] - startPoint[0]);
	return calculateL2(x, y);
}
/**
 * FunctionName:minF
 * Input:  两个浮点数较小值
 * Return:浮点数较小值
 * Description: 浮点数较小值
 * Author: 余游,2023/3/16
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
 * Input:  浮点数
 * Return:浮点数绝对值
 * Description: 浮点数绝对值
 * Author: 余游,2023/3/16
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
 * Input: 整数数组
 * Return:数组和
 * Description: sum数组
 * Author: 余游,2023/3/17
 */
 // int sumIntArray(int a[])
 //{
 //	int sum = 0;
 //	int length = sizeof(a) / sizeof(int);
 //	for (int i = 0;i < length;i++)
 //		sum += a[i];
 //	return sum;
 //}