/*
 *  相关的数学运算
 */
#ifndef _3D_MATRIX_H_
#define _3D_MATRIX_H_

#include <stdbool.h>

/*
 *  旋转矩阵,绕xyz顺序旋转
 *  参数:
 *      roll_xyz: 绕三轴旋转,单位:度
 *      xyz: 目标点
 *      retXyz: 旋转和平移后结果写到此
 */
void _3d_matrix_roll_calculate(double roll_xyz[3], double xyz[3], double retXyz[3]);

/*
 *  矩阵运算: 透视矩阵点乘三维坐标,然后除以z(透视除法),返回投影坐标[-ar, ar]U[-1, 1]
 * 
 *  参数:
 *      openAngle: 相机开角(单位:度,范围:[1,359])
 *      xyz[3]: 要计算的空间坐标
 *      ar: 相机的屏幕的宽高比
 *      nearZ: 相机近端距离
 *      farZ: 相机远端距离
 *      retXY: 计算结果,一个二维平面坐标(注意其坐标原点是屏幕中心)
 *      retDepth: 计算结果,深度值(远离屏幕的距离,单位:点)
 * 
 *  返回: 0/不再相框内  1/在相框内
 */
bool _3d_matrix_project_calculate(
    double openAngle,
    double xyz[3],
    double ar,
    int nearZ,
    int farZ,
    double *retXY,
    double *retDepth);

#endif
