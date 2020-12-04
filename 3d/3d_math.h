/*
 *  相关的数学运算
 * 
 *  address: https://github.com/wexiangis/3d_matrix
 *  address2: https://gitee.com/wexiangis/matrix_3d
 * 
 *  update: 2020.11.24 - wexiangis - 初版
 */
#ifndef _3D_MATH_H_
#define _3D_MATH_H_

#include <stdbool.h>

/*
 *  四元数方式旋转和逆旋转
 *  参数:
 *      quat[4]: 使用已有的四元数(可置NULL), 将不使用 roll_vector 和 roll_deg
 *      roll_vector[3]: 要绕转的空间向量,右手旋转,大拇指向量方向
 *      roll_deg: 旋转角度,单位:度
 *      vector[3]: 被旋转的向量,输出结果覆写到此
 *      T: 转置
 */
void quat_roll(float quat[4], float roll_vector[3], float roll_deg, float vector[3], bool T);

/*
 *  四元数依次三轴旋转
 *  参数:
 *      roll_xyz: 绕三轴旋转,单位:度
 *      xyz: 目标点,旋转结果覆写到此
 */
void quat_xyz(float roll_xyz[3], float xyz[3]);
void quat_zyx(float roll_xyz[3], float xyz[3]);

/*
 *  使用现有四元数进行旋转矩阵运算
 */
void quat_matrix_xyz(float quat[4], float xyz[3]);
void quat_matrix_zyx(float quat[4], float xyz[3]);

/*
 *  旋转矩阵(matrix_xyz 和 matrix_zyx 互为转置矩阵,互为逆向旋转)
 *  参数:
 *      roll_xyz: 绕三轴旋转,单位:度
 *      xyz: 目标点
 *      retXyz: 旋转和平移后结果写到此
 */
void matrix_xyz(float roll_xyz[3], float xyz[3], float retXyz[3]);
void matrix_zyx(float roll_xyz[3], float xyz[3], float retXyz[3]);

/*
 *  透视矩阵点乘三维坐标,然后除以z(透视除法),返回投影坐标[-ar, ar]U[-1, 1]
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
bool projection(
    float openAngle,
    float xyz[3],
    float ar,
    int nearZ,
    int farZ,
    float *retXY,
    float *retDepth);

#endif
