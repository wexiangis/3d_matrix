/*
 *  相关的数学运算
 * 
 *  address: https://github.com/wexiangis/3d_matrix
 *  address2: https://gitee.com/wexiangis/matrix_3d
 * 
 *  update: 2020.11.24 - wexiangis - 初版
 */
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#define _3D_MATRIX_PI 3.1415926535897

/*
 *  旋转矩阵,绕xyz顺序旋转
 *  参数:
 *      roll_xyz: 绕三轴旋转,单位:度
 *      xyz: 目标点
 *      retXyz: 旋转和平移后结果写到此
 */
void _3d_matrix_roll_calculate(float roll_xyz[3], float xyz[3], float retXyz[3])
{
    float x, y, z;
    float Xrad, Yrad, Zrad;
    //参数检查
    if (roll_xyz == NULL || xyz == NULL || retXyz == NULL)
        return;
    //
    x = xyz[0];
    y = xyz[1];
    z = xyz[2];
    //度转rad
    Xrad = roll_xyz[0] * _3D_MATRIX_PI / 180;
    Yrad = roll_xyz[1] * _3D_MATRIX_PI / 180;
    Zrad = roll_xyz[2] * _3D_MATRIX_PI / 180;

    /*      [scroll X]
    *   1       0       0
    *   0     cosA    -sinA
    *   0     sinA     cosA
    *
    *       [scroll Y]
    *  cosB     0      sinB
    *   0       1       0
    * -sinB     0      cosB
    *
    *       [scroll Z]
    *  cosC   -sinC     0
    *  sinC    cosC     0
    *   0       0       1
    *
    *                                          |x|
    *   result = [scroll X][scroll Y][scroll Z]|y|
    *                                          |z|
    *
    *            |retXyz[0]|
    *          = |retXyz[1]|
    *            |retXyz[2]|
    *
    *   retXyz[*] just like the following ...
    */

    retXyz[0] =
        x * cos(Yrad) * cos(Zrad) -
        y * cos(Yrad) * sin(Zrad) +
        z * sin(Yrad);
    retXyz[1] =
        x * (sin(Xrad) * sin(Yrad) * cos(Zrad) + cos(Xrad) * sin(Zrad)) -
        y * (sin(Xrad) * sin(Yrad) * sin(Zrad) - cos(Xrad) * cos(Zrad)) -
        z * sin(Xrad) * cos(Yrad);
    retXyz[2] =
        -x * (cos(Xrad) * sin(Yrad) * cos(Zrad) - sin(Xrad) * sin(Zrad)) +
        y * (cos(Xrad) * sin(Yrad) * sin(Zrad) + sin(Xrad) * cos(Zrad)) +
        z * cos(Xrad) * cos(Yrad);
}

/*
 *  矩阵运算: 透视矩阵点乘三维坐标,然后除以z(透视除法),返回投影坐标[-ar, ar]U[-1, 1]
 * 
 *  参数:
 *      openAngle: 相机开角(单位:rad,范围:(0,pi))
 *      xyz[3]: 要计算的空间坐标
 *      ar: 相机的屏幕的宽高比
 *      nearZ: 相机近端距离
 *      farZ: 相机远端距离
 *      retXY: 计算结果,一个二维平面坐标(注意其坐标原点是屏幕中心)
 *      retDepth: 计算结果,深度值(远离屏幕的距离,单位:点)
 * 
 *  返回: false/不再相框内  true/在相框内
 */
bool _3d_matrix_project_calculate(
    float openAngle,
    float xyz[3],
    float ar,
    int nearZ,
    int farZ,
    float *retXY,
    float *retDepth)
{
    float hMax, hMin, wMax, wMin;
    float retX, retY, retZ;

    //快速检查
    if (openAngle >= 360 || openAngle < 1)
        return false;
    if (ar <= 0 ||
        xyz == NULL ||
        nearZ >= farZ ||
        xyz[0] < nearZ ||
        xyz[0] > farZ)
        return false;

    //度转rad
    openAngle = openAngle * _3D_MATRIX_PI / 180;

    //屏幕高、宽范围(这里是假设屏幕高为2时的数值)
    hMax = 1;
    hMin = -1;
    wMax = ar;
    wMin = -ar;

    /*                          [project matrix]
    *
    *   1/ar/tan(a/2)           0               0               0   
    *       0               1/tan(a/2)          0               0
    *       0                   0     ((-nZ)-fZ)/(nZ-fZ)  2*fZ*nZ/(nZ-fZ)
    *       0                   0               1               0
    *
    *   ar: camera width/height
    *   a: camera open angle
    *   nZ: camera near Z
    *   fZ: camera far Z
    *
    *                   |x|               |retX|
    *   [project matrix]|y| and then /z = |retY|
    *                   |z|               |retZ|
    *                   |1|               | 1  |
    *
    *   output point request: retX in the range of (-ar, ar)
    *                         retY in the range of (-1, 1)
    *                         retZ in the range of (-1, 1)
    */
    retX = xyz[1] / ar / tan(openAngle / 2) / xyz[0];
    retY = xyz[2] / tan(openAngle / 2) / xyz[0];
    retZ = ((-nearZ) - farZ) / (nearZ - farZ) + 2 * farZ * nearZ / (nearZ - farZ) / xyz[0];

    //返回二维坐标
    if (retXY)
    {
        retXY[0] = retX;
        retXY[1] = retY;
    }
    //深度
    if (retDepth)
        *retDepth = xyz[0] - nearZ;
    //是否在相框范围内
    if (wMax > retX && retX > wMin &&
        hMax > retY && retY > hMin &&
        1 > retZ && retZ > -1)
    {
        return true;
    }
    return false;
}
