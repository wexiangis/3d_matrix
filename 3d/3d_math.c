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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define _3D_MATH_PI 3.1415926535897

// 四元数转欧拉角
// static void _quat_to_pry(float q[4], float pry[3])
// {
//     pry[0] = asin(-2 * q[1] * q[3] + 2 * q[0] * q[2]);
//     pry[1] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1);
//     pry[2] = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
// }

// 四元数乘法
static void _quat_multiply(float q1[4], float q2[4], float ret[4])
{
    float _ret[4];
    _ret[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    _ret[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    _ret[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    _ret[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
    ret[0] = _ret[0];
    ret[1] = _ret[1];
    ret[2] = _ret[2];
    ret[3] = _ret[3];
}

/*
 *  四元数方式旋转和逆旋转
 *  参数:
 *      quat[4]: 使用已有的四元数(可置NULL), 将不使用 roll_vector 和 roll_deg
 *      roll_vector[3]: 要绕转的空间向量,右手旋转,大拇指向量方向
 *      roll_deg: 旋转角度,单位:度
 *      vector[3]: 被旋转的向量,输出结果覆写到此
 *      T: 转置
 */
void quat_roll(float quat[4], float roll_vector[3], float roll_deg, float vector[3], bool T)
{
    float *q = quat;
    float _q[4], qT[4];
    float v[4], ret[4];
    float rad = roll_deg * _3D_MATH_PI / 180;
    // float norm;

    if (!q)
    {
        q = _q;
        q[0] = cos(rad / 2);
        q[1] = sin(rad / 2) * roll_vector[0];
        q[2] = sin(rad / 2) * roll_vector[1];
        q[3] = sin(rad / 2) * roll_vector[2];
    }

    qT[0] = q[0];
    qT[1] = -q[1];
    qT[2] = -q[2];
    qT[3] = -q[3];

    v[0] = 0;
    v[1] = vector[0];
    v[2] = vector[1];
    v[3] = vector[2];

    if (T)
    {
        _quat_multiply(qT, v, ret);
        _quat_multiply(ret, q, ret);
    }
    else
    {
        _quat_multiply(q, v, ret);
        _quat_multiply(ret, qT, ret);
    }

    // norm = sqrt(ret[1] * ret[1] + ret[2] * ret[2] + ret[3] * ret[3]);
    // if (!isnan(norm))
    // {
    //     ret[1] /= norm;
    //     ret[2] /= norm;
    //     ret[3] /= norm;

    //     norm = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    //     if (!isnan(norm))
    //     {
    //         ret[1] *= norm;
    //         ret[2] *= norm;
    //         ret[3] *= norm;
    //     }
    // }

    memcpy(vector, &ret[1], sizeof(float) * 3);
}

static void _quat_roll_xyz(float roll_xyz[3], float vector[3], bool zyx)
{
    float qx[4] = {0}, qy[4] = {0}, qz[4] = {0};
    float qxT[4] = {0}, qyT[4] = {0}, qzT[4] = {0};
    float v[4], ret[4];

    qx[0] = qxT[0] = cos(roll_xyz[0] / 2 * _3D_MATH_PI / 180);
    qx[1] = sin(roll_xyz[0] / 2 * _3D_MATH_PI / 180);
    qxT[1] = -qx[1];

    qy[0] = qyT[0] = cos(roll_xyz[1] / 2 * _3D_MATH_PI / 180);
    qy[2] = sin(roll_xyz[1] / 2 * _3D_MATH_PI / 180);
    qyT[2] = -qy[2];

    qz[0] = qzT[0] = cos(roll_xyz[2] / 2 * _3D_MATH_PI / 180);
    qz[3] = sin(roll_xyz[2] / 2 * _3D_MATH_PI / 180);
    qzT[3] = -qz[3];

    v[0] = 0;
    v[1] = vector[0];
    v[2] = vector[1];
    v[3] = vector[2];

    if (zyx)
    {
        _quat_multiply(qz, qy, ret);
        _quat_multiply(ret, qx, ret);
        _quat_multiply(ret, v, ret);
        _quat_multiply(ret, qxT, ret);
        _quat_multiply(ret, qyT, ret);
        _quat_multiply(ret, qzT, ret);

        // _quat_multiply(qxT, qyT, ret);
        // _quat_multiply(ret, qzT, ret);
        // _quat_multiply(ret, v, ret);
        // _quat_multiply(ret, qz, ret);
        // _quat_multiply(ret, qy, ret);
        // _quat_multiply(ret, qx, ret);
    }
    else
    {
        _quat_multiply(qx, qy, ret);
        _quat_multiply(ret, qz, ret);
        _quat_multiply(ret, v, ret);
        _quat_multiply(ret, qzT, ret);
        _quat_multiply(ret, qyT, ret);
        _quat_multiply(ret, qxT, ret);
    }

    memcpy(vector, &ret[1], sizeof(float) * 3);
}

/*
 *  四元数依次三轴旋转
 *  参数:
 *      roll_xyz: 绕三轴旋转,单位:度
 *      xyz: 目标点,旋转结果覆写到此
 */
void quat_xyz(float roll_xyz[3], float xyz[3])
{
    _quat_roll_xyz(roll_xyz, xyz, false);
}
void quat_zyx(float roll_xyz[3], float xyz[3])
{
    _quat_roll_xyz(roll_xyz, xyz, true);
}

/*
 *  使用现有四元数进行旋转矩阵运算
 */
void quat_matrix_xyz(float quat[4], float xyz[3])
{
    float q0 = quat[0];
    float q1 = quat[1];
    float q2 = quat[2];
    float q3 = quat[3];
    float ret[3];
    // float norm;

    ret[0] =
        xyz[0] * (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) +
        xyz[1] * 2 * (q1 * q2 - q0 * q3) +
        xyz[2] * 2 * (q1 * q3 + q0 * q2);
    ret[1] =
        xyz[0] * 2 * (q1 * q2 + q0 * q3) +
        xyz[1] * (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) +
        xyz[2] * 2 * (q2 * q3 + q0 * q1);
    ret[2] =
        xyz[0] * 2 * (q1 * q3 - q0 * q2) +
        xyz[1] * 2 * (q1 * q2 + q0 * q3) +
        xyz[2] * (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    
    // norm = sqrt(ret[0] * ret[0] + ret[1] * ret[1] + ret[2] * ret[2]);
    // if (!isnan(norm))
    // {
    //     ret[0] /= norm;
    //     ret[1] /= norm;
    //     ret[2] /= norm;

    //     norm = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);
    //     if (!isnan(norm))
    //     {
    //         ret[0] *= norm;
    //         ret[1] *= norm;
    //         ret[2] *= norm;
    //     }
    // }

    memcpy(xyz, ret, sizeof(float) * 3);
}
void quat_matrix_zyx(float quat[4], float xyz[3])
{
    float q0 = quat[0];
    float q1 = quat[1];
    float q2 = quat[2];
    float q3 = quat[3];
    float ret[3];
    // float norm;

    ret[0] =
        xyz[0] * (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) +
        xyz[1] * 2 * (q1 * q2 + q0 * q3) +
        xyz[2] * 2 * (q1 * q3 - q0 * q2);
    ret[1] =
        xyz[0] * 2 * (q1 * q2 - q0 * q3) +
        xyz[1] * (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) +
        xyz[2] * 2 * (q2 * q3 + q0 * q1);
    ret[2] =
        xyz[0] * 2 * (q1 * q3 + q0 * q2) +
        xyz[1] * 2 * (q2 * q3 + q0 * q1) +
        xyz[2] * (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    
    // norm = sqrt(ret[0] * ret[0] + ret[1] * ret[1] + ret[2] * ret[2]);
    // if (!isnan(norm))
    // {
    //     ret[0] /= norm;
    //     ret[1] /= norm;
    //     ret[2] /= norm;

    //     norm = sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);
    //     if (!isnan(norm))
    //     {
    //         ret[0] *= norm;
    //         ret[1] *= norm;
    //         ret[2] *= norm;
    //     }
    // }

    memcpy(xyz, ret, sizeof(float) * 3);
}

/*
 *  旋转矩阵(matrix_xyz 和 matrix_zyx 互为转置矩阵,互为逆向旋转)
 *  参数:
 *      roll_xyz: 绕三轴旋转,单位:度
 *      xyz: 目标点
 *      retXyz: 旋转和平移后结果写到此
 */
void matrix_xyz(float roll_xyz[3], float xyz[3], float retXyz[3])
{
    float x, y, z;
    float A, B, C;
    //参数检查
    if (roll_xyz == NULL || xyz == NULL || retXyz == NULL)
        return;
    //
    x = xyz[0];
    y = xyz[1];
    z = xyz[2];
    //度转rad
    A = roll_xyz[0] * _3D_MATH_PI / 180;
    B = roll_xyz[1] * _3D_MATH_PI / 180;
    C = roll_xyz[2] * _3D_MATH_PI / 180;

    /*
    *       [roll X]
    *   1       0       0
    *   0     cosA    -sinA
    *   0     sinA     cosA
    *
    *       [roll Y]
    *  cosB     0      sinB
    *   0       1       0
    * -sinB     0      cosB
    *
    *       [roll Z]
    *  cosC   -sinC     0
    *  sinC    cosC     0
    *   0       0       1
    *
    *                                   |x|
    *  result = [roll X][roll Y][roll Z]|y|
    *                                   |z|
    *
    *           |cB,     0,  sB    |        |x|
    *         = |sB*sA,  cA, -cB*sA|[roll Z]|y|
    *           |-sB*cA, sA, cB*cA |        |z|
    * 
    *           |cC*cB,             -sC*cB,            sB    ||x|
    *         = |cC*sB*sA + sC*cA,  -sC*sB*sA + cC*cA, -cB*sA||x|
    *           |-cC*sB*cA + sC*sA, sC*sB*cA + cC*sA,  cB*cA ||z|
    *
    *           |point[0]|
    *         = |point[1]|
    *           |point[2]|
    *
    *  point[*] is equal to the follow ...
    */
    retXyz[0] =
        x * cos(C) * cos(B) +
        y * (-sin(C) * cos(B)) +
        z * sin(B);
    retXyz[1] =
        x * (cos(C) * sin(B) * sin(A) + sin(C) * cos(A)) +
        y * (-sin(C) * sin(B) * sin(A) + cos(C) * cos(A)) +
        z * (-cos(B) * sin(A));
    retXyz[2] =
        x * (-cos(C) * sin(B) * cos(A) + sin(C) * sin(A)) +
        y * (sin(C) * sin(B) * cos(A) + cos(C) * sin(A)) +
        z * cos(B) * cos(A);
}
void matrix_zyx(float roll_xyz[3], float xyz[3], float retXyz[3])
{
    float x, y, z;
    float A, B, C;
    //参数检查
    if (roll_xyz == NULL || xyz == NULL || retXyz == NULL)
        return;
    //
    x = xyz[0];
    y = xyz[1];
    z = xyz[2];
    //度转rad
    A = roll_xyz[0] * _3D_MATH_PI / 180;
    B = roll_xyz[1] * _3D_MATH_PI / 180;
    C = roll_xyz[2] * _3D_MATH_PI / 180;

    /*
    *       [roll Z]
    *  cosC    sinC     0
    * -sinC    cosC     0
    *   0       0       1
    *
    *       [roll Y]
    *  cosB     0     -sinB
    *   0       1       0
    *  sinB     0      cosB
    * 
    *       [roll X]
    *   1       0       0
    *   0     cosA     sinA
    *   0    -sinA     cosA
    *
    *                                   |x|
    *  result = [roll Z][roll Y][roll X]|y|
    *                                   |z|
    *
    *           |cB*cC,  sC, -sB*cC|        |x|
    *         = |-cB*sC, cC, sB*sC |[roll X]|y|
    *           |sB,     0,  cB    |        |z|
    * 
    *           |cB*cC,  cA*sC + sA*sB*cC, sA*sC - cA*sB*cC||x|
    *         = |-cB*sC, cA*cC - sA*sB*sC, sA*cC + cA*sB*sC||x|
    *           |sB,     -sA*cB,           cA*cB           ||z|
    *
    *           |point[0]|
    *         = |point[1]|
    *           |point[2]|
    *
    *  point[*] is equal to the follow ...
    */
    retXyz[0] =
        x * cos(B) * cos(C) +
        y * (cos(A) * sin(C) + sin(A) * sin(B) * cos(C)) +
        z * (sin(A) * sin(C) - cos(A) * sin(B) * cos(C));
    retXyz[1] =
        x * (-cos(B) * sin(C)) +
        y * (cos(A) * cos(C) - sin(A) * sin(B) * sin(C)) +
        z * (sin(A) * cos(C) + cos(A) * sin(B) * sin(C));
    retXyz[2] =
        x * sin(B) +
        y * (-sin(A) * cos(B)) +
        z * cos(A) * cos(B);
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
bool projection(
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
    openAngle = openAngle * _3D_MATH_PI / 180;

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
