/*
 *  摄像机定义, 主要是定义可视范围、摄像机位置及旋转角度
 * 
 *  address: https://github.com/wexiangis/3d_matrix
 *  address2: https://gitee.com/wexiangis/matrix_3d
 * 
 *  update: 2020.11.24 - wexiangis - 初版
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "3d_camera.h"
#include "3d_math.h"

#ifndef M_PI //理论上在 math.h 中有定义
#define M_PI 3.14159265358979323846
#endif

/*
 *  相机初始化
 *  参数:
 *      width, height: 相机屏幕宽高
 *      openAngle: 相机视野开角, 范围[1,179], 推荐值: 90
 *      near, far: 可视范围的近端和远端, 要求大于0且far要大于near, 推荐值: near=5 far=1000
 * 
 *  返回: NULL/参数错误
 *  其它: 当 xyz = NULL, roll_xyz = NULL 相机在空间默认位置为原点,面朝x轴正方向,头顶z轴正方向,左边y轴正方向
 */
_3D_Camera *camera_init(
    uint32_t width,
    uint32_t height,
    float openAngle,
    uint32_t near,
    uint32_t far,
    float *xyz,
    float *roll_xyz)
{
    _3D_Camera *camera;

    //参数检查
    if (openAngle > 179 || openAngle < 1 || near < 1 || far <= near)
        return NULL;

    camera = (_3D_Camera *)calloc(1, sizeof(_3D_Camera));
    camera->width = width;
    camera->height = height;
    camera->ar = (float)width / height;
    camera->openAngle = openAngle;
    camera->near = near;
    camera->far = far;
    camera->quat[0] = 1;

    //照片内存
    camera->photoSize = width * height * 3;
    camera->photoMap = (uint8_t *)calloc(camera->photoSize, sizeof(uint8_t));
    camera->photoDepth = (float *)calloc(width * height, sizeof(float));

    //初始状态
    if (xyz)
        memcpy(camera->xyz, xyz, sizeof(float) * 3);
    if (roll_xyz)
        pry_to_quat2(roll_xyz, camera->quat);

    //备份
    camera->backup = (_3D_Camera *)calloc(1, sizeof(_3D_Camera));
    memcpy(camera->backup, camera, sizeof(_3D_Camera));

    return camera;
}

// 相机重置
void camera_reset(_3D_Camera *camera)
{
    memcpy(camera, camera->backup, sizeof(_3D_Camera));
}

// 清空照片
void camera_photo_clear(_3D_Camera *camera, uint32_t rgbColor)
{
    uint32_t mapCount, depthCount;
    uint8_t r = (rgbColor >> 16) & 0xFF;
    uint8_t g = (rgbColor >> 8) & 0xFF;
    uint8_t b = (rgbColor >> 0) & 0xFF;
    for (mapCount = depthCount = 0; mapCount < camera->photoSize;)
    {
        camera->photoMap[mapCount++] = r;
        camera->photoMap[mapCount++] = g;
        camera->photoMap[mapCount++] = b;
        camera->photoDepth[depthCount++] = 0;
    }
}

// 相机参数备份
void camera_backup(_3D_Camera *camera)
{
    memcpy(camera->backup, camera, sizeof(_3D_Camera));
}

// 相机拷贝生成新的相机
_3D_Camera *camera_copy(_3D_Camera *camera)
{
    _3D_Camera *camera2 = (_3D_Camera *)calloc(1, sizeof(_3D_Camera));
    //拷贝参数
    memcpy(camera2, camera, sizeof(_3D_Camera));
    //专有指针重新分配内存
    camera2->photoMap = (uint8_t *)calloc(camera2->photoSize, sizeof(uint8_t));
    memcpy(camera2->photoMap, camera->photoMap, camera2->photoSize);
    camera2->photoDepth = (float *)calloc(camera2->width * camera2->height, sizeof(float));
    //备份
    camera2->backup = (_3D_Camera *)calloc(1, sizeof(_3D_Camera));
    memcpy(camera2->backup, camera2, sizeof(_3D_Camera));
    return camera2;
}

// 内存销毁
void camera_release(_3D_Camera **camera)
{
    if (camera && (*camera))
    {
        if ((*camera)->photoMap)
            free((*camera)->photoMap);
        if ((*camera)->photoDepth)
            free((*camera)->photoDepth);
        if ((*camera)->backup)
            free((*camera)->backup);
        free(*camera);
        *camera = NULL;
    }
}

/* ---------- 运动 ---------- */

// 相机3轴旋转, 增量式, 绕空间坐标系, 单位:度
void camera_roll(_3D_Camera *camera, float x, float y, float z)
{
    float rxyz[] = {x, y, z};
    //
    quat_roll(camera->quat, NULL, 0, rxyz, true);
    //
    camera_roll2(camera, rxyz[1], rxyz[2], rxyz[0]);
}

// 相机3轴旋转, 增量式, 绕自身坐标系, 单位:度
void camera_roll2(_3D_Camera *camera, float rUpDown, float rLeftRight, float rClock)
{
    float roll_xyz[] = {rClock, rUpDown, rLeftRight};
    quat_diff2(camera->quat, roll_xyz);
}

// 相机3轴平移, 增量式, 基于空间坐标系
void camera_mov(_3D_Camera *camera, float x, float y, float z)
{
    camera->xyz[0] += x;
    camera->xyz[1] += y;
    camera->xyz[2] += z;
}

// 相机3轴平移, 增量式, 基于自身坐标系
void camera_mov2(_3D_Camera *camera, float upDown, float leftRight, float frontBack)
{
    //组成增量向量
    float mXYZ[] = {frontBack, leftRight, upDown};
    //
    quat_roll(camera->quat, NULL, 0, mXYZ, false);
    //再平移
    camera_mov(camera, mXYZ[0], mXYZ[1], mXYZ[2]);
}

/* ---------- 特效 ---------- */

// 缩放, zoom为1时原始比例, 大于1放大图像, 小于1缩小图像
void camera_zoom(_3D_Camera *camera, float zoom)
{
    ;
}

// 锁定目标, 之后 camera_roll 将变成完全绕目标转动
void camera_lock(_3D_Camera *camera, float xyz[3])
{
    memcpy(camera->lock_xyz, xyz, sizeof(float) * 3);
}

// 解除锁定
void camera_unlock(_3D_Camera *camera)
{
    memset(camera->lock_xyz, 0, sizeof(float) * 3);
}

/* ---------- 其它 ---------- */

static float _fabs(float v)
{
    return v > 0 ? v : (-v);
}

//空间坐标(相机坐标系)是否在相机可视范围内
bool camera_isInside(_3D_Camera *camera, float xyz[3])
{
    //远近
    if (xyz[0] < camera->near || xyz[0] > camera->far)
        return false;
    //上下
    if (_fabs(tan(camera->openAngle / 2 * M_PI / 180)) > _fabs(xyz[2] / xyz[0]))
        return false;
    //左右
    if (_fabs(tan(camera->openAngle / 2 * M_PI / 180)) > _fabs(xyz[1] / xyz[0]))
        return false;

    return true;
}
