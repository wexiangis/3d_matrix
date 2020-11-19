/*
 *  摄像机定义, 主要是定义可视范围、摄像机位置及旋转角度
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "3d_camera.h"

#define _3D_CAMERA_PI 3.1415926535897
#define _3D_CAMERA_2PI (_3D_CAMERA_PI * 2)

/*
 *  相机初始化
 *  参数:
 *      width, height: 相机屏幕宽高
 *      openAngle: 相机视野开角, 范围(0,PI), 推荐值: PI/2 即 1.57
 *      near, far: 可视范围的近端和远端, 要求大于0且far要大于near, 推荐值: near=5 far=1000
 * 
 *  返回: NULL/参数错误
 */
_3D_Camera *_3d_camera_init(
    uint32_t width,
    uint32_t height,
    double openAngle,
    uint32_t near,
    uint32_t far,
    double *xyz,
    double *roll_xyz)
{
    _3D_Camera *camera;
    //参数检查
    if (openAngle < 0.1 || openAngle > 3.14 || near < 1 || far <= near)
        return NULL;

    camera = (_3D_Camera *)calloc(1, sizeof(_3D_Camera));
    camera->width = width;
    camera->height = height;
    camera->ar = (double)width / height;
    camera->openAngle = openAngle;
    camera->near = near;
    camera->far = far;
    //照片内存
    camera->photoSize = width * height * 3;
    camera->photoMap = (uint8_t *)calloc(camera->photoSize, sizeof(uint8_t));
    //初始状态
    if (xyz)
        memcpy(camera->xyz, xyz, sizeof(double) * 3);
    if (roll_xyz)
        memcpy(camera->roll_xyz, roll_xyz, sizeof(double) * 3);
    //备份
    camera->backup = (_3D_Camera *)calloc(1, sizeof(_3D_Camera));
    memcpy(camera->backup, camera, sizeof(_3D_Camera));

    return camera;
}

// 相机重置
void _3d_camera_reset(_3D_Camera *camera)
{
    memcpy(camera, camera->backup, sizeof(_3D_Camera));
}

// 相机参数备份
void _3d_camera_backup(_3D_Camera *camera)
{
    memcpy(camera->backup, camera, sizeof(_3D_Camera));
}

// 相机拷贝生成新的相机
_3D_Camera *_3d_camera_copy(_3D_Camera *camera)
{
    _3D_Camera *camera2 = (_3D_Camera *)calloc(1, sizeof(_3D_Camera));
    //拷贝参数
    memcpy(camera2, camera, sizeof(_3D_Camera));
    //专有指针重新分配内存
    camera2->photoMap = (uint8_t *)calloc(camera2->photoSize, sizeof(uint8_t));
    memcpy(camera2->photoMap, camera->photoMap, camera2->photoSize);
    //备份
    camera2->backup = (_3D_Camera *)calloc(1, sizeof(_3D_Camera));
    memcpy(camera2->backup, camera2, sizeof(_3D_Camera));
    return camera2;
}

// 内存销毁
void _3d_camera_release(_3D_Camera **camera)
{
    if (camera && (*camera))
    {
        if ((*camera)->photoMap)
            free((*camera)->photoMap);
        if ((*camera)->backup)
            free((*camera)->backup);
        free(*camera);
        *camera = NULL;
    }
}

/* ---------- 运动 ---------- */

// 相机3轴旋转, 增量式, 绕自身坐标系, 单位:rad
void _3d_camera_roll(_3D_Camera *camera, double *roll_xyz)
{
    camera->roll_xyz[0] += roll_xyz[0];
    camera->roll_xyz[1] += roll_xyz[1];
    camera->roll_xyz[2] += roll_xyz[2];
    //范围限制
    if (camera->roll_xyz[0] > _3D_CAMERA_2PI)
        camera->roll_xyz[0] -= _3D_CAMERA_2PI;
    else if (camera->roll_xyz[0] < -_3D_CAMERA_2PI)
        camera->roll_xyz[0] += _3D_CAMERA_2PI;

    if (camera->roll_xyz[1] > _3D_CAMERA_2PI)
        camera->roll_xyz[1] -= _3D_CAMERA_2PI;
    else if (camera->roll_xyz[1] < -_3D_CAMERA_2PI)
        camera->roll_xyz[1] += _3D_CAMERA_2PI;

    if (camera->roll_xyz[2] > _3D_CAMERA_2PI)
        camera->roll_xyz[3] -= _3D_CAMERA_2PI;
    else if (camera->roll_xyz[2] < -_3D_CAMERA_2PI)
        camera->roll_xyz[2] += _3D_CAMERA_2PI;
}

// 相机3轴平移, 增量式, 基于空间坐标系
void _3d_camera_mov(_3D_Camera *camera, double *mov_xyz)
{
    camera->xyz[0] += mov_xyz[0];
    camera->xyz[1] += mov_xyz[1];
    camera->xyz[2] += mov_xyz[2];
}

// 相机3轴平移, 增量式, 基于自身坐标系
void _3d_camera_mov2(_3D_Camera *camera, double *mov_xyz)
{
    ;
}

/* ---------- 特效 ---------- */

// 缩放, zoom为1时原始比例, 大于1放大图像, 小于1缩小图像
void _3d_camera_zoom(_3D_Camera *camera, double zoom)
{
    ;
}

// 锁定目标, 之后 _3d_camera_roll 将变成完全绕目标转动
void _3d_camera_lock(_3D_Camera *camera, double *xyz)
{
    memcpy(camera->lock_xyz, xyz, sizeof(double) * 3);
}

// 解除锁定
void _3d_camera_unlock(_3D_Camera *camera)
{
    memset(camera->lock_xyz, 0, sizeof(double) * 3);
}
