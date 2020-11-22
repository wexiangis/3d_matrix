/*
 *  相机定义, 主要是定义可视范围、相机位置及旋转角度
 */
#ifndef _3D_CAMERA_H_
#define _3D_CAMERA_H_

#include <stdint.h>

typedef struct _3DCamera
{
    /*
     *  定义相机可视范围,当空间坐标(x,y,z)在下列数值范围时,可以显示在相机上:
     *      x: near ~ far
     *      y: -width/2 ~ width/2
     *      z: -height/2 ~ height/2
     */
    uint32_t width;   //摄像屏幕宽
    uint32_t height;  //摄像屏幕高
    double ar;        //屏幕宽高比: ar = width / height
    double openAngle; //相机视野开角,单位:度,范围[1,359]
    uint32_t near;    //近端距离(屏幕到相机原点距离,即openAngle所在原点的距离)
    uint32_t far;     //远端距离(远端到相机原点距离）

    double xyz[3];      //相机原点当前所在坐标
    double roll_xyz[3]; //相机绕自身坐标系转角(单位:度)
    double lock_xyz[3]; //锁定目标点(就是让相机的旋转以此为原点)

    uint32_t photoSize; //照片内存大小width*height*3
    uint8_t *photoMap;  //照片缓冲区,RGB存储格式,大小 width*height*3

    struct _3DCamera *backup; //对初始化时的参数进行备份(注意其中的 photoMap 不要重复释放)

} _3D_Camera;

/* ---------- 构造和销毁 ---------- */

/*
 *  相机初始化
 *  参数:
 *      width, height: 相机屏幕宽高
 *      openAngle: 相机视野开角, 范围[1,359], 推荐值: 90
 *      near, far: 可视范围的近端和远端, 要求大于0且far要大于near, 推荐值: near=5 far=1000
 *      xyz: 初始位置,不用则置NULL
 *      roll_xyz: 初始角度,不用则置NULL
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
    double *roll_xyz);

// 相机重置
void _3d_camera_reset(_3D_Camera *camera);

// 清空照片
void _3d_camera_photo_clear(_3D_Camera *camera, uint32_t rgbColor);

// 相机参数备份
void _3d_camera_backup(_3D_Camera *camera);

// 相机拷贝生成新的相机
_3D_Camera *_3d_camera_copy(_3D_Camera *camera);

// 内存销毁
void _3d_camera_release(_3D_Camera **camera);

/* ---------- 运动 ---------- */

// 相机3轴旋转, 增量式, 绕自身坐标系, 单位:度
void _3d_camera_roll(_3D_Camera *camera, double *roll_xyz);

// 相机3轴平移, 增量式, 基于空间坐标系
void _3d_camera_mov(_3D_Camera *camera, double *mov_xyz);

// 相机3轴平移, 增量式, 基于自身坐标系
void _3d_camera_mov2(_3D_Camera *camera, double *mov_xyz);

/* ---------- 特效 ---------- */

// 缩放, zoom为1时原始比例, 大于1放大图像, 小于1缩小图像
void _3d_camera_zoom(_3D_Camera *camera, double zoom);

// 锁定目标, 之后 _3d_camera_roll 将变成完全绕目标转动
void _3d_camera_lock(_3D_Camera *camera, double *xyz);

// 解除锁定
void _3d_camera_unlock(_3D_Camera *camera);

#endif
