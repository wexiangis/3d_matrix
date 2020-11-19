/*
 *  为3D模型和摄像机提供场地,并在此基础上展开运动计算
 */
#ifndef _3D_ANGINE_H_
#define _3D_ANGINE_H_

// 基于相机、模型和算法
#include "3d_camera.h"
#include "3d_model.h"
#include "3d_matrix.h"

// 单元的运动状态
typedef struct _3DSport
{
    double xyz[3];           //质心在空间中的位置
    double roll_xyz[3];      //绕空间坐标系原点旋转角度,单位:rad
    double roll_xyz_self[3]; //绕自身坐标系旋转角度,单位:rad
    double speed[3];         //速度向量,单位:点/秒
    // struct _3DSport *next;   //用链表来记录历史状态
} _3D_Sport;

// 空间中的物体单元
typedef struct _3DUnit
{
    _3D_Model *model; //模型(该参数在这里是只读的,所以可以把一个模型赋值给多个单元)
    _3D_Sport *sport; //运动状态
    struct _3DUnit *next;
} _3D_Unit;

// 主结构体
typedef struct _3DEngine
{
    _3D_Unit unit;       //单元链表
    uint32_t intervalMs; //刷新/计算间隔
    uint32_t xMax, xMin; //空间范围
    uint32_t yMax, yMin;
    uint32_t zMax, zMin;
} _3D_Engine;

/* ---------- 构造和销毁 ---------- */

/*
 *  引擎初始化
 *  参数:
 *      intervalMs: 刷新间隔,单位:ms
 *      xSize, ySize, zSize: 空间场地大小, 其中点(xSize/2, ySize/2, zSize/2)的位置将作为空间原点
 */
_3D_Engine *_3d_engine_init(uint32_t intervalMs, uint32_t xSize, uint32_t ySize, uint32_t zSize);

/*
 *  添加模型
 *  参数:
 *      xyz: 在空间中的初始位置
 *      roll_xyz_self: 绕自身坐标系旋转角度,即朝向
 * 
 *  返回: 模型序号,用于移除时使用
 */
uint32_t _3d_engine_model_add(_3D_Model *model, double *xyz, double *roll_xyz_self);

/*
 *  模型移除
 */
void _3d_engine_model_remove(uint32_t order);

/*
 *  相机抓拍,照片缓存在 camera->photoMap
 */
void _3d_engine_photo(_3D_Camera *camera);

/*
 *  相机移除
 */
void _3d_engine_camera_remove(uint32_t order);

// 内存销毁(注意其中用到的 model 和 camera 需自行销毁)
void _3d_engine_release(_3D_Engine **engine);

#endif
