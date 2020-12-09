/*
 *  为3D模型和摄像机提供场地,并在此基础上展开运动计算
 * 
 *  address: https://github.com/wexiangis/3d_matrix
 *  address2: https://gitee.com/wexiangis/matrix_3d
 * 
 *  update: 2020.11.24 - wexiangis - 初版
 */
#ifndef _3D_ANGINE_H_
#define _3D_ANGINE_H_

#include <pthread.h>

// 基于相机、模型和算法
#include "3d_camera.h"
#include "3d_model.h"
#include "3d_math.h"

// 单元的运动控制状态
typedef struct _3DSport
{
    float xyz[3];         //质心在空间中的位置
    float roll_xyz[3];    //绕自身坐标系转角(单位:度)

    float speed[3];       //速度向量,相对空间坐标系,单位:点/秒
    float speed_angle[3]; //角速度向量,相对自身坐标系,单位:度/秒
    
    float quat[4];        //使用四元数微分方程来更新 roll_xyz[]
    // struct _3DSport *next;   //用链表来记录历史状态
} _3D_Sport;

// 空间中的物体单元
typedef struct _3DUnit
{
    _3D_Sport *sport; //运动状态
    _3D_Model *model; //模型(该参数在这里是只读的,所以可以把一个模型赋值给多个单元)
    struct _3DUnit *next;
} _3D_Unit;

// 主结构体
typedef struct _3DEngine
{
    _3D_Unit *unit;       //单元链表
    uint32_t intervalMs;  //刷新/计算间隔,单位:ms
    float xyzSize[3];     //xyz空间范围
    float xyzRange[3][2]; //空间范围 [x][0]/min [x][1]/max
    pthread_t th;
    pthread_mutex_t lock;
    bool run;        //开/停标志
    bool threadExit; //线程回收标志
} _3D_Engine;

/*
 *  引擎初始化
 *  参数:
 *      intervalMs: 刷新间隔,单位:ms
 *      xSize, ySize, zSize: 空间场地大小,其中点(xSize/2, ySize/2, zSize/2)的位置将作为空间原点
 */
_3D_Engine *engine_init(uint32_t intervalMs, float xSize, float ySize, float zSize);

/*
 *  添加模型
 *  参数:
 *      xyz: 在空间中的初始位置(可以置NULL)
 *      roll_xyz: 绕自身坐标系旋转角度,即朝向,欧拉角,单位:度(可以置NULL)
 * 
 *  返回: 模型运动控制器,可进行异步控制、获取模型的运动状态
 *  其它: 当 xyz = NULL, roll_xyz = NULL 模型在空间默认位置为原点,面朝x轴正方向,头顶z轴正方向,左边y轴正方向
 */
_3D_Sport *engine_model_add(_3D_Engine *engine, _3D_Model *model, float *xyz, float *roll_xyz);

// 模型移除,成功返回true (注意 sport 指针成功移除等于被释放,不能再使用)
bool engine_model_remove(_3D_Engine *engine, _3D_Sport *sport);

// 相机抓拍,照片缓存在 camera->photoMap
void engine_photo(_3D_Engine *engine, _3D_Camera *camera);

// 开始
void engine_start(_3D_Engine *engine);

// 暂停
void engine_pause(_3D_Engine *engine);

// 内存销毁(注意其中用到的 model 和 camera 需自行销毁)
void engine_release(_3D_Engine **engine);

#endif
