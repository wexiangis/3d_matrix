/*
 *  为3D模型和摄像机提供场地,并在此基础上展开运动计算
 * 
 *  address: https://github.com/wexiangis/3d_matrix
 *  address2: https://gitee.com/wexiangis/matrix_3d
 * 
 *  update: 2020.11.24 - wexiangis - 初版
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "3d_engine.h"
#include "2d_draw.h"

// 延时
#include <sys/time.h>
void engine_delayus(unsigned int us)
{
    struct timeval tv;
    tv.tv_sec = us / 1000000;
    tv.tv_usec = us % 1000000;
    select(0, NULL, NULL, NULL, &tv);
}
long engine_getTickUs(void)
{
    struct timeval tv = {0};
    gettimeofday(&tv, NULL);
    return (long)(tv.tv_sec * 1000000u + tv.tv_usec);
}
// 自校准延时
#define ENGINE_DELAY_INIT \
    long _tick1 = 0, _tick2 = 0;
#define ENGINE_DELAY_US(us)                      \
    _tick2 = engine_getTickUs();                 \
    if (_tick2 > _tick1 && _tick2 - _tick1 < us) \
        engine_delayus(us - (_tick2 - _tick1));  \
    _tick1 = engine_getTickUs();

//根据sport运动状态,更新位置和旋转角度(朝向)
static void engine_sport(_3D_Engine *engine, _3D_Sport *sport)
{
    uint32_t count;
    float _speed_angle[3];
    //平移
    for (count = 0; count < 3; count++)
    {
        sport->xyz[count] += sport->speed[count] * engine->intervalMs / 1000;
        //范围限制(循环进出)
        if (sport->xyz[count] > engine->xyzRange[count][1])
            sport->xyz[count] -= engine->xyzSize[count];
        else if (sport->xyz[count] < engine->xyzRange[count][0])
            sport->xyz[count] += engine->xyzSize[count];
    }
    //旋转
    // for (count = 0; count < 3; count++)
    // {
    //     sport->roll_xyz[count] += sport->speed_angle[count] * engine->intervalMs / 1000;
    //     //范围限制
    //     if (sport->roll_xyz[count] > 360)
    //         sport->roll_xyz[count] -= 360;
    //     else if (sport->roll_xyz[count] < -360)
    //         sport->roll_xyz[count] += 360;
    // }
    _speed_angle[0] = sport->speed_angle[0] * engine->intervalMs / 1000;
    _speed_angle[1] = sport->speed_angle[1] * engine->intervalMs / 1000;
    _speed_angle[2] = sport->speed_angle[2] * engine->intervalMs / 1000;
    quat_diff2(sport->quat, _speed_angle);
    quat_to_pry2(sport->quat, sport->roll_xyz);
}

// 主线程
static void engine_thread(void *argv)
{
    _3D_Engine *engine = (_3D_Engine *)argv;
    _3D_Unit *unit;
    ENGINE_DELAY_INIT;
    while (!engine->threadExit)
    {
        ENGINE_DELAY_US(engine->intervalMs * 1000);
        //暂停状态
        if (!engine->run)
            continue;
        //遍历模型链表
        pthread_mutex_lock(&engine->lock);
        unit = engine->unit;
        while (unit)
        {
            //更新运动状态
            engine_sport(engine, unit->sport);
            //下一个
            unit = unit->next;
        }
        pthread_mutex_unlock(&engine->lock);
    }
}

/*
 *  引擎初始化
 *  参数:
 *      intervalMs: 刷新间隔,单位:ms
 *      xSize, ySize, zSize: 空间场地大小,其中点(xSize/2, ySize/2, zSize/2)的位置将作为空间原点
 */
_3D_Engine *engine_init(uint32_t intervalMs, float xSize, float ySize, float zSize)
{
    _3D_Engine *engine;
    //参数检查
    if (xSize < 2 || ySize < 2 || zSize < 2)
        return NULL;
    engine = (_3D_Engine *)calloc(1, sizeof(_3D_Engine));
    engine->intervalMs = intervalMs;
    engine->xyzSize[0] = xSize;
    engine->xyzSize[1] = ySize;
    engine->xyzSize[2] = zSize;
    engine->xyzRange[0][0] = -(xSize / 2);
    engine->xyzRange[0][1] = xSize / 2;
    engine->xyzRange[1][0] = -(ySize / 2);
    engine->xyzRange[1][1] = ySize / 2;
    engine->xyzRange[2][0] = -(zSize / 2);
    engine->xyzRange[2][1] = zSize / 2;
    pthread_mutex_init(&engine->lock, NULL);
    pthread_create(&engine->th, NULL, (void *)&engine_thread, engine);
    return engine;
}

/*
 *  添加模型
 *  参数:
 *      xyz: 在空间中的初始位置(可以置NULL)
 *      roll_xyz: 绕自身坐标系旋转角度,即朝向,欧拉角,单位:度(可以置NULL)
 * 
 *  返回: 模型运动控制器,可进行异步控制、获取模型的运动状态
 *  其它: 当 xyz = NULL, roll_xyz = NULL 模型在空间默认位置为原点,面朝x轴正方向,头顶z轴正方向,左边y轴正方向
 */
_3D_Sport *engine_model_add(_3D_Engine *engine, _3D_Model *model, float *xyz, float *roll_xyz)
{
    _3D_Unit *unit, *tmpUnit;
    //参数检查
    if (!model)
        return NULL;
    //参数初始化
    unit = (_3D_Unit *)calloc(1, sizeof(_3D_Unit));
    unit->sport = (_3D_Sport *)calloc(1, sizeof(_3D_Sport));
    unit->sport->quat[0] = 1.0f;
    unit->model = model;
    if (xyz)
        memcpy(unit->sport->xyz, xyz, sizeof(float) * 3);
    if (roll_xyz)
    {
        pry_to_quat2(roll_xyz, unit->sport->quat);
        quat_to_pry2(unit->sport->quat, unit->sport->roll_xyz);
    }
    //加入链表
    pthread_mutex_lock(&engine->lock);
    if (engine->unit == NULL)
        engine->unit = unit;
    else
    {
        tmpUnit = engine->unit;
        while (tmpUnit->next)
            tmpUnit = tmpUnit->next;
        tmpUnit->next = unit;
    }
    pthread_mutex_unlock(&engine->lock);
    return unit->sport;
}

// 模型移除,成功返回true (注意 sport 指针成功移除等于被释放,不能再使用)
bool engine_model_remove(_3D_Engine *engine, _3D_Sport *sport)
{
    _3D_Unit *unit, *unitNext;
    if (engine->unit)
    {
        //是第一个
        if (engine->unit->sport == sport)
        {
            pthread_mutex_lock(&engine->lock);
            unit = engine->unit;
            engine->unit = engine->unit->next;
            free(unit->sport);
            free(unit);
            pthread_mutex_unlock(&engine->lock);
            return true;
        }
        //不是第一个
        else
        {
            //检索
            unit = engine->unit;
            unitNext = engine->unit->next;
            while (unitNext && unitNext->sport != sport)
            {
                unit = unit->next;
                unitNext = unit->next;
            }
            //移除
            if (unitNext && unitNext->sport == sport)
            {
                pthread_mutex_lock(&engine->lock);
                unit->next = unitNext->next;
                free(unitNext->sport);
                free(unitNext);
                pthread_mutex_unlock(&engine->lock);
                return true;
            }
        }
    }
    return false;
}

/*
 *  三维坐标点相对于相机的位置转换
 *  参数:
 *      xyz[3 * pointTotal]: 坐标点数组
 *      pointTotal: 数组中坐标点的个数
 */
static void engine_position_of_camera(_3D_Camera *camera, float *xyz, uint32_t pointTotal)
{
    uint32_t xyzCount;
    //先把相机的平移转嫁为坐标点相对相机的平移(即让坐标点以相机位置作为原点)
    for (xyzCount = 0; xyzCount < pointTotal * 3;)
    {
        //注意这里要反方向平移
        xyz[xyzCount++] -= camera->xyz[0];
        xyz[xyzCount++] -= camera->xyz[1];
        xyz[xyzCount++] -= camera->xyz[2];
    }
    //再把相机自身的旋转转嫁为坐标点相对相机的旋转
    for (xyzCount = 0; xyzCount < pointTotal * 3; xyzCount += 3)
    {
        // quat_to_pry(camera->quat, roll_xyz);
        // matrix_xyz(roll_xyz, &xyz[xyzCount], &xyz[xyzCount]);
        quat_roll(camera->quat, NULL, 0, &xyz[xyzCount], true);
    }
}

/*
 *  透视投影三维坐标点到相机的二维平面
 *      xyz[3 * pointTotal]: 坐标点数组
 *      pointTotal: 数组中坐标点的个数
 *      xy: 返回平面坐标数组
 *      depth: 返回每个 xy 点的深度信息,单位:点
 *      inside: 返回每个 xy 点是否在屏幕内
 */
static void engine_project_into_camera(
    _3D_Camera *camera,
    float *xyz,
    uint32_t pointTotal,
    float *xy,
    float *depth,
    bool *inside)
{
    uint32_t cInside, cXy, cXyz; //三个数组的指针移动计数
    //处理每一个点
    for (cInside = cXy = cXyz = 0; cInside < pointTotal; cInside += 1, cXy += 2, cXyz += 3)
    {
        //根据相机参数进行透视投影
        inside[cInside] = projection(
            camera->openAngle,
            &xyz[cXyz],
            camera->ar,
            camera->near,
            camera->far,
            &xy[cXy],
            &depth[cInside]);
        //由于投影矩阵计算时是假设屏幕高为2(继而宽为2ar)的情况下计算,这里需对坐标进行比例恢复
        xy[cXy] = xy[cXy] / (2 * camera->ar) * camera->width;
        xy[cXy + 1] = xy[cXy + 1] / 2 * camera->height;
        //把坐标原点移动到屏幕中心
        xy[cXy] = xy[cXy] + camera->width / 2;
        xy[cXy + 1] = camera->height / 2 - xy[cXy + 1];
    }
}

// 相机抓拍,照片缓存在 camera->photoMap
void engine_photo(_3D_Engine *engine, _3D_Camera *camera)
{
    uint32_t count;
    float xyz[3 * 3]; //3个三维坐标
    float xy[3 * 2]; //3个二维坐标
    float depth[3]; //3个坐标的深度信息
    bool inside[3]; //3个坐标是否入屏

    _3D_Unit *unit;
    _3D_Plane *plane;
    _3D_Label *label;

    //遍历单元链表
    unit = engine->unit;
    while (unit)
    {
        //遍历plane链表
        plane = unit->model->plane;
        while (plane)
        {
            //拷贝坐标数组
            for (count = 0; count < 9; count++)
                xyz[count] = plane->xyz[count];

            //坐标点相对于相机的位置变化
            engine_position_of_camera(camera, xyz, 3);
            //透视投影三维坐标点到相机的二维平面,得到二维坐标点信息
            engine_project_into_camera(camera, xyz, 3, xy, depth, inside);

            //画三角平面
            if (inside[0] || inside[1] || inside[2]) //有任意一点入屏
            {
                ;
            }

            //下一个
            plane = plane->next;
        }

        //遍历label链表
        label = unit->model->label;
        while (label)
        {
            //拷贝坐标
            for (count = 0; count < 3; count++)
                xyz[count] = label->xyz[count];

            //坐标点相对于相机的位置变化
            engine_position_of_camera(camera, xyz, 1);
            //透视投影三维坐标点到相机的二维平面,得到二维坐标点信息
            engine_project_into_camera(camera, xyz, 1, xy, depth, inside);

            //画点
            ;
            //画label
            ;

            //下一个
            label = label->next;
        }

        //下一个
        unit = unit->next;
    }
}

// 开始
void engine_start(_3D_Engine *engine)
{
    engine->run = true;
}

// 暂停
void engine_pause(_3D_Engine *engine)
{
    engine->run = false;
}

// 内存销毁(注意其中用到的 model 和 camera 需自行销毁)
void engine_release(_3D_Engine **engine)
{
    _3D_Unit *unit, *unitNext;
    if (engine && (*engine))
    {
        //结束线程
        (*engine)->threadExit = true;
        pthread_join((*engine)->th, NULL);
        pthread_mutex_destroy(&(*engine)->lock);
        //释放链表
        if ((*engine)->unit)
        {
            unitNext = (*engine)->unit;
            do
            {
                unit = unitNext;
                unitNext = unitNext->next;
                free(unit->sport);
                free(unit);
            } while (unitNext);
        }
        free(*engine);
        *engine = NULL;
    }
}
