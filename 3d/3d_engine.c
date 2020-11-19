/*
 *  为3D模型和摄像机提供场地,并在此基础上展开运动计算
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "3d_engine.h"

#define _3D_ENGINE_PI 3.1415926535897
#define _3D_ENGINE_2PI (_3D_ENGINE_PI * 2)

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
void _3d_engine_sport(_3D_Engine *engine, _3D_Sport *sport)
{
    uint32_t count;
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
    for (count = 0; count < 3; count++)
    {
        sport->roll_xyz[count] += sport->speed_angle[count] * engine->intervalMs / 1000;
        //范围限制
        if (sport->roll_xyz[count] > _3D_ENGINE_2PI)
            sport->roll_xyz[count] -= _3D_ENGINE_2PI;
        else if (sport->roll_xyz[count] < -_3D_ENGINE_2PI)
            sport->roll_xyz[count] += _3D_ENGINE_2PI;
    }
}

// 主线程
void _3d_engine_thread(void *argv)
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

        pthread_mutex_lock(&engine->lock);

        //遍历模型链表
        unit = engine->unit;
        while (unit)
        {
            //更新运动状态
            _3d_engine_sport(engine, &unit->sport);
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
_3D_Engine *_3d_engine_init(uint32_t intervalMs, uint32_t xSize, uint32_t ySize, uint32_t zSize)
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
    engine->xyzRange[0][0] = xSize / 2;
    engine->xyzRange[0][1] = -xSize / 2;
    engine->xyzRange[1][0] = ySize / 2;
    engine->xyzRange[1][1] = -ySize / 2;
    engine->xyzRange[2][0] = zSize / 2;
    engine->xyzRange[2][1] = -zSize / 2;
    pthread_mutex_init(&engine->lock, NULL);
    pthread_create(&engine->th, NULL, (void *)&_3d_engine_thread, engine);
    return engine;
}

/*
 *  添加模型
 *  参数:
 *      xyz: 在空间中的初始位置
 *      roll_xyz: 绕自身坐标系旋转角度,即朝向
 * 
 *  返回: 模型运动控制器,可用于移除时使用
 */
_3D_Sport *_3d_engine_model_add(_3D_Engine *engine, _3D_Model *model, double *xyz, double *roll_xyz)
{
    _3D_Unit *unit, *tmpUnit;
    //参数检查
    if (!model)
        return NULL;
    //参数初始化
    unit = (_3D_Unit *)calloc(1, sizeof(_3D_Unit));
    unit->model = model;
    if (xyz)
        memcpy(unit->sport.xyz, xyz, sizeof(double) * 3);
    if (roll_xyz)
        memcpy(unit->sport.roll_xyz, roll_xyz, sizeof(double) * 3);
    //加入链表
    if (engine->unit == NULL)
        engine->unit = unit;
    else
    {
        tmpUnit = engine->unit;
        while (tmpUnit->next)
            tmpUnit = tmpUnit->next;
        tmpUnit->next = unit;
    }
    return &unit->sport;
}

// 模型移除
void _3d_engine_model_remove(_3D_Engine *engine, _3D_Sport *sport)
{
    _3D_Unit *unit, *unitNext;
    if (engine->unit)
    {
        //是第一个
        if (&engine->unit->sport == sport)
        {
            pthread_mutex_lock(&engine->lock);
            unit = engine->unit;
            engine->unit = engine->unit->next;
            free(unit);
            pthread_mutex_unlock(&engine->lock);
        }
        //不是第一个
        else
        {
            //检索
            unit = engine->unit;
            unitNext = engine->unit->next;
            while (unitNext && &unitNext->sport != sport)
            {
                unit = unit->next;
                unitNext = unit->next;
            }
            //移除
            if (unitNext && (&unitNext->sport) == sport)
            {
                pthread_mutex_lock(&engine->lock);
                unit->next = unitNext->next;
                free(unitNext);
                pthread_mutex_unlock(&engine->lock);
            }
        }
    }
}

// 相机抓拍,照片缓存在 camera->photoMap
void _3d_engine_photo(_3D_Engine *engine, _3D_Camera *camera)
{
    ;
}

// 开始
void _3d_engine_start(_3D_Engine *engine)
{
    engine->run = true;
}

// 暂停
void _3d_engine_pause(_3D_Engine *engine)
{
    engine->run = false;
}

// 内存销毁(注意其中用到的 model 和 camera 需自行销毁)
void _3d_engine_release(_3D_Engine **engine)
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
            do{
                unit = unitNext;
                unitNext = unitNext->next;
                free(unit);
            }while(unitNext);
        }
        free(*engine);
        *engine = NULL;
    }
}
