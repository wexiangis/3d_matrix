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
 *      xyz: 在空间中的初始位置
 *      roll_xyz: 绕自身坐标系旋转角度,即朝向
 * 
 *  返回: 模型运动控制器,可用于移除时使用
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

// 模型移除
void engine_model_remove(_3D_Engine *engine, _3D_Sport *sport)
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
            }
        }
    }
}

/*
 *  获取模型身上每一个点的真实三维坐标
 *  参数:
 *      retXyz: 返回运算后模型所有点位置                              !! 用完记得free !!
 *      retXyzTotal: 返回 retXyz 中坐标点的个数
 *      retXyzLabel: 返回运算后模型所有label位置,可能*retXyzLabel=NULL !! 用完记得free !!
 *      retXyzLabelTotal: 返回 retXyzLabel 中坐标点的个数
 */
static void engine_model_location(
    _3D_Unit *unit,
    float **retXyz, uint32_t *retXyzTotal,
    float **retXyzLabel, uint32_t *retXyzLabelTotal)
{
    float *xyz = NULL;      //三维坐标数组
    float *xyzLabel = NULL; //注释坐标数组
    uint32_t xyzCount;      //坐标数组计数
    _3D_Label *label;
    //点数组分配内存
    xyz = (float *)calloc(unit->model->pCount * 3, sizeof(float));
    //根据sport参数对这些三维坐标进行旋转和平移
    for (xyzCount = 0; xyzCount < unit->model->pCount * 3;)
    {
        //先旋转
        matrix_zyx2(
            unit->sport->roll_xyz,
            &unit->model->xyz[xyzCount],
            &xyz[xyzCount]);
        //再平移
        xyz[xyzCount++] += unit->sport->xyz[0];
        xyz[xyzCount++] += unit->sport->xyz[1];
        xyz[xyzCount++] += unit->sport->xyz[2];
    }
    //返回
    *retXyz = xyz;
    *retXyzTotal = unit->model->pCount;
    //这个模型有 label 链表吗?
    label = unit->model->label;
    if (label && unit->model->labelCount > 0)
    {
        //点数组分配内存
        xyzLabel = (float *)calloc(unit->model->labelCount * 3, sizeof(float));
        //根据sport参数对这些三维坐标进行旋转和平移
        for (xyzCount = 0; xyzCount < unit->model->labelCount * 3 && label;)
        {
            //先旋转
            matrix_zyx2(
                unit->sport->roll_xyz,
                label->xyz,
                &xyzLabel[xyzCount]);
            //再平移
            xyzLabel[xyzCount++] += unit->sport->xyz[0];
            xyzLabel[xyzCount++] += unit->sport->xyz[1];
            xyzLabel[xyzCount++] += unit->sport->xyz[2];
            //链表下一个
            label = label->next;
        }
        //返回
        *retXyzLabel = xyzLabel;
        *retXyzLabelTotal = unit->model->labelCount;
    }
    else
    {
        //返回
        *retXyzLabel = NULL;
        *retXyzLabelTotal = 0;
    }
}

/*
 *  三维坐标点相对于相机的位置转换
 *  参数:
 *      xyz: 坐标点数组
 *      xyzTotal: 数组中坐标点的个数
 */
static void engine_location_in_camera(_3D_Camera *camera, float *xyz, uint32_t xyzTotal)
{
    uint32_t xyzCount;
    //先把相机的平移转嫁为坐标点相对相机的平移(即让坐标点以相机位置作为原点)
    for (xyzCount = 0; xyzCount < xyzTotal * 3;)
    {
        //注意这里要反方向平移
        xyz[xyzCount++] -= camera->xyz[0];
        xyz[xyzCount++] -= camera->xyz[1];
        xyz[xyzCount++] -= camera->xyz[2];
    }
    //再把相机自身的旋转转嫁为坐标点相对相机的旋转
    for (xyzCount = 0; xyzCount < xyzTotal * 3; xyzCount += 3)
    {
        // quat_to_pry(camera->quat, roll_xyz);
        // matrix_xyz(roll_xyz, &xyz[xyzCount], &xyz[xyzCount]);

        quat_roll(camera->quat, NULL, 0, &xyz[xyzCount], true);
    }
}

/*
 *  透视投影三维坐标点到相机的二维平面
 *      xyz: 坐标点数组
 *      xyzTotal: 数组中坐标点的个数
 *      retXy: 返回平面坐标数组                     !! 用完记得free !!
 *      retDepth: 返回每个 retXy 点的深度信息,单位:点 !! 用完记得free !!
 *      retInside: 返回每个 retXy 点是否在屏幕内     !! 用完记得free !!
 */
static void engine_project_in_camera(
    _3D_Camera *camera,
    float *xyz,
    uint32_t xyzTotal,
    float **retXy,
    float **retDepth,
    bool **retInside)
{
    uint32_t c1, c2, c3; //三种step的计数
    float *xy;
    float *depth;
    bool *inside;
    //内存分配
    xy = (float *)calloc(xyzTotal * 2, sizeof(float));
    depth = (float *)calloc(xyzTotal, sizeof(float));
    inside = (bool *)calloc(xyzTotal, sizeof(bool));
    //处理每一个点
    for (c1 = c2 = c3 = 0; c1 < xyzTotal; c1 += 1, c2 += 2, c3 += 3)
    {
        //根据相机参数进行透视投影
        inside[c1] = projection(
            camera->openAngle,
            &xyz[c3],
            camera->ar,
            camera->near,
            camera->far,
            &xy[c2],
            &depth[c1]);
        //由于投影矩阵计算时是假设屏幕高为2(继而宽为2ar)的情况下计算,这里需对坐标进行比例恢复
        xy[c2] = xy[c2] / (2 * camera->ar) * camera->width;
        xy[c2 + 1] = xy[c2 + 1] / 2 * camera->height;
        //把坐标原点移动到屏幕中心
        xy[c2] = xy[c2] + camera->width / 2;
        xy[c2 + 1] = camera->height / 2 - xy[c2 + 1];
    }
    //返回
    *retXy = xy;
    *retDepth = depth;
    *retInside = inside;
}

// 相机抓拍,照片缓存在 camera->photoMap
void engine_photo(_3D_Engine *engine, _3D_Camera *camera)
{
    float *xyz;        //坐标数组
    uint32_t xyzTotal; //坐标点总数
    float *xy;
    float *depth;
    bool *inside;

    float *xyzLabel;        //注释坐标数组
    uint32_t xyzLabelTotal; //注释坐标点总数
    float *xyLabel;
    float *depthLabel;
    bool *insideLabel;

    _3D_Unit *unit;
    _3D_Net *net;
    _3D_Label *label;

    uint32_t count;
    int32_t xyStart[2], xyEnd[2];

    //遍历模型链表
    unit = engine->unit;
    while (unit)
    {
        //清空可能分配内存的指针
        xyz = xyzLabel = NULL;
        xyzTotal = xyzLabelTotal = 0;
        xy = xyLabel = NULL;
        depth = depthLabel = NULL;
        inside = insideLabel = NULL;

        //获取模型身上每一个点的真实三维坐标
        engine_model_location(unit, &xyz, &xyzTotal, &xyzLabel, &xyzLabelTotal);

        //坐标点相对于相机的位置变化
        if (xyz && xyzTotal > 0)
            engine_location_in_camera(camera, xyz, xyzTotal);
        if (xyzLabel && xyzLabelTotal > 0)
            engine_location_in_camera(camera, xyzLabel, xyzLabelTotal);

        //透视投影三维坐标点到相机的二维平面,得到二维坐标点信息
        if (xyz && xyzTotal > 0)
            engine_project_in_camera(camera, xyz, xyzTotal, &xy, &depth, &inside);
        if (xyzLabel && xyzLabelTotal > 0)
            engine_project_in_camera(camera, xyzLabel, xyzLabelTotal, &xyLabel, &depthLabel, &insideLabel);

        //画模型连线关系画线
        net = unit->model->net;
        if (xy && inside && xyzTotal > 0 && net)
        {
            //遍历net链表
            while (net)
            {
                //src点和所有dist点连线
                for (count = 0; count < net->pDistCount; count++)
                {
                    //注意这里转换为int32_t类型、数组计数要*2
                    xyStart[0] = (int32_t)xy[net->pSrc * 2];
                    xyStart[1] = (int32_t)xy[net->pSrc * 2 + 1];
                    xyEnd[0] = (int32_t)xy[net->pDist[count] * 2];
                    xyEnd[1] = (int32_t)xy[net->pDist[count] * 2 + 1];
                    //检查点是否在屏幕内,且屏幕内的点优先作为xyStart[2]
                    if (inside[net->pSrc])
                        _2d_draw_line(
                            camera->photoMap, camera->width, camera->height,
                            xyStart, xyEnd,
                            net->rgbColor, 1);
                    else if (inside[net->pDist[count]])
                        _2d_draw_line(
                            camera->photoMap, camera->width, camera->height,
                            xyEnd, xyStart,
                            net->rgbColor, 1);
                }
                //下一个
                net = net->next;
            }
        }

        //画模型注释
        label = unit->model->label;
        if (xyLabel && insideLabel && xyzLabelTotal > 0 && label)
        {
            count = 0;
            //遍历net链表
            while (label && count < xyzLabelTotal)
            {
                xyStart[0] = (int32_t)xyLabel[count * 2];
                xyStart[1] = (int32_t)xyLabel[count * 2 + 1];
                //画点
                _2d_draw_dot(
                    camera->photoMap, camera->width, camera->height,
                    xyStart, label->rgbColor, 1);
                //画label
                ;
                //下一个
                label = label->next;
                count += 1;
            }
        }

        //内存回收
        if (xyz)
            free(xyz);
        if (xy)
            free(xy);
        if (depth)
            free(depth);
        if (inside)
            free(inside);
        if (xyzLabel)
            free(xyzLabel);
        if (xyLabel)
            free(xyLabel);
        if (depthLabel)
            free(depthLabel);
        if (insideLabel)
            free(insideLabel);

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
