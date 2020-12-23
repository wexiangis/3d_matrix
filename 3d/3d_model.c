/*
 *  三维模型的定义
 * 
 *  address: https://github.com/wexiangis/3d_matrix
 *  address2: https://gitee.com/wexiangis/matrix_3d
 * 
 *  update: 2020.11.24 - wexiangis - 初版
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "3d_model.h"

#define _3D_MODEL_PI 3.1415926535897

/*
 *  模型初始化,点设置
 *  参数:
 *      pCount: 点个数
 *      x, y, z, rgbColor: 第一个点的坐标和颜色参数, 其中x,y,z必须写成0.00的格式, 如3写成3.00
 *      ...: 变长参数,按照第一个点的格式,继续凑够 pCount 个点的参数
 * 
 *  返回: NULL/失败
 */
_3D_Model *model_init(uint32_t pCount, float x, float y, float z, uint32_t rgbColor, ...)
{
    _3D_Model *model;
    va_list ap;
    uint32_t xyZCount = 0, rgbColorCount = 0;
    //参数检查
    if (pCount < 1)
        return NULL;
    //基本参数和内存准备
    model = (_3D_Model *)calloc(1, sizeof(_3D_Model));
    model->pCount = pCount;
    model->xyz = (float *)calloc(pCount * 3, sizeof(float));
    model->rgbColor = (uint32_t *)calloc(pCount, sizeof(uint32_t));
    //记录第一个点
    model->xyz[xyZCount++] = x;
    model->xyz[xyZCount++] = y;
    model->xyz[xyZCount++] = z;
    model->rgbColor[rgbColorCount++] = rgbColor;
    //记录其它点
    if (rgbColorCount < pCount)
    {
        va_start(ap, rgbColor);
        while (rgbColorCount < pCount)
        {
            model->xyz[xyZCount++] = va_arg(ap, double);
            model->xyz[xyZCount++] = va_arg(ap, double);
            model->xyz[xyZCount++] = va_arg(ap, double);
            model->rgbColor[rgbColorCount++] = va_arg(ap, uint32_t);
        }
        va_end(ap);
    }
    return model;
}

/*
 *  模型初始化2,数组导入
 *  参数:
 *      pCount: 点个数
 *      rgbColor: 点颜色
 *      autoNet: 相邻两点自动连线
 *      circleNet: 头尾两点连线
 *      xyzArray: xyz坐标点数组,内存长度为 sizeof(float) * 3 * pCount
 * 
 *  返回: NULL/失败
 */
_3D_Model *model_init2(uint32_t pCount, uint32_t rgbColor, bool autoNet, bool circleNet, float *xyzArray)
{
    _3D_Model *model;
    uint32_t xyZCount = 0, rgbColorCount = 0, arrayCount = 0;
    //参数检查
    if (pCount < 1)
        return NULL;
    //基本参数和内存准备
    model = (_3D_Model *)calloc(1, sizeof(_3D_Model));
    model->pCount = pCount;
    model->xyz = (float *)calloc(pCount * 3, sizeof(float));
    model->rgbColor = (uint32_t *)calloc(pCount, sizeof(uint32_t));
    //添加点
    while (rgbColorCount < pCount)
    {
        model->xyz[xyZCount++] = xyzArray[arrayCount++];
        model->xyz[xyZCount++] = xyzArray[arrayCount++];
        model->xyz[xyZCount++] = xyzArray[arrayCount++];
        model->rgbColor[rgbColorCount++] = rgbColor;
    }
    //相邻两点自动连线
    if (autoNet && pCount > 1)
    {
        for (xyZCount = 1; xyZCount < pCount; xyZCount++)
            model_net_add(model, rgbColor, xyZCount - 1, 1, xyZCount);
    }
    //头尾两点连线
    if (circleNet && pCount > 1)
        model_net_add(model, rgbColor, 0, 1, pCount - 1);
    return model;
}

#define _model_xyz_mode(x, y, z, mode) \
if (mode == 1) {\
    model->xyz[xyZCount++] = x;\
    model->xyz[xyZCount++] = z;\
    model->xyz[xyZCount++] = y;\
} else if (mode == 2) {\
    model->xyz[xyZCount++] = y;\
    model->xyz[xyZCount++] = x;\
    model->xyz[xyZCount++] = z;\
} else if (mode == 3) {\
    model->xyz[xyZCount++] = y;\
    model->xyz[xyZCount++] = z;\
    model->xyz[xyZCount++] = x;\
} else if (mode == 4) {\
    model->xyz[xyZCount++] = z;\
    model->xyz[xyZCount++] = x;\
    model->xyz[xyZCount++] = y;\
} else if (mode == 3) {\
    model->xyz[xyZCount++] = y;\
    model->xyz[xyZCount++] = z;\
    model->xyz[xyZCount++] = x;\
} else {\
    model->xyz[xyZCount++] = x;\
    model->xyz[xyZCount++] = y;\
    model->xyz[xyZCount++] = z;\
}

/*
 *  二维数组导入(model_init2的变种)
 *  参数:
 *      xyArray: 二维坐标点数组,内存长度为 sizeof(float) * 2 * pCount
 *      z: 指定z值
 *      mode: 指定三轴坐标映射方式(即坐标轴调换)
 *          0 / xyz --> xyz (默认)
 *          1 / xyz --> xzy
 *          2 / xyz --> yxz
 *          3 / xyz --> yzx
 *          4 / xyz --> zxy
 *          5 / xyz --> zyx
 */
_3D_Model *model_init_map(uint32_t pCount, uint32_t rgbColor, bool autoNet, bool circleNet, float *xyArray, float z, char mode)
{
    _3D_Model *model;
    uint32_t xyZCount = 0, rgbColorCount = 0, arrayCount = 0;
    //参数检查
    if (pCount < 1)
        return NULL;
    //基本参数和内存准备
    model = (_3D_Model *)calloc(1, sizeof(_3D_Model));
    model->pCount = pCount;
    model->xyz = (float *)calloc(pCount * 3, sizeof(float));
    model->rgbColor = (uint32_t *)calloc(pCount, sizeof(uint32_t));
    //添加点
    while (rgbColorCount < pCount)
    {
        _model_xyz_mode(xyArray[arrayCount++], xyArray[arrayCount++], z, mode);
        model->rgbColor[rgbColorCount++] = rgbColor;
    }
    //相邻两点自动连线
    if (autoNet && pCount > 1)
    {
        for (xyZCount = 1; xyZCount < pCount; xyZCount++)
            model_net_add(model, rgbColor, xyZCount - 1, 1, xyZCount);
    }
    //头尾两点连线
    if (circleNet && pCount > 1)
        model_net_add(model, rgbColor, 0, 1, pCount - 1);
    return model;
}

/*
 *  一维数组导入(model_init_map的变种)
 *  参数:
 *      xArray: 一维坐标点数组,内存长度为 sizeof(float) * pCount
 *      y: 指定y值
 *      z: 指定z值
 */
_3D_Model *model_init_line(uint32_t pCount, uint32_t rgbColor, bool autoNet, bool circleNet, float *xArray, float y, float z, char mode)
{
    _3D_Model *model;
    uint32_t xyZCount = 0, rgbColorCount = 0, arrayCount = 0;
    //参数检查
    if (pCount < 1)
        return NULL;
    //基本参数和内存准备
    model = (_3D_Model *)calloc(1, sizeof(_3D_Model));
    model->pCount = pCount;
    model->xyz = (float *)calloc(pCount * 3, sizeof(float));
    model->rgbColor = (uint32_t *)calloc(pCount, sizeof(uint32_t));
    //添加点
    while (rgbColorCount < pCount)
    {
        _model_xyz_mode(xArray[arrayCount++], y, z, mode);
        model->rgbColor[rgbColorCount++] = rgbColor;
    }
    //相邻两点自动连线
    if (autoNet && pCount > 1)
    {
        for (xyZCount = 1; xyZCount < pCount; xyZCount++)
            model_net_add(model, rgbColor, xyZCount - 1, 1, xyZCount);
    }
    //头尾两点连线
    if (circleNet && pCount > 1)
        model_net_add(model, rgbColor, 0, 1, pCount - 1);
    return model;
}

/*
 *  连线关系,以 pSrc 作为顶点,和多个 pDist 点相连
 *  参数:
 *      rgbColor: 连线颜色
 *      pSrc: 选定点序号, 按初始化时传入点的顺序, 从0数起
 *      pDistCount: 要连接到的点的个数
 *      pDist: 第一个要连的点的序号, 按初始化时传入点的顺序, 从0数起
 *      ...: 变长参数,参考 pDist 格式继续凑够 pDistCount 个参数
 */
void model_net_add(_3D_Model *model, uint32_t rgbColor, uint32_t pSrc, uint32_t pDistCount, uint32_t pDist, ...)
{
    _3D_Net *net, *tmpNet;
    va_list ap;
    uint32_t count = 0;
    //参数检查
    if (pDistCount < 1)
        return;
    //基本参数和内存准备
    net = (_3D_Net *)calloc(1, sizeof(_3D_Net));
    net->rgbColor = rgbColor;
    net->pSrc = pSrc;
    net->pDistCount = pDistCount;
    net->pDist = (uint32_t *)calloc(pDistCount, sizeof(uint32_t));
    //记录第一个点
    net->pDist[count] = pDist;
    count += 1;
    //记录其它点
    if (count < pDistCount)
    {
        va_start(ap, pDist);
        for (; count < pDistCount; count++)
            net->pDist[count] = va_arg(ap, uint32_t);
        va_end(ap);
    }
    //加入到模型的net链表
    if (model->net == NULL)
        model->net = net;
    else
    {
        tmpNet = model->net;
        while (tmpNet->next)
            tmpNet = tmpNet->next;
        tmpNet->next = net;
    }
}

/*
 *  添加注释
 *  参数:
 *      rgbColor: 连线颜色
 *      label: 注释内容
 *      x, y, z: 位置
 */
void model_label_add(_3D_Model *model, uint32_t rgbColor, float x, float y, float z, char *text)
{
    _3D_Label *label, *tmpLabel;
    //基本参数和内存准备
    label = (_3D_Label *)calloc(1, sizeof(_3D_Label));
    label->xyz[0] = x;
    label->xyz[1] = y;
    label->xyz[2] = z;
    label->rgbColor = rgbColor;
    label->text = (char *)calloc(strlen(text) + 1, sizeof(char));
    strcpy(label->text, text);
    //加入到模型的label链表
    if (model->label == NULL)
        model->label = label;
    else
    {
        tmpLabel = model->label;
        while (tmpLabel->next)
            tmpLabel = tmpLabel->next;
        tmpLabel->next = label;
    }
    model->labelCount += 1;
}

// 模型拷贝
_3D_Model *model_copy(_3D_Model *model)
{
    _3D_Net *net, *net2;
    _3D_Label *label, *label2;
    _3D_Model *model2 = (_3D_Model *)calloc(1, sizeof(_3D_Model));
    //点数组拷贝
    model2->pCount = model->pCount;
    model2->xyz = (float *)calloc(model2->pCount * 3, sizeof(float));
    model2->rgbColor = (uint32_t *)calloc(model2->pCount, sizeof(uint32_t));
    memcpy(model2->xyz, model->xyz, sizeof(float) * 3 * model2->pCount);
    memcpy(model2->rgbColor, model->rgbColor, sizeof(uint32_t) * model2->pCount);
    //net链表拷贝
    if (model->net)
    {
        net = model->net;
        net2 = model2->net = (_3D_Net *)calloc(1, sizeof(_3D_Net));
        do
        {
            net2->pSrc = net->pSrc;
            net2->rgbColor = net->rgbColor;
            net2->pDistCount = net->pDistCount;
            net2->pDist = (uint32_t *)calloc(net2->pDistCount, sizeof(uint32_t));
            memcpy(net2->pDist, net->pDist, sizeof(uint32_t) * net2->pDistCount);
            //下一个
            net = net->next;
            if (net)
            {
                net2->next = (_3D_Net *)calloc(1, sizeof(_3D_Net));
                net2 = net2->next;
            }
        } while (net);
    }
    //label链表拷贝
    if (model->label)
    {
        label = model->label;
        label2 = model2->label = (_3D_Label *)calloc(1, sizeof(_3D_Label));
        do
        {
            memcpy(label2->xyz, label->xyz, sizeof(float) * 3);
            label2->rgbColor = label->rgbColor;
            label2->text = (char *)calloc(strlen(label->text), sizeof(char));
            strcpy(label2->text, label->text);
            //下一个
            label = label->next;
            if (label)
            {
                label2->next = (_3D_Label *)calloc(1, sizeof(_3D_Label));
                label2 = label2->next;
            }
        } while (label);
    }
    return model2;
}

// 内存销毁
void model_release(_3D_Model **model)
{
    _3D_Net *net, *netNext;
    _3D_Label *label, *labelNext;
    if (model && (*model))
    {
        //释放链表
        if ((*model)->net)
        {
            netNext = (*model)->net;
            do
            {
                net = netNext;
                netNext = netNext->next;
                if (net->pDist)
                    free(net->pDist);
                free(net);
            } while (netNext);
        }
        //释放链表
        if ((*model)->label)
        {
            labelNext = (*model)->label;
            do
            {
                label = labelNext;
                labelNext = labelNext->next;
                if (label->text)
                    free(label->text);
                free(label);
            } while (labelNext);
        }
        //数组
        if ((*model)->xyz)
            free((*model)->xyz);
        if ((*model)->rgbColor)
            free((*model)->rgbColor);
        free((*model));
        (*model) = NULL;
    }
}