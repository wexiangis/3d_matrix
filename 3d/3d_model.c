/*
 *  三维模型的定义
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
_3D_Model *_3d_model_init(uint32_t pCount, double x, double y, double z, uint32_t rgbColor, ...)
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
    model->xyz = (double *)calloc(pCount * 3, sizeof(double));
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
 *  连线关系,以 pSrc 作为顶点,和多个 pDist 点相连
 *  参数:
 *      rgbColor: 连线颜色
 *      pSrc: 选定点序号, 按初始化时传入点的顺序, 从0数起
 *      pDistCount: 要连接到的点的个数
 *      pDist: 第一个要连的点的序号, 按初始化时传入点的顺序, 从0数起
 *      ...: 变长参数,参考 pDist 格式继续凑够 pDistCount 个参数
 */
void _3d_model_net_add(_3D_Model *model, uint32_t rgbColor, uint32_t pSrc, uint32_t pDistCount, uint32_t pDist, ...)
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
void _3d_model_label_add(_3D_Model *model, uint32_t rgbColor, double x, double y, double z, char *text)
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
_3D_Model *_3d_model_copy(_3D_Model *model)
{
    _3D_Net *net, *net2;
    _3D_Label *label, *label2;
    _3D_Model *model2 = (_3D_Model *)calloc(1, sizeof(_3D_Model));
    //点数组拷贝
    model2->pCount = model->pCount;
    model2->xyz = (double *)calloc(model2->pCount * 3, sizeof(double));
    model2->rgbColor = (uint32_t *)calloc(model2->pCount, sizeof(uint32_t));
    memcpy(model2->xyz, model->xyz, sizeof(double) * 3 * model2->pCount);
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
            memcpy(label2->xyz, label->xyz, sizeof(double) * 3);
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
    return NULL;
}

// 内存销毁
void _3d_model_release(_3D_Model **model)
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