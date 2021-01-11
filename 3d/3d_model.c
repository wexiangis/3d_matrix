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
 *  模型初始化,添加三角平面
 *  参数:
 *      model: 传入为NULL时自动创建内存
 *      rgbColor: 平面颜色
 *      xyz: 三个点的位置
 * 
 *  返回: 更新后的模型指针
 */
_3D_Model *model_plane_add(_3D_Model *model, uint32_t rgbColor, float xyz[3][3])
{
    _3D_Plane *plane;
    uint32_t i;

    if (!model)
        model = (_3D_Model *)calloc(1, sizeof(_3D_Model));
    
    if (!model->plane)
        model->plane = plane = (_3D_Plane *)calloc(1, sizeof(_3D_Plane));
    else
    {
        plane = model->plane;
        while(plane->next)
            plane = plane->next;
        plane->next = (_3D_Plane *)calloc(1, sizeof(_3D_Plane));
        plane = plane->next;
    }

    for (i = 0; i < 3; i++)
    {
        plane->xyz[i][0] = xyz[i][0];
        plane->xyz[i][1] = xyz[i][1];
        plane->xyz[i][2] = xyz[i][2];
    }

    plane->rgbColor = rgbColor;

    model->planeCount += 1;

    return model;
}

_3D_Model *model_plane_add2(_3D_Model *model, uint32_t rgbColor, float xyz1[3], float xyz2[3], float xyz3[3])
{
    float *_xyz[3] = {xyz1, xyz2, xyz3};
    return model_plane_add(model, rgbColor, _xyz);
}

_3D_Model *model_plane_add3(_3D_Model *model, uint32_t rgbColor,
    float x1, float y1, float z1,
    float x2, float y2, float z2,
    float x3, float y3, float z3)
{
    float _xyz[3][3] = {
        {x1, y1, z1},
        {x2, y2, z2},
        {x3, y3, z3}
    };
    return model_plane_add(model, rgbColor, _xyz);
}

/*
 *  模型初始化,添加注释
 *  参数:
 *      model: 传入为NULL时自动创建内存
 *      rgbColor: 连线颜色
 *      label: 注释内容
 *      xyz: 位置
 * 
 *  返回: 更新后的模型指针
 */
_3D_Model *model_label_add(_3D_Model *model, uint32_t rgbColor, char *text, float xyz[3])
{
    _3D_Label *label;
    uint32_t i;

    if (!model)
        model = (_3D_Model *)calloc(1, sizeof(_3D_Model));
    
    if (!model->label)
        model->label = label = (_3D_Label *)calloc(1, sizeof(_3D_Label));
    else
    {
        label = model->label;
        while(label->next)
            label = label->next;
        label->next = (_3D_Label *)calloc(1, sizeof(_3D_Label));
        label = label->next;
    }

    label->xyz[0] = xyz[0];
    label->xyz[1] = xyz[1];
    label->xyz[2] = xyz[2];

    label->rgbColor = rgbColor;

    label->text = (char *)calloc(strlen(text) + 1, 1);
    strcpy(label->text, text);

    model->labelCount += 1;

    return model;
}

_3D_Model *model_label_add2(_3D_Model *model, uint32_t rgbColor, char *text, float x, float y, float z)
{
    float _xyz[3] = {x, y, z};
    return model_label_add(model, rgbColor, text, _xyz);
}

// 模型拷贝
_3D_Model *model_copy(_3D_Model *model)
{
    _3D_Plane *plane, *plane2;
    _3D_Label *label, *label2;
    _3D_Model *model2 = (_3D_Model *)calloc(1, sizeof(_3D_Model));
    //net链表拷贝
    if (model->plane)
    {
        plane = model->plane;
        plane2 = model2->plane = (_3D_Plane *)calloc(1, sizeof(_3D_Plane));
        do
        {
            memcpy(plane2->xyz, plane->xyz, sizeof(float) * 3 * 3);
            plane2->rgbColor = plane->rgbColor;
            //下一个
            plane = plane->next;
            if (plane)
            {
                plane2->next = (_3D_Plane *)calloc(1, sizeof(_3D_Plane));
                plane2 = plane2->next;
            }
        } while (plane);
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
    _3D_Plane *plane, *planeNext;
    _3D_Label *label, *labelNext;
    if (model && (*model))
    {
        //释放链表
        if ((*model)->plane)
        {
            planeNext = (*model)->plane;
            do
            {
                plane = planeNext;
                planeNext = planeNext->next;
                free(plane);
            } while (planeNext);
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
        //
        free((*model));
        (*model) = NULL;
    }
}