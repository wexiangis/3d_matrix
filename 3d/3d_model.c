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

/*
 *  模型初始化,添加线条
 *  参数:
 *      model: 传入为NULL时自动创建内存
 *      argbColor: 线颜色
 *      xyz[6 * count]: 三个点的坐标数组
 *      count: 线条数,决定xyz字节长度: count * 6 * sizeof(float)
 * 
 *  返回: 更新后的模型指针
 */
_3D_Model *model_line_add(_3D_Model *model, uint32_t argbColor, float *xyz, uint32_t count)
{
    _3D_Line *line;
    uint32_t i = 0;

    if (!model)
        model = (_3D_Model *)calloc(1, sizeof(_3D_Model));

    if (count < 1)
        return model;
    
    //取链表节点
    if (!model->line)
        model->line = line = (_3D_Line *)calloc(1, sizeof(_3D_Line));
    else
    {
        line = model->line;
        while(line->next)
            line = line->next;
        line->next = (_3D_Line *)calloc(1, sizeof(_3D_Line));
        line = line->next;
    }

    //逐个添加线条
    while(line)
    {
        //参数拷贝
        memcpy(line->xyz, xyz, sizeof(float) * 6);
        line->argbColor = argbColor;
        //下一个
        if (++i < count)
        {
            xyz += 6;
            line->next = (_3D_Line *)calloc(1, sizeof(_3D_Line));
        }
        line = line->next;
    }

    return model;
}

_3D_Model *model_line_add2(_3D_Model *model, uint32_t argbColor, float xyz1[3], float xyz2[3])
{
    float _xyz[6] = {
        xyz1[0], xyz1[1], xyz1[2],
        xyz2[0], xyz2[1], xyz2[2],
    };
    return model_line_add(model, argbColor, _xyz, 1);
}

_3D_Model *model_line_add3(_3D_Model *model, uint32_t argbColor,
    float x1, float y1, float z1,
    float x2, float y2, float z2)
{
    float _xyz[6] = {
        x1, y1, z1,
        x2, y2, z2,
    };
    return model_line_add(model, argbColor, _xyz, 1);
}

/*
 *  模型初始化,添加三角平面
 *  参数:
 *      model: 传入为NULL时自动创建内存
 *      argbColor: 平面颜色
 *      xyz[9 * count]: 三个点的坐标数组
 *      count: 三角平面个数,决定 xyz 数组的字节长度: count * 9 * sizeof(float)
 * 
 *  返回: 更新后的模型指针
 */
_3D_Model *model_plane_add(_3D_Model *model, uint32_t argbColor, float *xyz, uint32_t count)
{
    _3D_Plane *plane;
    uint32_t i = 0;

    if (!model)
        model = (_3D_Model *)calloc(1, sizeof(_3D_Model));

    if (count < 1)
        return model;
    
    //取链表节点
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

    //逐个添加平面
    while(plane)
    {
        //参数拷贝
        memcpy(plane->xyz, xyz, sizeof(float) * 9);
        plane->argbColor = argbColor;
        //下一个
        if (++i < count)
        {
            xyz += 9;
            plane->next = (_3D_Plane *)calloc(1, sizeof(_3D_Plane));
        }
        plane = plane->next;
    }

    return model;
}

_3D_Model *model_plane_add2(_3D_Model *model, uint32_t argbColor, float xyz1[3], float xyz2[3], float xyz3[3])
{
    float _xyz[9] = {
        xyz1[0], xyz1[1], xyz1[2],
        xyz2[0], xyz2[1], xyz2[2],
        xyz3[0], xyz3[1], xyz3[2],
    };
    return model_plane_add(model, argbColor, _xyz, 1);
}

_3D_Model *model_plane_add3(_3D_Model *model, uint32_t argbColor,
    float x1, float y1, float z1,
    float x2, float y2, float z2,
    float x3, float y3, float z3)
{
    float _xyz[9] = {
        x1, y1, z1,
        x2, y2, z2,
        x3, y3, z3,
    };
    return model_plane_add(model, argbColor, _xyz, 1);
}

/*
 *  模型初始化,添加注释
 *  参数:
 *      model: 传入为NULL时自动创建内存
 *      argbColor: 连线颜色
 *      text: 注释内容,不用可以置NULL
 *      xyz: 位置
 * 
 *  返回: 更新后的模型指针
 */
_3D_Model *model_label_add(_3D_Model *model, uint32_t argbColor, char *text, float xyz[3])
{
    _3D_Label *label;

    if (!model)
        model = (_3D_Model *)calloc(1, sizeof(_3D_Model));
    
    //取链表节点
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

    //参数拷贝
    label->xyz[0] = xyz[0];
    label->xyz[1] = xyz[1];
    label->xyz[2] = xyz[2];
    label->argbColor = argbColor;
    if (text)
    {
        label->text = (char *)calloc(strlen(text) + 1, 1);
        strcpy(label->text, text);
    }

    return model;
}

_3D_Model *model_label_add2(_3D_Model *model, uint32_t argbColor, char *text, float x, float y, float z)
{
    float _xyz[3] = {x, y, z};
    return model_label_add(model, argbColor, text, _xyz);
}

// 模型拷贝
_3D_Model *model_copy(_3D_Model *model)
{
    _3D_Line *line, *line2;
    _3D_Plane *plane, *plane2;
    _3D_Label *label, *label2;
    _3D_Model *model2 = (_3D_Model *)calloc(1, sizeof(_3D_Model));
    //line链表拷贝
    if (model->line)
    {
        line = model->line;
        line2 = model2->line = (_3D_Line *)calloc(1, sizeof(_3D_Line));
        do
        {
            memcpy(line2->xyz, line->xyz, sizeof(float) * 6);
            line2->argbColor = line->argbColor;
            //下一个
            line = line->next;
            if (line)
            {
                line2->next = (_3D_Line *)calloc(1, sizeof(_3D_Line));
                line2 = line2->next;
            }
        } while (line);
    }
    //plane链表拷贝
    if (model->plane)
    {
        plane = model->plane;
        plane2 = model2->plane = (_3D_Plane *)calloc(1, sizeof(_3D_Plane));
        do
        {
            memcpy(plane2->xyz, plane->xyz, sizeof(float) * 9);
            plane2->argbColor = plane->argbColor;
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
            label2->argbColor = label->argbColor;
            if (label->text)
            {
                label2->text = (char *)calloc(strlen(label->text), sizeof(char));
                strcpy(label2->text, label->text);
            }
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
    _3D_Line *line, *lineNext;
    _3D_Plane *plane, *planeNext;
    _3D_Label *label, *labelNext;
    if (model && (*model))
    {
        //释放链表
        if ((*model)->line)
        {
            lineNext = (*model)->line;
            do
            {
                line = lineNext;
                lineNext = lineNext->next;
                free(line);
            } while (lineNext);
        }
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