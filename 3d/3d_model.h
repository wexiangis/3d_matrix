/*
 *  三维模型的定义
 * 
 *  address: https://github.com/wexiangis/3d_matrix
 *  address2: https://gitee.com/wexiangis/matrix_3d
 * 
 *  update: 2020.11.24 - wexiangis - 初版
 */
#ifndef _3D_MODEL_H_
#define _3D_MODEL_H_

#include <stdint.h>

// 三维空间的三角平面(任意多边形可以通过"三角剖分"拆分为有限个三角形的组合)
typedef struct _3DPlane
{
    float xyz[3][3];
    uint32_t rgbColor; //面颜色
    struct _3DPlane *next;
} _3D_Plane;

// 点注释
typedef struct _3DLabel
{
    float xyz[3];      //位置
    char *text;        //注释内容
    uint32_t rgbColor; //文字颜色
    struct _3DLabel *next;
} _3D_Label;

// 主结构体
typedef struct _3DModel
{
    _3D_Plane *plane;    //三角平面链表
    _3D_Label *label;    //注释链表
    uint32_t planeCount; //平面链表长度
    uint32_t labelCount; //注释链表长度
} _3D_Model;

/*
 *  模型初始化,添加三角平面
 *  参数:
 *      model: 传入为NULL时自动创建内存
 *      rgbColor: 平面颜色
 *      xyz: 三个点的位置
 * 
 *  返回: 更新后的模型指针
 */
_3D_Model *model_plane_add(_3D_Model *model, uint32_t rgbColor, float xyz[3][3]);
_3D_Model *model_plane_add2(_3D_Model *model, uint32_t rgbColor, float xyz1[3], float xyz2[3], float xyz3[3]);
_3D_Model *model_plane_add3(_3D_Model *model, uint32_t rgbColor,
    float x1, float y1, float z1,
    float x2, float y2, float z2,
    float x3, float y3, float z3);

/*
 *  模型初始化,添加注释
 *  参数:
 *      model: 传入为NULL时自动创建内存
 *      rgbColor: 连线颜色
 *      text: 注释内容
 *      xyz: 位置
 * 
 *  返回: 更新后的模型指针
 */
_3D_Model *model_label_add(_3D_Model *model, uint32_t rgbColor, char *text, float xyz[3]);
_3D_Model *model_label_add2(_3D_Model *model, uint32_t rgbColor, char *text, float x, float y, float z);

// 模型拷贝
_3D_Model *model_copy(_3D_Model *model);

// 内存销毁
void model_release(_3D_Model **model);

#endif
