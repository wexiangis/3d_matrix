/*
 *  三维模型的定义
 */
#ifndef _3D_MODEL_H_
#define _3D_MODEL_H_

#include <stdint.h>
#include <stdarg.h>

// 三维空间下的点属性
typedef struct _3DPoint
{
    double x, y, z;
} _3D_Point;

// 三维空间点的连线关系
typedef struct _3DNet
{
    uint32_t pSrcOrder;   //选定点的序号
    uint32_t *pDistOrder; //连接点的序号数组(用一个src点连接多个dist点)
    uint32_t pDistCount;  //连接点数量
    uint32_t color;       //线颜色
    struct _3DNet *next;
} _3D_Net;

// 点注释
typedef struct _3DLabel
{
    _3D_Point point; //位置
    char *comment;   //注释内容
    int color;       //文字颜色
    struct _3DLabel *next;
} _3D_Label;

// 主结构体
typedef struct _3DModel
{
    uint32_t pCount;   //点的数量
    _3D_Point *pArray; //点数组
    _3D_Net *net;      //连线关系
    _3D_Label *label;  //注释
} _3D_Model;

/*
 *  模型初始化,点设置
 *  参数:
 *      pCount: 点个数
 *      x, y, z, color: 第一个点的坐标和颜色参数
 *      ...: 变长参数,按照第一个点的格式,继续凑够 pCount 个点的参数
 * 
 *  返回: NULL/失败
 */
_3D_Model *_3d_model_init(uint32_t pCount, double x, double y, double z, uint32_t color, ...);

/*
 *  连线关系,以 pSrc 作为顶点,和多个 pDist 点相连
 *  参数:
 *      color: 连线颜色
 *      pSrc: 选定点序号, 按初始化时传入点的顺序, 从0数起
 *      pDistCount: 要连接到的点的个数
 *      pDist: 第一个要连的点的序号, 按初始化时传入点的顺序, 从0数起
 *      ...: 变长参数,参考 pDist 格式继续凑够 pDistCount 个参数
 */
void _3d_model_net_add(_3D_Model *model, uint32_t color, uint32_t pSrc, uint32_t pDistCount, uint32_t pDist, ...);

/*
 *  添加注释
 *  参数:
 *      color: 连线颜色
 *      comment: 注释内容
 *      x, y, z: 位置
 */
void _3d_model_comment_add(_3D_Model *model, uint32_t color, char *comment, double x, double y, double z);

// 模型拷贝
_3D_Model *_3d_model_copy(_3D_Model *model);

// 内存销毁
void _3d_model_release(_3D_Model **model);

#endif
