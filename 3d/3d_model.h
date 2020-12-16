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
#include <stdbool.h>
#include <stdarg.h>

// 三维空间点的连线关系
typedef struct _3DNet
{
    uint32_t pSrc;       //选定点的序号
    uint32_t *pDist;     //连接点的序号数组(用一个src点连接多个dist点)
    uint32_t pDistCount; //连接点数量
    uint32_t rgbColor;   //线颜色
    struct _3DNet *next;
} _3D_Net;

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
    uint32_t pCount;     //点的数量
    float *xyz;          //点数组(内存长度为 sizeof(float)*3*pCount)
    uint32_t *rgbColor;  //点颜色数组(内存长度为 sizeof(uint32_t)*pCount)
    _3D_Net *net;        //连线关系链表
    _3D_Label *label;    //注释链表
    uint32_t labelCount; //注释链表长度
} _3D_Model;

/*
 *  模型初始化,点设置
 *  参数:
 *      pCount: 点个数
 *      x, y, z, rgbColor: 第一个点的坐标和颜色参数
 *      ...: 变长参数,按照第一个点的格式,继续凑够 pCount 个点的参数
 * 
 *  注意!! 上面x,y,z为float类型的变长参数必须使用0.0格式, 例如: 3要写成3.0
 * 
 *  返回: NULL/失败
 */
_3D_Model *model_init(uint32_t pCount, float x, float y, float z, uint32_t rgbColor, ...);

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
_3D_Model *model_init2(uint32_t pCount, uint32_t rgbColor, bool autoNet, bool circleNet, float *xyzArray);

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
_3D_Model *model_init3(uint32_t pCount, uint32_t rgbColor, bool autoNet, bool circleNet, float *xyArray, float z, char mode);

/*
 *  一维数组导入(model_init3的变种)
 *  参数:
 *      xArray: 一维坐标点数组,内存长度为 sizeof(float) * pCount
 *      y: 指定y值
 *      z: 指定z值
 */
_3D_Model *model_init4(uint32_t pCount, uint32_t rgbColor, bool autoNet, bool circleNet, float *xArray, float y, float z, char mode);

/*
 *  连线关系,以 pSrc 作为顶点,和多个 pDist 点相连
 *  参数:
 *      rgbColor: 连线颜色
 *      pSrc: 选定点序号, 按初始化时传入点的顺序, 从0数起
 *      pDistCount: 要连接到的点的个数
 *      pDist: 第一个要连的点的序号, 按初始化时传入点的顺序, 从0数起
 *      ...: 变长参数,参考 pDist 格式继续凑够 pDistCount 个参数
 */
void model_net_add(_3D_Model *model, uint32_t rgbColor, uint32_t pSrc, uint32_t pDistCount, uint32_t pDist, ...);

/*
 *  添加注释
 *  参数:
 *      rgbColor: 连线颜色
 *      label: 注释内容
 *      x, y, z: 位置
 */
void model_label_add(_3D_Model *model, uint32_t rgbColor, float x, float y, float z, char *text);

// 模型拷贝
_3D_Model *model_copy(_3D_Model *model);

// 内存销毁
void model_release(_3D_Model **model);

#endif
