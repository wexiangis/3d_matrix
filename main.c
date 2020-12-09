#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "3d_engine.h"
#include "delayus.h"
#include "fbmap.h"
#include "bmp.h"
#include "key.h"

//使能输出帧图片
// #define OUTPUT_FRAME_FOLDER "./frameOutput"

//main函数刷新间隔ms
#ifdef OUTPUT_FRAME_FOLDER
#define INTERVAL_MS 200 // 保存帧图片建议每秒5张
#else
#define INTERVAL_MS 50
#endif
//引擎计算间隔ms
#define ENGINE_INTERVAL_MS 50
//平移和旋转最小分度格
#define DIV_MOV  1 //单位:点
#define DIV_ROLL 1 //单位:度

//3个相机(三视图)
static _3D_Camera *camera1, *camera2, *camera3;
//3个模型
static _3D_Model *model0, *model1, *model2;
//3个运动控制器(往引擎添加模型后返回的运动控制指针)
static _3D_Sport *sport0, *sport1, *sport2;
//引擎
static _3D_Engine *engine;

//把大陀的初始化代码放到main函数后面,方便快速查看
void all_init(void);

//按键事件回调函数, 控制相机位置和角度
void key_callback(void *obj, int key, int type)
{
    // printf("key/%d type/%d\r\n", key, type);

    float mUpDown = 0, mLeftRight = 0, mFrontBack = 0;
    float rUpDown = 0, rLeftRight = 0, rClock = 0;

    //'r'键复位
    if (key == 19 && type == 0) // type = 0, 按键松开
    {
        camera_reset(camera1);
        camera_reset(camera2);
        camera_reset(camera3);
    }
    else if (type == 1 || type == 2) // type = 1,2, 按键按下或一直按住
    {
        //'w'键, 前移
        if (key == 17)
            mFrontBack = DIV_MOV;
        //'s'键， 后移
        else if (key == 31)
            mFrontBack = -DIV_MOV;
        //'a'键, 左移
        else if (key == 30)
            mLeftRight = -DIV_MOV;
        //'d'键, 右移
        else if (key == 32)
            mLeftRight = DIV_MOV;
        //'q'键, 上移
        else if (key == 16)
            mUpDown = DIV_MOV;
        //'e'键， 下移
        else if (key == 18)
            mUpDown = -DIV_MOV;
        
        //'下'键, 上翻
        else if (key == 108)
            rUpDown = -DIV_ROLL;
        //'上'键, 下翻
        else if (key == 103)
            rUpDown = DIV_ROLL;
        //'左'键, 左翻
        else if (key == 105)
            rLeftRight = -DIV_ROLL;
        //'右'键, 右翻
        else if (key == 106)
            rLeftRight = DIV_ROLL;
        //'左Shift'键, 旋转
        else if (key == 42)
            rClock = DIV_ROLL;
        //'空格'键, 旋转
        else if (key == 57)
            rClock = -DIV_ROLL;

        //没有触发键位
        else
            return;

        //旋转和平移相机

        // camera_roll(camera1, rClock, rUpDown, rLeftRight); //空间坐标系
        // camera_roll(camera2, rClock, rUpDown, rLeftRight);
        // camera_roll(camera3, rClock, rUpDown, rLeftRight);

        camera_roll2(camera1, rUpDown, rLeftRight, rClock); //自身坐标系
        camera_roll2(camera2, rUpDown, rLeftRight, rClock);
        camera_roll2(camera3, rUpDown, rLeftRight, rClock);

        // camera_mov(camera1, mFrontBack, mLeftRight, mUpDown); //空间坐标系
        // camera_mov(camera2, mFrontBack, mLeftRight, mUpDown);
        // camera_mov(camera3, mFrontBack, mLeftRight, mUpDown);

        camera_mov2(camera1, mUpDown, mLeftRight, mFrontBack); //自身坐标系
        camera_mov2(camera2, mUpDown, mLeftRight, mFrontBack);
        camera_mov2(camera3, mUpDown, mLeftRight, mFrontBack);
    }
}

int main(int argc, char **argv)
{
#ifdef OUTPUT_FRAME_FOLDER
    //3个相机输出的帧序号起始
    int order1 = 1000, order2 = 2000, order3 = 3000;
#endif

    //初始化相机、模型、引擎
    all_init();

    //引擎启动
    engine_start(engine);

    //给模型1 x轴 的初速度,单位:点/秒
    // sport1->speed[0] = 50;
    //给模型2 z轴 的初速度,单位:点/秒
    // sport2->speed[2] = 50;

    //给模型1 x轴,y轴 的旋转初速度,单位:度/秒
    sport1->speed_angle[0] = 90;
    sport1->speed_angle[1] = 90;
    //给模型2 z轴 的旋转初速度,单位:度/秒
    sport2->speed_angle[1] = 90;

    //注册按键回调
    key_register(NULL, &key_callback);

    //周期抓拍图像,显示到屏幕或保存图片
    while (1)
    {
        delayms(INTERVAL_MS);

        //清空相机照片
        camera_photo_clear(camera1, 0x220000);
        camera_photo_clear(camera2, 0x002200);
        camera_photo_clear(camera3, 0x000022);

        //相机抓拍,照片放在了 camera->photoMap
        engine_photo(engine, camera1);
        engine_photo(engine, camera2);
        engine_photo(engine, camera3);

        //把照片显示到屏幕(由于这里要打开 /dev/fb0 设备,所以需要 sudo 运行)
        fb_output(camera1->photoMap, 0, 0, camera1->width, camera1->height);
        fb_output(camera2->photoMap, camera1->width, 0, camera2->width, camera2->height);
        fb_output(camera3->photoMap, 0, camera1->height, camera3->width, camera3->height);

#ifdef OUTPUT_FRAME_FOLDER
        //输出帧图片
        bmp_create2(order1++, OUTPUT_FRAME_FOLDER, camera1->photoMap, camera1->width, camera1->height, 3);
        bmp_create2(order2++, OUTPUT_FRAME_FOLDER, camera2->photoMap, camera2->width, camera2->height, 3);
        bmp_create2(order3++, OUTPUT_FRAME_FOLDER, camera3->photoMap, camera3->width, camera3->height, 3);
#endif
    }

    //各模块的释放示例

    // 先释放 engine,由于其占用着model指针
    // 添加 model 时返回的 sport 指针属于 engine,会同时被释放掉
    engine_release(&engine);

    // 释放模型
    model_release(&model0);
    model_release(&model1);
    model_release(&model2);

    // 释放相机
    camera_release(&camera1);
    camera_release(&camera2);
    camera_release(&camera3);

    return 0;
}

void all_init(void)
{
    //相机初始位置和转角
    float camera1_xyz[3] = {-100, 0, 0};
    float camera2_xyz[3] = {0, 100, 0};
    float camera3_xyz[3] = {0, 0, 100};
    float camera1_roll_xyz[3] = {0, 0, 0};
    float camera2_roll_xyz[3] = {0, 0, -90};
    float camera3_roll_xyz[3] = {0, 90, 0};
    //模型初始位置和转角
    float model1_xyz[3] = {-15, -30, 0};
    float model2_xyz[3] = {15, 30, 0};
    float model1_roll_xyz[3] = {0, 0, 0};
    float model2_roll_xyz[3] = {0, 0, 0};

    //相机初始化: 300x300窗口,开角90度,近远范围(5,1000)
    camera1 = camera_init(300, 300, 90, 5, 1000, camera1_xyz, camera1_roll_xyz);
    camera2 = camera_init(300, 300, 90, 5, 1000, camera2_xyz, camera2_roll_xyz);
    camera3 = camera_init(300, 300, 90, 5, 1000, camera3_xyz, camera3_roll_xyz);

    //模型0初始化: 空间xyz坐标轴
    model0 = model_init(6,
        50.0, 0.0, 0.0, 0x800000,
        -50.0, 0.0, 0.0, 0x800000,
        0.0, 50.0, 0.0, 0x008000,
        0.0, -50.0, 0.0, 0x008000,
        0.0, 0.0, 50.0, 0x000080,
        0.0, 0.0, -50.0, 0x000080);
    model_net_add(model0, 0x800000, 0, 1, 1); //连线关系
    model_net_add(model0, 0x008000, 2, 1, 3);
    model_net_add(model0, 0x000080, 4, 1, 5);
    model_label_add(model0, 0x800000, 50.0, 0.0, 0.0, "X"); //注释
    model_label_add(model0, 0x008000, 0.0, 50.0, 0.0, "Y");
    model_label_add(model0, 0x000080, 0.0, 0.0, 50.0, "Z");

    //模型1初始化: 长方体 (注意!! 变长参数中的float类型一定要0.0格式)
    model1 = model_init(8,
        10.0, 20.0, 30.0, 0xFFFFFF,
        10.0, -20.0, 30.0, 0xFFFFFF,
        10.0, -20.0, -30.0, 0xFFFFFF,
        10.0, 20.0, -30.0, 0xFFFFFF,
        -10.0, 20.0, -30.0, 0xFFFFFF,
        -10.0, 20.0, 30.0, 0xFFFFFF,
        -10.0, -20.0, 30.0, 0xFFFFFF,
        -10.0, -20.0, -30.0, 0xFFFFFF);
    model_net_add(model1, 0xFFFFFF, 0, 3, 1, 3, 5); //连线关系
    model_net_add(model1, 0xFFFFFF, 1, 2, 2, 6);
    model_net_add(model1, 0xFFFFFF, 2, 2, 3, 7);
    model_net_add(model1, 0xFFFFFF, 3, 1, 4);
    model_net_add(model1, 0xFFFFFF, 4, 2, 5, 7);
    model_net_add(model1, 0xFFFFFF, 5, 1, 6);
    model_net_add(model1, 0xFFFFFF, 6, 1, 7);
    model_label_add(model1, 0xFF0000, 10.0, 20.0, 30.0, "A"); //注释
    model_label_add(model1, 0x00FF00, 10.0, -20.0, 30.0, "B");
    model_label_add(model1, 0x0000FF, 10.0, -20.0, -30.0, "C");
    model_label_add(model1, 0xFFFF00, 10.0, 20.0, -30.0, "D");
    model_label_add(model1, 0x00FFFF, -10.0, 20.0, -30.0, "E");
    model_label_add(model1, 0xFF00FF, -10.0, 20.0, 30.0, "F");
    model_label_add(model1, 0xFF8000, -10.0, -20.0, 30.0, "G");
    model_label_add(model1, 0x00FF80, -10.0, -20.0, -30.0, "H");

    //模型2初始化: 三棱柱 (注意!! 变长参数中的float类型一定要0.0格式)
    model2 = model_init(6,
        20.0, 0.0, 20.0, 0xFF0000,
        -10.0, 17.3, 20.0, 0x00FF00,
        -10.0, -17.3, 20.0, 0x0000FF,
        -10.0, -17.3, -20.0, 0xFFFF00,
        20.0, 0.0, -20.0, 0x00FFFF,
        -10.0, 17.3, -20.0, 0xFF00FF);
    model_net_add(model2, 0xFFFFFF, 0, 3, 1, 2, 4); //连线关系
    model_net_add(model2, 0xFFFFFF, 1, 2, 2, 5);
    model_net_add(model2, 0xFFFFFF, 2, 1, 3);
    model_net_add(model2, 0xFFFFFF, 3, 2, 4, 5);
    model_net_add(model2, 0xFFFFFF, 4, 1, 5);
    model_label_add(model2, 0xFF0000, 20.0, 0.0, 20.0, "A"); //注释
    model_label_add(model2, 0x00FF00, -10.0, 17.3, 20.0, "B");
    model_label_add(model2, 0x0000FF, -10.0, -17.3, 20.0, "C");
    model_label_add(model2, 0xFFFF00, -10.0, -17.3, -20.0, "D");
    model_label_add(model2, 0x00FFFF, 20.0, 0.0, -20.0, "E");
    model_label_add(model2, 0xFF00FF, -10.0, 17.3, -20.0, "F");

    //引擎初始化: 建立 250 x 250 x 250 三维空间
    engine = engine_init(ENGINE_INTERVAL_MS, 250, 250, 250);

    //往引擎添加模型,得到模型的运动控制器
    sport0 = engine_model_add(engine, model0, NULL, NULL); //这是xyz坐标轴,放到空间原点处
    sport1 = engine_model_add(engine, model1, model1_xyz, model1_roll_xyz);
    sport2 = engine_model_add(engine, model2, model2_xyz, model2_roll_xyz);
}
