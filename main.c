#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "3d_engine.h"
#include "delayus.h"

//main函数刷新间隔
#define INTERVAL_MS 100
//引擎计算间隔
#define ENGINE_INTERVAL_MS 100

//2个相机(两个视图窗口)
static _3D_Camera *camera1, *camera2;
//3个模型
static _3D_Model *model0, *model1, *model2;
//3个远动控制器(往引擎添加模型后返回的运动控制指针)
static _3D_Sport *sport0, *sport1, *sport2;
//引擎
static _3D_Engine *engine;

//把大陀的初始化代码放到main函数后面,方便快速查看
void init(void);

int main(int argc, char **argv)
{
    //终端输入
    char input[16];
    int fd;

    //打开终端
    if (argc > 1)
        fd = open(argv[1], O_RDONLY);
    else
        fd = open("/dev/console", O_RDONLY);
    //非阻塞设置
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    //引擎启动
    _3d_engine_start (engine);

    while (1)
    {
        delayms(INTERVAL_MS);

        //读取终端输入
        memset(input, 0, sizeof(input));
        if (read(fd, input, sizeof(input)) > 0)
        {
            ;
        }
    }

    return 0;
}

void init(void)
{
    //相机初始位置和转角
    double camera1_xyz[3] = {0, 0, 0};
    double camera1_roll_xyz[3] = {0, 0, 0};
    double camera2_xyz[3] = {0, 0, 0};
    double camera2_roll_xyz[3] = {0, 0, 0};
    //模型初始位置和转角
    double model1_xyz[3] = {0, 0, 0};
    double model1_roll_xyz[3] = {0, 0, 0};
    double model2_xyz[3] = {0, 0, 0};
    double model2_roll_xyz[3] = {0, 0, 0};

    //相机1,2初始化: 300x300窗口,开角90度,近远范围(5,1000)
    camera1 = _3d_camera_init(300, 300, 90, 5, 1000, camera1_xyz, camera1_roll_xyz);
    camera2 = _3d_camera_init(300, 300, 90, 5, 1000, camera2_xyz, camera2_roll_xyz);

    //模型0初始化: 空间坐标轴 (注意!! 变长参数中的double类型一定要0.0格式)
    model0 = _3d_model_init(6,
        300.0, 0.0, 0.0, 0x800000,
        -300.0, 0.0, 0.0, 0x800000,
        0.0, 300.0, 0.0, 0x008000,
        0.0, -300.0, 0.0, 0x008000,
        0.0, 0.0, 300.0, 0x000080,
        0.0, 0.0, -300.0, 0x000080);
    _3d_model_net_add(model0, 0x800000, 0, 1, 1); //连线关系
    _3d_model_net_add(model0, 0x008000, 2, 1, 3);
    _3d_model_net_add(model0, 0x000080, 4, 1, 5);
    _3d_model_label_add(model0, 0x800000, 300.0, 0.0, 0.0, "X"); //注释
    _3d_model_label_add(model0, 0x008000, 0.0, 300.0, 0.0, "Y");
    _3d_model_label_add(model0, 0x000080, 0.0, 0.0, 300.0, "Z");

    //模型1初始化: 长方体 (注意!! 变长参数中的double类型一定要0.0格式)
    model1 = _3d_model_init(8,
        10.0, 20.0, 30.0, 0xFFFFFF,
        10.0, -20.0, 30.0, 0xFFFFFF,
        10.0, -20.0, -30.0, 0xFFFFFF,
        10.0, 20.0, -30.0, 0xFFFFFF,
        -10.0, 20.0, -30.0, 0xFFFFFF,
        -10.0, 20.0, 30.0, 0xFFFFFF,
        -10.0, -20.0, 30.0, 0xFFFFFF,
        -10.0, -20.0, -30.0, 0xFFFFFF);
    _3d_model_net_add(model1, 0xFFFFFF, 0, 3, 1, 3, 5); //连线关系
    _3d_model_net_add(model1, 0xFFFFFF, 1, 2, 2, 6);
    _3d_model_net_add(model1, 0xFFFFFF, 2, 2, 3, 7);
    _3d_model_net_add(model1, 0xFFFFFF, 3, 1, 4);
    _3d_model_net_add(model1, 0xFFFFFF, 4, 2, 5, 7);
    _3d_model_net_add(model1, 0xFFFFFF, 5, 1, 6);
    _3d_model_net_add(model1, 0xFFFFFF, 6, 1, 7);
    _3d_model_label_add(model1, 0xFFFFFF, 10.0, 20.0, 30.0, "A"); //注释
    _3d_model_label_add(model1, 0xFFFFFF, 10.0, -20.0, 30.0, "B");
    _3d_model_label_add(model1, 0xFFFFFF, 10.0, -20.0, -30.0, "C");
    _3d_model_label_add(model1, 0xFFFFFF, 10.0, 20.0, -30.0, "D");
    _3d_model_label_add(model1, 0xFFFFFF, -10.0, 20.0, -30.0, "E");
    _3d_model_label_add(model1, 0xFFFFFF, -10.0, 20.0, 30.0, "F");
    _3d_model_label_add(model1, 0xFFFFFF, -10.0, -20.0, 30.0, "G");
    _3d_model_label_add(model1, 0xFFFFFF, -10.0, -20.0, -30.0, "H");

    //模型2初始化: 三棱柱 (注意!! 变长参数中的double类型一定要0.0格式)
    model2 = _3d_model_init(6,
        20.0, 0.0, 30, 0xFFFFFF,
        -10.0, 17.3, 30, 0xFFFFFF,
        -10.0, -17.3, 30, 0xFFFFFF,
        -10.0, -17.3, -30, 0xFFFFFF,
        20.0, 0.0, -30, 0xFFFFFF,
        -10.0, 17.3, -30, 0xFFFFFF);
    _3d_model_net_add(model2, 0xFFFFFF, 0, 3, 1, 2, 4); //连线关系
    _3d_model_net_add(model2, 0xFFFFFF, 1, 2, 2, 5);
    _3d_model_net_add(model2, 0xFFFFFF, 2, 1, 3);
    _3d_model_net_add(model2, 0xFFFFFF, 3, 2, 4, 5);
    _3d_model_net_add(model2, 0xFFFFFF, 4, 1, 5);
    _3d_model_label_add(model2, 0xFFFFFF, 20.0, 0.0, 30, "A"); //注释
    _3d_model_label_add(model2, 0xFFFFFF, -10.0, 17.3, 30, "B");
    _3d_model_label_add(model2, 0xFFFFFF, -10.0, -17.3, 30, "C");
    _3d_model_label_add(model2, 0xFFFFFF, -10.0, -17.3, -30, "D");
    _3d_model_label_add(model2, 0xFFFFFF, 20.0, 0.0, -30, "E");
    _3d_model_label_add(model2, 0xFFFFFF, -10.0, 17.3, -30, "F");

    //引擎初始化: 建立1000x1000x1000空间
    engine = _3d_engine_init(ENGINE_INTERVAL_MS, 1000, 1000, 1000);

    //往引擎添加模型,得到模型的运动控制器
    sport0 = _3d_engine_model_add(engine, model0, NULL, NULL); //默认位置空间原点处
    sport1 = _3d_engine_model_add(engine, model1, model1_xyz, model1_roll_xyz);
    sport2 = _3d_engine_model_add(engine, model2, model2_xyz, model2_roll_xyz);
}
