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

//使能输出帧图片
#define OUTPUT_FRAME_FOLDER "./frameOutput"

//main函数刷新间隔
#define INTERVAL_MS 200
//引擎计算间隔
#define ENGINE_INTERVAL_MS 200
//平移和旋转最小分度格
#define DIV_MOV  10 //单位:点
#define ROLL_DIV 10 //单位:度

//3个相机(三视图)
static _3D_Camera *camera1, *camera2, *camera3;
//3个模型
static _3D_Model *model0, *model1, *model2;
//3个远动控制器(往引擎添加模型后返回的运动控制指针)
static _3D_Sport *sport0, *sport1, *sport2;
//引擎
static _3D_Engine *engine;

//把大陀的初始化代码放到main函数后面,方便快速查看
void engine_init(void);

int main(int argc, char **argv)
{
#ifdef OUTPUT_FRAME_FOLDER
    //3个相机输出的帧序号起始
    int order1 = 1000, order2 = 2000, order3 = 3000;
#endif

    //终端输入
    char input[16];
    int fd;
    float mov_xyz[3];
    float roll_xyz[3];
    //打开终端
    if (argc > 1)
        fd = open(argv[1], O_RDONLY);
    else
        fd = open("/dev/console", O_RDONLY);
    //非阻塞设置
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

    //初始化引擎
    engine_init();

    //引擎启动
    _3d_engine_start(engine);

    //给模型1 x轴 的初速度,单位:点/秒
    // sport1->speed[0] = 50;
    //给模型2 z轴 的初速度,单位:点/秒
    // sport2->speed[2] = 50;

    //给模型1 x轴 的旋转初速度,单位:度/秒
    sport1->speed_angle[0] = 90;
    //给模型2 z轴 的旋转初速度,单位:度/秒
    sport2->speed_angle[2] = 90;

    while (1)
    {
        delayms(INTERVAL_MS);

        //清空相机照片
        _3d_camera_photo_clear(camera1, 0x220000);
        _3d_camera_photo_clear(camera2, 0x002200);
        _3d_camera_photo_clear(camera3, 0x000022);

        //相机抓拍,照片放在了 camera->photoMap
        _3d_engine_photo(engine, camera1);
        _3d_engine_photo(engine, camera2);
        _3d_engine_photo(engine, camera3);

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

        //读取终端输入
        memset(input, 0, sizeof(input));
        if (read(fd, input, sizeof(input)) > 0)
        // if (scanf("%s", input) > 0)
        {
            memset(mov_xyz, 0, sizeof(float) * 3);
            memset(roll_xyz, 0, sizeof(float) * 3);

            //平移
            if (input[0] == '1')
                mov_xyz[0] = DIV_MOV;
            else if (input[0] == '3')
                mov_xyz[0] = -DIV_MOV;
            else if (input[0] == '2')
                mov_xyz[2] = DIV_MOV;
            else if (input[0] == 'w')
                mov_xyz[2] = -DIV_MOV;
            else if (input[0] == 'e')
                mov_xyz[1] = DIV_MOV;
            else if (input[0] == 'q')
                mov_xyz[1] = -DIV_MOV;
            
            //旋转
            if (input[0] == 's')
                roll_xyz[1] = ROLL_DIV;
            else if (input[0] == 'x')
                roll_xyz[1] = -ROLL_DIV;
            else if (input[0] == 'z')
                roll_xyz[2] = ROLL_DIV;
            else if (input[0] == 'c')
                roll_xyz[2] = -ROLL_DIV;
            else if (input[0] == 'a')
                roll_xyz[0] = ROLL_DIV;
            else if (input[0] == 'd')
                roll_xyz[0] = -ROLL_DIV;

            else if (input[0] == 'R')
            {
                _3d_camera_reset(camera1);
                _3d_camera_reset(camera2);
                _3d_camera_reset(camera3);
            }

            _3d_camera_mov(camera1, mov_xyz[0], mov_xyz[1], mov_xyz[2]);
            _3d_camera_mov(camera2, mov_xyz[0], mov_xyz[1], mov_xyz[2]);
            _3d_camera_mov(camera3, mov_xyz[0], mov_xyz[1], mov_xyz[2]);

            _3d_camera_roll(camera1, roll_xyz[0], roll_xyz[1], roll_xyz[2]);
            _3d_camera_roll(camera2, roll_xyz[0], roll_xyz[1], roll_xyz[2]);
            _3d_camera_roll(camera3, roll_xyz[0], roll_xyz[1], roll_xyz[2]);
        }
    }

    return 0;
}

void engine_init(void)
{
    //相机初始位置和转角
    float camera1_xyz[3] = {-120, 0, 0};
    float camera2_xyz[3] = {0, 120, 0};
    float camera3_xyz[3] = {0, 0, 120};
    float camera1_roll_xyz[3] = {0, 0, 0};
    float camera2_roll_xyz[3] = {0, 0, 90};
    float camera3_roll_xyz[3] = {0, -90, 0};
    //模型初始位置和转角
    float model1_xyz[3] = {-15, -30, 0};
    float model2_xyz[3] = {15, 30, 0};
    float model1_roll_xyz[3] = {0, 0, 0};
    float model2_roll_xyz[3] = {0, 0, 0};

    //相机初始化: 300x300窗口,开角90度,近远范围(5,1000)
    camera1 = _3d_camera_init(300, 300, 90, 5, 1000, camera1_xyz, camera1_roll_xyz);
    camera2 = _3d_camera_init(300, 300, 90, 5, 1000, camera2_xyz, camera2_roll_xyz);
    camera3 = _3d_camera_init(300, 300, 90, 5, 1000, camera3_xyz, camera3_roll_xyz);

    //模型0初始化: 空间xyz坐标轴
    model0 = _3d_model_init(6,
        100.0, 0.0, 0.0, 0x800000,
        -100.0, 0.0, 0.0, 0x800000,
        0.0, 100.0, 0.0, 0x008000,
        0.0, -100.0, 0.0, 0x008000,
        0.0, 0.0, 100.0, 0x000080,
        0.0, 0.0, -100.0, 0x000080);
    _3d_model_net_add(model0, 0x800000, 0, 1, 1); //连线关系
    _3d_model_net_add(model0, 0x008000, 2, 1, 3);
    _3d_model_net_add(model0, 0x000080, 4, 1, 5);
    _3d_model_label_add(model0, 0x800000, 100.0, 0.0, 0.0, "X"); //注释
    _3d_model_label_add(model0, 0x008000, 0.0, 100.0, 0.0, "Y");
    _3d_model_label_add(model0, 0x000080, 0.0, 0.0, 100.0, "Z");

    //模型1初始化: 长方体 (注意!! 变长参数中的float类型一定要0.0格式)
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
    _3d_model_label_add(model1, 0xFF0000, 10.0, 20.0, 30.0, "A"); //注释
    _3d_model_label_add(model1, 0x00FF00, 10.0, -20.0, 30.0, "B");
    _3d_model_label_add(model1, 0x0000FF, 10.0, -20.0, -30.0, "C");
    _3d_model_label_add(model1, 0xFFFF00, 10.0, 20.0, -30.0, "D");
    _3d_model_label_add(model1, 0x00FFFF, -10.0, 20.0, -30.0, "E");
    _3d_model_label_add(model1, 0xFF00FF, -10.0, 20.0, 30.0, "F");
    _3d_model_label_add(model1, 0xFF8000, -10.0, -20.0, 30.0, "G");
    _3d_model_label_add(model1, 0x00FF80, -10.0, -20.0, -30.0, "H");

    //模型2初始化: 三棱柱 (注意!! 变长参数中的float类型一定要0.0格式)
    model2 = _3d_model_init(6,
        20.0, 0.0, 20.0, 0xFF0000,
        -10.0, 17.3, 20.0, 0x00FF00,
        -10.0, -17.3, 20.0, 0x0000FF,
        -10.0, -17.3, -20.0, 0xFFFF00,
        20.0, 0.0, -20.0, 0x00FFFF,
        -10.0, 17.3, -20.0, 0xFF00FF);
    _3d_model_net_add(model2, 0xFFFFFF, 0, 3, 1, 2, 4); //连线关系
    _3d_model_net_add(model2, 0xFFFFFF, 1, 2, 2, 5);
    _3d_model_net_add(model2, 0xFFFFFF, 2, 1, 3);
    _3d_model_net_add(model2, 0xFFFFFF, 3, 2, 4, 5);
    _3d_model_net_add(model2, 0xFFFFFF, 4, 1, 5);
    _3d_model_label_add(model2, 0xFF0000, 20.0, 0.0, 20.0, "A"); //注释
    _3d_model_label_add(model2, 0x00FF00, -10.0, 17.3, 20.0, "B");
    _3d_model_label_add(model2, 0x0000FF, -10.0, -17.3, 20.0, "C");
    _3d_model_label_add(model2, 0xFFFF00, -10.0, -17.3, -20.0, "D");
    _3d_model_label_add(model2, 0x00FFFF, 20.0, 0.0, -20.0, "E");
    _3d_model_label_add(model2, 0xFF00FF, -10.0, 17.3, -20.0, "F");

    //引擎初始化: 建立 250 x 250 x 250 空间
    engine = _3d_engine_init(ENGINE_INTERVAL_MS, 250, 250, 250);

    //往引擎添加模型,得到模型的运动控制器
    sport0 = _3d_engine_model_add(engine, model0, NULL, NULL); //默认位置空间原点处
    sport1 = _3d_engine_model_add(engine, model1, model1_xyz, model1_roll_xyz);
    sport2 = _3d_engine_model_add(engine, model2, model2_xyz, model2_roll_xyz);
}
