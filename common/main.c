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
//3个模型
static _3D_Model *model1, *model2, *model3;
//2个相机(两个视图窗口)
static _3D_Camera *camera1, *camera2;
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


    while (1)
    {
        delayms(INTERVAL_MS);

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
    //相机1的初始位置和角度
    double camera1_xyz[] = {0, 0, 0};
    double camera1_roll_xyz[] = {0, 0, 0};
    //相机2的初始位置和角度
    double camera2_xyz[] = {0, 0, 0};
    double camera2_roll_xyz[] = {0, 0, 0};

    //相机初始化
    camera1 = _3d_camera_init(320, 320, 1.57, 5, 1000, camera1_xyz, camera1_roll_xyz);
    camera2 = _3d_camera_init(320, 320, 1.57, 5, 1000, camera2_xyz, camera2_roll_xyz);

    //模型初始化
    model1 = _3d_model_init(8,
        10.00, 20.00, 30.00, 0xFF0000,
        -10.00, 20.00, 30.00, 0xFF0000,
        10.00, 20.00, 30.00, 0xFF0000,
        -10.00, 20.00, 30.00, 0xFF0000,
        10.00, 20.00, 30.00, 0xFF0000,
        -10.00, 20.00, 30.00, 0xFF0000,
        10.00, 20.00, 30.00, 0xFF0000,
        -10.00, 20.00, 30.00, 0xFF0000);

    //引擎初始化
    ;
}
