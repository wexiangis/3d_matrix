#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <linux/input.h>

#include "key.h"

//支持最大同按下的按键数量
#define KEY_COMBINATION 10

//长按按键时会反复触发事件,这里设置时间间隔ms,为0时不触发长按事件
#define KEY_HOLD_EVENT_INTERVALMS 50

typedef struct
{
    int fd;
    void *obj;
    void (*callback)(void *, int, int);
} Key_Struct;

typedef struct
{
    void *obj;
    int key;
    int type;
    void (*callback)(void *, int, int);
    //数组指针
    int *combin;
} Key_Param;

//延时工具
#include <sys/time.h>
void key_delayms(unsigned int ms)
{
    struct timeval tv;
    tv.tv_sec = ms / 1000;
    tv.tv_usec = ms % 1000 * 1000;
    select(0, NULL, NULL, NULL, &tv);
}

//抛线程工具
static void throwOut_thread(void *obj, void (*callback)(void *))
{
    pthread_t th;
    pthread_attr_t attr;
    //attr init
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED); //禁用线程同步, 线程运行结束后自动释放
    //抛出线程
    pthread_create(&th, &attr, (void *)callback, (void *)obj);
    //attr destroy
    pthread_attr_destroy(&attr);
}

static void key_callback(void *argv)
{
    Key_Param *kp = (Key_Param *)argv;
    do {
        //按键事件回调
        if (kp->callback)
            kp->callback(kp->obj, kp->key, kp->type);
        //该值为0时,不触发长按事件
        if (KEY_HOLD_EVENT_INTERVALMS > 0) {
            //按下事件,经过2倍延时后切换为长按事件
            if (kp->type == 1) {
                key_delayms(KEY_HOLD_EVENT_INTERVALMS * 2);
                kp->type = 2;
            }
            //长按事件
            else if (kp->type == 2)
                key_delayms(KEY_HOLD_EVENT_INTERVALMS);
        }
        //周期触发长按事件
    } while (kp->type == 2 && kp->combin[0] == kp->key);
    free(kp);
}

//数组元素的设置和清除,返回位置
static int _arrayAdd(int *array, int len, int value)
{
    int i;
    for (i = 0; i < len; i++) {
        if (array[i] == 0) {
            array[i] = value;
            return i;
        }
    }
    return 0;
}
static int _arrayClear(int *array, int len, int value)
{
    int i;
    for (i = 0; i < len; i++) {
        if (array[i] == value) {
            array[i] = 0;
            return i;
        }
    }
    return 0;
}

static void key_thread(void *argv)
{
    Key_Struct *ks = (Key_Struct *)argv;
    Key_Param *kp;
    struct input_event key_info;
    int order;
    int combin[KEY_COMBINATION] = {0};
    while (1)
    {
        //阻塞读
        if (read(ks->fd, &key_info, sizeof(struct input_event)) > 0)
        {
            //这是按键类事件(触屏类事件也是这样读的)
            if (key_info.type == EV_KEY)
            {
                if (!ks->callback || key_info.value > 1)
                    continue;
                //按键按下时注册到数组,释放时清除
                order = 0;
                if (key_info.value == 1)
                    order = _arrayAdd(combin, KEY_COMBINATION, key_info.code);
                else
                    _arrayClear(combin, KEY_COMBINATION, key_info.code);
                //参数准备
                kp = (Key_Param *)calloc(1, sizeof(Key_Param));
                kp->obj = ks->obj;
                kp->key = key_info.code;   //键位
                kp->type = key_info.value; //键值
                kp->callback = ks->callback;
                kp->combin = &combin[order];
                //抛线程,在异步线程中触发用户回调函数
                throwOut_thread(kp, &key_callback);
            }
        }
    }
}

/*
 *  按键回调注册
 *  参数:
 *      obj: 用户私有指针,会在互调的时候传回给用户
 *      callback: 回调函数原型 void callback(void *obj, int key, int type)
 *  回调函数参数:
 *      obj: 前面传入的用户私有指针
 *      key: 键位值,可以看<linux/input.h>中的定义,或者先测试打印一遍就知道哪个按键对哪个值了
 *      type: 按键状态,0/松开时,1/按下时,2/一直按住(会反复触发回调)
 *  返回: 0/成功 -1/失败,找不到设备或者没有sudo运行
 */
int key_register(void *obj, void (*callback)(void *, int, int))
{
    Key_Struct *ks;
    //关键参数检查
    if (!callback)
        return -1;
    //只读打开键盘所在input设备
    int fd = open(INPUT_DEV_PATH, O_RDONLY);
    if (fd < 1)
    {
        printf("key_register: open %s failed\r\n", INPUT_DEV_PATH);
        return -1;
    }
    //参数备份,抛线程检测按键
    ks = (Key_Struct *)calloc(1, sizeof(Key_Struct));
    ks->fd = fd;
    ks->obj = obj;
    ks->callback = callback;
    throwOut_thread(ks, &key_thread);
    return 0;
}
