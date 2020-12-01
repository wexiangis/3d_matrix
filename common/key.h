/*
 *  xx
 */
#ifndef _KEY_H_
#define _KEY_H_

/*
 *  如何确认键盘在"/dev/input/event"几号?
 *  通过"cat /proc/bus/input/devices"可以看到"keyboard"所在的event号
 */
#define INPUT_DEV_PATH "/dev/input/event1"

/*
 *  按键回调注册
 *  参数:
 *      obj: 用户私有指针,会在互调的时候传回给用户
 *      callback: 回调函数原型 void callback(void *obj, int key, int type)
 *  回调函数参数:
 *      obj: 前面传入的用户私有指针
 *      key: 键位,可以看头文件<linux/input.h>中的定义,或者先测试打印一遍就知道哪个按键对哪个值了
 *      type: 按键状态,0/松开时,1/按下时,2/一直按住(会反复触发回调,间隔多久我也忘了)
 *  返回: 0/成功 -1/失败,找不到设备或者没有sudo运行
 */
int key_register(void *obj, void (*callback)(void *, int, int));

#endif