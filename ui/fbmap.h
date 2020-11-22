/*
 *  fb矩阵输出
 */
#ifndef _FBMAP_H_
#define _FBMAP_H_

#include <stdint.h>

#define FB_PATH "/dev/fb0"

//屏幕宽高
extern int fb_width, fb_height;

/*
 *  屏幕输出
 *  data: 图像数组,数据长度必须为 width*height*3, RGB格式
 *  offsetX, offsetY: 屏幕起始位置
 *  width, height: 图像宽高
 */
void fb_output(uint8_t *data, uint32_t offsetX, uint32_t offsetY, uint32_t width, uint32_t height);

#endif