/*
 *  2d作图的工具,用于3d引擎输出2d画面
 * 
 *  address: https://github.com/wexiangis/3d_matrix
 *  address2: https://gitee.com/wexiangis/matrix_3d
 * 
 *  update: 2020.11.24 - wexiangis - 初版
 */
#include <stdio.h>
#include "2d_draw.h"

void _2d_draw_dot(
    uint8_t *rgbMap,
    uint32_t width, uint32_t height,
    int32_t *xy,
    uint32_t rgbColor, uint32_t size)
{
    uint32_t offset;
    //范围限制
    if (xy[0] < 0 || xy[0] >= width || xy[1] < 0 || xy[1] >= height)
        return;
    if (size < 1)
        return;
    //偏移量
    offset = xy[1] * width * 3 + xy[0] * 3;
    rgbMap[offset + 0] = (rgbColor >> 16) & 0xFF;
    rgbMap[offset + 1] = (rgbColor >> 8) & 0xFF;
    rgbMap[offset + 2] = (rgbColor >> 0) & 0xFF;
    //加大尺寸
    ;
}

void _2d_draw_line(
    uint8_t *rgbMap,
    uint32_t width, uint32_t height,
    int32_t *xyStart, int32_t *xyEnd,
    uint32_t rgbColor, uint32_t size)
{
    int32_t xerr = 0, yerr = 0;
    int32_t delta_x, delta_y;
    int32_t distance;
    int32_t incx, incy, xCount, yCount, count;
    uint32_t offset;
    uint32_t timeout = 0;

    // if(xyStart[0] - xyEnd[0] > 10000 || xyEnd[0] - xyStart[0] > 10000 || 
    //     xyStart[1] - xyEnd[1] > 10000 || xyEnd[1] - xyStart[1] > 10000)
    // {
    //     printf("_2d_draw_line %d/%d --> %d/%d too far away !!\r\n", 
    //         xyStart[0], xyStart[1], xyEnd[0], xyEnd[1]);
    //     return;
    // }

    if (size <= 0)
        return;

    delta_x = xyEnd[0] - xyStart[0]; //计算坐标增量
    delta_y = xyEnd[1] - xyStart[1];
    xCount = xyStart[0];
    yCount = xyStart[1];

    if (delta_x > 0)
        incx = 1; //设置单步方向
    else if (delta_x == 0)
        incx = 0; //垂直线
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; //水平线
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if (delta_x > delta_y)
        distance = delta_x; //选取基本增量坐标轴
    else
        distance = delta_y;

    for (count = 0; count <= distance + 1 && timeout++ < 10000; count++) //画线输出
    {
        //范围检查
        if (xCount >= 0 && xCount < width && yCount >= 0 && yCount < height)
        {
            //偏移量
            offset = yCount * width * 3 + xCount * 3;
            rgbMap[offset + 0] = (rgbColor >> 16) & 0xFF;
            rgbMap[offset + 1] = (rgbColor >> 8) & 0xFF;
            rgbMap[offset + 2] = (rgbColor >> 0) & 0xFF;
            //加大尺寸
            ;
        }

        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            xCount += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            yCount += incy;
        }
    }
}
