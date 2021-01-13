/*
 *  2d作图的工具,用于3d引擎输出2d画面
 * 
 *  address: https://github.com/wexiangis/3d_matrix
 *  address2: https://gitee.com/wexiangis/matrix_3d
 * 
 *  update: 2020.11.24 - wexiangis - 初版
 */
#ifndef _2D_DRAW_H_
#define _2D_DRAW_H_

#include <stdint.h>

void _2d_draw_dot(
    uint8_t *rgbMap,
    uint32_t width, uint32_t height,
    int32_t *xy,
    uint32_t argbColor, uint32_t size);

void _2d_draw_line(
    uint8_t *rgbMap,
    uint32_t width, uint32_t height,
    int32_t *xyStart, int32_t *xyEnd,
    uint32_t argbColor, uint32_t size);

#endif
