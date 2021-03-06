## 效果演示

* 演示一
  * [用3个相机作 正视图、右视图、俯视图](https://gitee.com/wexiangis/matrix_3d "跳转gitee查看演示")

![GIF](/效果演示/正视图.gif) ![GIF](/效果演示/右视图.gif)
![GIF](/效果演示/俯视图.gif)

* 演示二
  * [第一人称走动视角下的三视图](https://gitee.com/wexiangis/matrix_3d "跳转gitee查看演示")

![GIF](/效果演示/正视图2.gif) ![GIF](/效果演示/右视图2.gif)
![GIF](/效果演示/俯视图2.gif)

## 编译

* make

## 运行(ubuntu虚拟机)

* 先 ctrl + alt + F1 进入命令行模式, 用cd指令进入到工程所在目录

* 再 sudo ./out

* 运行中使用键位 [ w、s、a、d、q、e ] 来移动相机

* 运行中使用键位 [ 上、下、左、右、Lshift、空格 ] 来旋转相机

* 运行中使用键位 r 复位相机状态

## 文件夹说明

* 3d: 包含了模型(model)、引擎(engine)、相机(camera)以及矩阵运算(math)

* common: 包含其它工具,如延时、bmp图片读写等

* frameOutput: 相机输出的帧图像保存在此

* obj: 缓存编译生成的.o文件

* ui: 基于framebuffer封装的rgb图像输出方法

## 基本原理: 模型(model)、引擎(engine)、相机(camera)

* 模型: 由三维坐标点及其连线组成的一坨数据.

* 引擎: 提供虚拟空间,然后把模型放置于此,由引擎来记录各个模型当前的位置、角度及运动参数; 在时间推进过程中,根据模型运动参数不断计算并更新着模型的位置、角度信息.

* 相机: 也就是电脑屏幕,用来窥探虚拟空间中各个模型的状态; 通过透视投影矩阵把三维坐标点投影到二维平面; 相机可以运动到虚拟空间中的任意位置、角度进行拍照.
