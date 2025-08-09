# Easy  SLAM and Robotic Tutorial Python Implement

# 简单的SLAM与机器人教程Python代码实现
**欢迎点小星星收藏本github项目**，如果你想增加内容欢迎fork，然后再在自己项目下修改，然后提交合并请求。或者发私信给[知乎@司南牧](https://www.zhihu.com/people/yuanmuou/activities)，please feel free。
项目作者：[知乎@司南牧](https://www.zhihu.com/people/yuanmuou/activities)
# How to learn SLAM and Robotic

[![Build Status](https://travis-ci.org/AtsushiSakai/PythonRobotics.svg?branch=master)](https://travis-ci.org/varyshare/easy_slam_tutorial)
[![Documentation Status](https://readthedocs.org/projects/pythonrobotics/badge/?version=latest)](https://github.com/varyshare/easy_slam_tutorial/blob/master/README.md)
[![Build status](https://ci.appveyor.com/api/projects/status/sb279kxuv1be391g?svg=true)](https://ci.appveyor.com/project/varyshare/easy_slam_tutorial)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/AtsushiSakai/PythonRobotics.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/AtsushiSakai/PythonRobotics/context:python)
[![CodeFactor](https://www.codefactor.io/repository/github/atsushisakai/pythonrobotics/badge/master)](https://www.codefactor.io/repository/github/varyshare/easy_slam_tutorial/overview/master)
[![tokei](https://tokei.rs/b1/github/varyshare/easy_slam_tutorial/)](https://github.com/varyshare/easy_slam_tutorial)
[![Say Thanks!](https://img.shields.io/badge/Say%20Thanks-!-1EAEDB.svg)](https://www.zhihu.com/people/yuanmuou/activities)

简单的从零开始实现视觉SLAM理论与实践教程，使用Python实现。包括：ORB特征点提取，对极几何，视觉里程计后端优化，实时三维重建地图。Otsu二值化、贝叶斯滤波、快速连通域标记算法,带标记的目标跟踪实践
# A easy SLAM practical tutorial (Python).

## 目录

##  [特征提取](./feature_extract/)
###  [从零开始实现FAST特征点提取算法教程](./feature_extract/从零开始实现FAST特征点提取算法教程.md)
[FAST教程](./feature_extract/从零开始实现FAST特征点提取算法教程.md) [代码](./feature_extract/FAST_feature_extraction.py)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20190722103253875.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3ZhcnlzaGFyZQ==,size_16,color_FFFFFF,t_70)

### 计算机图形学Bresenham画圆法Python实现
[教程](./feature_extract/Bresenham布雷森汉姆算法画圆教程.md) [代码](./feature_extract/bresenham_circle.py)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20190721174903599.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3ZhcnlzaGFyZQ==,size_16,color_FFFFFF,t_70)

### ORB特征提取Python调用OpenCV2实现
ORB特征提取主要是[FAST提取特征点](./feature_extract/从零开始实现FAST特征点提取算法教程.md)+[BRIEF算法](https://blog.csdn.net/varyshare/article/details/96568030)提取周围信息
[代码](./feature_extract/ORB_feature_extract.py)

![1563794343832](./img/orb_效果图.png)

## Otsu二值化算法/大津算法（Otsu's Method Algorithm ）
[教程+Python源代码](./Otsu's_Method_algorithm/如何理解图像处理中的Otsu's 二值化算法（大津算法）Python编程实践.md) 
![在这里插入图片描述](./Otsu's_Method_algorithm/eight_二值化效果.png)
# 高斯模糊代码+教程
[教程+Python源代码](./image_smooth_blur/如何理解高斯模糊原理与具体Python编程实现.ipynb) 
![高斯模糊效果](./image_smooth_blur/高斯模糊效果.png)

### Fast Labeling快速标记连通物体检测与标记目标追踪
[连通组件检测与标记目标追踪代码与教程](./Connected_Components)
[连通组件检测与标记目标追踪真实场景实践](./Connected_Components/real_scene_practice)

# 十分钟如何理解RANSAC算法Python实践

[RANSAC教程与代码](RANSAC)


# 机器人模拟

## [两连杆机械臂机器人(2DOF)模拟](./joint_robot_simulation/)

鼠标选定屏幕上一点，然后求逆解进行运动Python实现代码

代码地址（同一个文件夹）:[two_joint_arm_robot.py](./joint_robot_simulation/two_joint_arm_robot.py)

下面是效果图，**打开你的编辑器跟着我写的代码实践吧，你的star收藏和关注是我持续分享的动力** 。
![在这里插入图片描述](https://img-blog.csdnimg.cn/20190724160425592.gif)





