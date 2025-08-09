'''
Features from Accelerated Segment Test (FAST)
Python实现，
从0开始，最原始最简单的FAST特征点提取方法（无金字塔采样）
代码地址：https://github.com/varyshare/easy_slam_tutorial/tree/master/feature_extract
欢迎start这项目
教程地址：https://blog.csdn.net/varyshare/article/details/96430924
代码没有怎么经过优化，所以会有0.8s左右的卡顿
'''
import numpy as np
import matplotlib.pyplot as plt
import cv2 # 用于读取图片
   

# 1. 读取图片(为了简化问题我就直接构造一个数组作为图片)
img = cv2.imread('feature.png',cv2.IMREAD_GRAYSCALE)

# 2. 设置参数
# 设置灰度值相差多大才算较大差异，
# 以及周围点超过多少个高差异点才算当前中心像素点是关键点
h_gray_scale = 20 # 在ORB特征提取中使用的FAST像素差阈值默认是20
k_diff_point = 9 # 超过k_diff_point个差异那就认为是关键点（周围共16个点）
r_pixel = 3 # 获取周围像素所用的圆的半径,单位：像素


# 3. 遍历所有的像素进行检测关键点
def bresenham_circle():
    """
    return: 圆周上所有的点相对圆心的坐标列表。
    即，返回圆心在(0,0)处时圆周上各点的坐标。
    返回一个r_pixel*r_pixel的矩阵，圆周上的点标记为1，其他地方标记为0
    """

    _masked_canvas = np.zeros((2*r_pixel+1,2*r_pixel+1))
    def save(x,y):
        """ 
         把(x,y)加入到结果列表中
         注意：需要把(x,y)变换到数组坐标系（图形学坐标系）
        """
        _masked_canvas[-y+r_pixel,x+r_pixel] = 1
    pass

    # 初始化,画第一个点，从水平最右边那个点开始画
    (x,y) = (r_pixel,0)
    
    """
    从定义来讲就是
    P_k=d1+d2
    d1 = 第1个下一步待选点离圆弧的距离（负数）
    d2 = 第2个下一步待选点离圆弧的距离（正数）
    但是为了提高效率通常使用递推来求P_{k+1}=P_k + 一个数
    """
    P_k = -2*r_pixel + 3
    
    # 迭代的求完1/8圆弧
    while x>=y:
        # 下一步有两个待选点，具体选哪个要看P_k>0 或 <0
        if P_k>=0:# 外侧候选点偏离圆弧更远
            P_k_next =  P_k - 4*x + 4*y + 10
            (x_next,y_next) = (x-1, y+1)
        else:# 内侧候选点偏离圆弧更远
            P_k_next =  P_k + 4*y + 6
            (x_next,y_next) = (x, y+1)
        # 对称法画这对称的8个地方
        save(x,y)
        save(-x,y) 
        save(x,-y) 
        save(-x,-y) 
    
        save(y,x) 
        save(y,-x) 
        save(-y,x) 
        save(-y,-x) 
        # 更新当前坐标和P_k
        (x,y) = (int(x_next),int(y_next))
        P_k = P_k_next
    pass

    return _masked_canvas

# 先bresenham算法算出半径为r_pixel时圆周上的点相对圆心的坐标
masked_canvas = bresenham_circle()
def key_point_test(_row,_col):
    """
    检测像素点(_row,_col)是否是关键点。
    满足关键点只有一个条件：周围16个像素点与中心像素点相比
    差异度较大(>h_gray_scale)的像素点个数超过k_diff_point个
    return: boolean
    """
    # 获取以(_row,_col)为几何中心的7x7正方形区域内的像素值
    surround_points = img[_row-3:_row+3+1,_col-3:_col+3+1]
    
    # 获取圆周上的点与圆心的像素差值的绝对值
    _dist = np.abs((surround_points - img[_row,_col])) * masked_canvas
    
    if (_dist>h_gray_scale).sum()> k_diff_point:
        return True
    else:
        return False


key_point_list = []

for row in range(r_pixel,img.shape[0]-r_pixel):
    for col in range(r_pixel,img.shape[1]-r_pixel):
        
        if key_point_test(row,col):
            key_point_list.append(cv2.KeyPoint(x=row,y=col,_size=1))
        else:
            continue
            
    pass
pass


img_with_keypoints = cv2.drawKeypoints(img,key_point_list,outImage=np.array([]),color=(0,0,255))
cv2.imshow("show key points",img_with_keypoints)
cv2.waitKey(0)

