## 线性系统与卡尔曼滤波 (KF)
### 1. 预测:
$\check{\boldsymbol{x}}_k = \boldsymbol{A}_k \hat{\boldsymbol{x}}_{k - 1} + \boldsymbol{u}_k, \quad \check{\boldsymbol{P}}_k = \boldsymbol{A}_k \hat{\boldsymbol{P}}_{k - 1} \boldsymbol{A}_k^{\mathrm{T}} + \boldsymbol{R}. $

### 2. 更新: 
先计算 $\boldsymbol{K}$, 它又称为卡尔曼增益。  

$\boldsymbol{K} = \check{\boldsymbol{P}}_k \boldsymbol{C}_k^{\mathrm{T}} (\boldsymbol{C}_k \check{\boldsymbol{P}}_k \boldsymbol{C}_k^{\mathrm{T}} + \boldsymbol{Q}_k)^{-1}.$

然后计算后验概率的分布。

$\hat{\boldsymbol{x}}_k = \check{\boldsymbol{x}}_k + \boldsymbol{K} (z_k - \boldsymbol{C}_k \check{\boldsymbol{x}}_k)$  

$\hat{\boldsymbol{P}}_k = (\boldsymbol{I} - \boldsymbol{K} \boldsymbol{C}_k) \check{\boldsymbol{P}}_k.$


## 非线性系统与扩展卡尔曼滤波 (EKF)
我们希望把卡尔曼滤波器的结果拓展到非线性系统中，称为扩展卡尔曼滤波器。通常的做法是，在某个点附近考虑运动方程及观测方程的一阶泰勒展开，只保留一阶项，即线性的部分，然后按照线性系统进行推导。  
令$k - 1$时刻的均值与协方差矩阵为$\hat{\boldsymbol{x}}_{k - 1}, \hat{\boldsymbol{P}}_{k - 1}$。在k时刻，我们把运动方程和观测方程在$\hat{\boldsymbol{x}}_{k - 1}, \hat{\boldsymbol{P}}_{k - 1}$处进行**线性化**（相当于一阶泰勒展开），有    

$\boldsymbol{x}_k \approx f(\hat{\boldsymbol{x}}_{k - 1}, \boldsymbol{u}_k) + \left.\frac{\partial f}{\partial \boldsymbol{x}_{k - 1}}\right|_{\hat{\boldsymbol{x}}_{k - 1}} (\boldsymbol{x}_{k - 1} - \hat{\boldsymbol{x}}_{k - 1}) + \boldsymbol{w}_k$  
记这里的偏导数为
$\boldsymbol{F} = \left. \frac{\partial f}{\partial \boldsymbol{x}_{k - 1}} \right|_{\hat{\boldsymbol{x}}_{k - 1}}$

同样，对于观测方程，亦有  

$z_k \approx h(\check{\boldsymbol{x}}_k) + \left. \frac{\partial h}{\partial \boldsymbol{x}_k} \right|_{\check{\boldsymbol{x}}_k} (\boldsymbol{x}_k - \check{\boldsymbol{x}}_k) + \boldsymbol{n}_k$

记这里的偏导数为
$\boldsymbol{H} = \left. \frac{\partial h}{\partial \boldsymbol{x}_k} \right|_{\check{\boldsymbol{x}}_k}$

那么，在预测步骤中，根据运动方程有  
$P(\boldsymbol{x}_k | \boldsymbol{x}_0, \boldsymbol{u}_{1:k}, \boldsymbol{z}_{0:k - 1}) = N \left( f(\hat{\boldsymbol{x}}_{k - 1}, \boldsymbol{u}_k), \boldsymbol{F} \hat{\boldsymbol{P}}_{k - 1} \boldsymbol{F}^T + \boldsymbol{R}_k \right)$  
这些推导和卡尔曼滤波是十分相似的。为方便表述，记这里的先验和协方差的均值为

$\check{\boldsymbol{x}}_k = f(\hat{\boldsymbol{x}}_{k - 1}, \boldsymbol{u}_k), \quad \check{\boldsymbol{P}}_k = \boldsymbol{F}\hat{\boldsymbol{P}}_{k - 1}\boldsymbol{F}^\mathrm{T} + \boldsymbol{R}_k.$

然后，考虑在观测中我们有

$P(\boldsymbol{z}_k|\boldsymbol{x}_k) = N(h(\check{\boldsymbol{x}}_k) + \boldsymbol{H}(\boldsymbol{x}_k - \check{\boldsymbol{x}}_k), \boldsymbol{Q}_k).$

最后，根据最开始的贝叶斯展开式，可以推导出 $\boldsymbol{x}_k$ 的后验概率形式。我们略去中间的推导过程，只介绍其结果。读者可以仿照卡尔曼滤波器的方式，推导 EKF 的预测与更新方程。简而言之，我们会先定义一个卡尔曼增益 $\boldsymbol{K}_k$：

$\boldsymbol{K}_k = \check{\boldsymbol{P}}_k\boldsymbol{H}^\mathrm{T}(\boldsymbol{H}\check{\boldsymbol{P}}_k\boldsymbol{H}^\mathrm{T} + \boldsymbol{Q}_k)^{-1}.$ 

在卡尔曼增益的基础上，后验概率的形式为

$\hat{\boldsymbol{x}}_k = \check{\boldsymbol{x}}_k + \boldsymbol{K}_k(\boldsymbol{z}_k - h(\check{\boldsymbol{x}}_k)),\quad \hat{\boldsymbol{P}}_k = (\boldsymbol{I} - \boldsymbol{K}_k\boldsymbol{H})\check{\boldsymbol{P}}_k.$ 

卡尔曼滤波器给出了在线性化之后状态变量分布的变化过程。在线性系统和高斯噪声下，卡尔曼滤波器给出了无偏最优估计。而在 SLAM 这种非线性的情况下，它给出了单次线性近似下的最大后验估计。