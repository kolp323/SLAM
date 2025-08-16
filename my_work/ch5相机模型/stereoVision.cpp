#include <opencv2/opencv.hpp> // 包含 OpenCV 核心库，用于图像处理
#include <vector>             // 包含 vector 容器
#include <string>             // 包含 string 类型
#include <Eigen/Core>         // 包含 Eigen 核心库，用于矩阵和向量运算
#include <pangolin/pangolin.h> // 包含 Pangolin 可视化库
#include <unistd.h>           // 包含 usleep 函数，用于控制帧率

using namespace std;          // 使用标准命名空间
using namespace Eigen;        // 使用 Eigen 命名空间

// 文件路径
string left_file = "../left.png"; // 左眼图像文件路径
string right_file = "../right.png"; // 右眼图像文件路径

// 在pangolin中画图，已写好，无需调整
// 函数声明：用于在 Pangolin 中显示点云
void showPointCloud(
    const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);

int main(int argc, char **argv) {

    // 相机内参
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    // 双目相机的基线 (baseline)，即左右相机光心之间的距离
    double b = 0.573;

    // 读取图像
    cv::Mat left = cv::imread(left_file, 0); // 读取左眼图像，0表示灰度图
    cv::Mat right = cv::imread(right_file, 0); // 读取右眼图像，0表示灰度图

    // 检查图像是否成功读取
    if (left.empty() || right.empty()) {
        cerr << "Error: Failed to load left or right image." << endl;
        return -1;
    }

    // 创建 Stereo SGBM 算法对象，用于计算视差图
    // SGBM (Semi-Global Block Matching) 是一种经典的双目匹配算法
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0,         // minDisparity：最小视差值，通常设为0
        96,        // numDisparities：视差范围，必须是16的倍数
        9,         // blockSize：匹配块大小
        8 * 9 * 9, // P1：控制视差平滑度的参数，越大越平滑
        32 * 9 * 9, // P2：与P1类似，但用于处理更大的视差变化
        1,         // disp12MaxDiff：左右视差检查的最大差异
        63,        // preFilterCap：预处理滤波器的截断值
        10,        // uniquenessRatio：视差唯一性百分比
        100,       // speckleWindowSize：斑点过滤窗口大小
        32          // speckleRange：斑点过滤视差范围
    );      

    // 定义用于存储视差图的变量
    cv::Mat disparity_sgbm, disparity;
    // 使用Stereo SGBM 算法对象的compute() 计算左右图像的视差图，结果保存在 disparity_sgbm 中
    sgbm->compute(left, right, disparity_sgbm);
    // 将计算出的视差图转换为浮点数类型，并进行缩放，存放到 disparity 中
    // SGBM 算法的视差值是16倍整数，需要除以16得到实际的浮点数视差
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    // 生成点云
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud; // 定义点云容器
    // 预先分配内存，以避免频繁的内存重新分配
    pointcloud.reserve(left.rows * left.cols);

    // 遍历图像中的每一个像素
    // 如果你的机器慢，请把后面的v++和u++改成v+=2, u+=2，可以跳过一些像素以加速
    for (int v = 0; v < left.rows; v++)
        for (int u = 0; u < left.cols; u++) {
            // 获取当前像素的视差值
            // at<float>明确告诉编译器你期望访问的像素数据类型是 float，OpenCV 在运行时会检查你指定的类型是否与 cv::Mat 实际存储的类型匹配。
            // 如果类型不匹配（例如，图像是 CV_8UC1 而你却尝试使用 at<float>），它会抛出异常，而不是导致未定义的行为或崩溃。从而以安全的方式访问浮点数类型的视差值
            if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0) {
                // 如果视差值为无效值（小于等于0）或超出范围，则跳过该像素
                continue;
            }

            // 创建一个 Eigen::Vector4d 向量
            // 前三维 (x, y, z) 暂时为0，第四维存储像素的灰度值（归一化到0-1之间）
            Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0); 

            // 根据双目成像模型，计算当前像素在三维空间中的坐标
            // disparity = fx * b / depth
            // depth = fx * b / disparity
            double depth = fx * b / (disparity.at<float>(v, u));
            
            // 计算归一化坐标 x, y
            double x_norm = (u - cx) / fx;
            double y_norm = (v - cy) / fy;
            
            // 计算三维坐标 (X, Y, Z)
            point[0] = x_norm * depth;
            point[1] = y_norm * depth;
            point[2] = depth;

            // 将计算出的三维点添加到点云容器中
            pointcloud.push_back(point);
        }

    // 显示视差图
    // 将视差图进行归一化，以便于显示，因为 imshow 默认显示 0-255 的图像
    cv::imshow("disparity", disparity / 96.0);
    // 等待按键
    cv::waitKey(0);
    
    // 调用函数画出点云
    showPointCloud(pointcloud);
    
    return 0;
}

// showPointCloud 函数实现
void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

    // 如果点云为空，则打印错误信息并退出
    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    // 创建 Pangolin 窗口
    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    // 启用深度测试
    glEnable(GL_DEPTH_TEST);
    // 启用混合
    glEnable(GL_BLEND);
    // 设置混合函数，用于实现半透明效果
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 定义相机的 OpenGL 渲染状态
    pangolin::OpenGlRenderState s_cam(
        // 设置投影矩阵
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        // 设置模型视图矩阵
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // 创建一个显示区域
    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    // 主循环，只要窗口不关闭就一直执行
    while (pangolin::ShouldQuit() == false) {
        // 清除颜色和深度缓冲区
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 激活 3D 视图
        d_cam.Activate(s_cam);
        // 设置背景颜色为白色
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        // 设置点的大小为 2
        glPointSize(2);
        // 开始绘制点
        glBegin(GL_POINTS);
        // 遍历点云中的每一个点
        for (auto &p: pointcloud) {
            // 使用点的第四个维度（灰度值）作为颜色
            glColor3f(p[3], p[3], p[3]);
            // 设置点的三维坐标
            glVertex3d(p[0], p[1], p[2]);
        }
        // 结束绘制
        glEnd();
        // 结束本帧绘制，交换缓冲区
        pangolin::FinishFrame();
        // 暂停 5 毫秒，以控制帧率
        usleep(5000);
    }
    return;
}