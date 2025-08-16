#include <iostream>      // 标准输入输出流，用于 cout 等
#include <fstream>       // 文件流，用于从文件读入数据
#include <unistd.h>      // 包含 usleep 函数，用于在可视化中暂停程序
#include <pangolin/pangolin.h> // Pangolin 可视化库，用于绘制轨迹
#include <sophus/se3.hpp>    // Sophus 库，用于处理 SE3d 类型（三维空间的刚体变换）

using namespace Sophus; // 使用 Sophus 命名空间，可以直接使用 SE3d
using namespace std;    // 使用标准命名空间

string groundtruth_file = "../groundtruth.txt"; // 真值轨迹文件路径
string estimated_file = "../estimated.txt";     // 估计轨迹文件路径

// 定义一个类型别名，表示由 Sophus::SE3d 组成的 vector，并指定了内存对齐分配器
typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

// 函数声明：用于绘制两条轨迹
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti);

// 函数声明：从文件中读取轨迹数据
TrajectoryType ReadTrajectory(const string &path);

int main(int argc, char **argv) {
    // 从文件中读取真值轨迹和估计轨迹
    TrajectoryType groundtruth = ReadTrajectory(groundtruth_file);
    TrajectoryType estimated = ReadTrajectory(estimated_file);
    cout<< "Read groundtruth trajectory: " << groundtruth.size() << endl;
    cout<< "Read estimated trajectory: " << estimated.size() << endl;
    // 断言确保两个轨迹都非空且大小相同
    assert(!groundtruth.empty() && !estimated.empty());
    assert(groundtruth.size() == estimated.size());

    // 计算均方根误差 (RMSE)
    double rmse = 0;
    // 遍历估计轨迹的每一个位姿
    for (size_t i = 0; i < estimated.size(); i++) {
        // 获取第 i 个真值位姿和估计位姿
        Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
        // 计算两个位姿之间的误差：
        // p2.inverse() * p1 得到从 p2 到 p1 的变换
        // .log() 将 SE3d 变换转换为李代数 se3
        // .norm() 计算李代数向量的范数，即误差值
        double error = (p2.inverse() * p1).log().norm();
        // 将误差平方后累加
        rmse += error * error;
    }
    // 计算均值
    rmse = rmse / double(estimated.size());
    // 取平方根得到 RMSE
    rmse = sqrt(rmse);
    cout << "RMSE = " << rmse << endl;

    // 调用函数绘制轨迹
    DrawTrajectory(groundtruth, estimated);
    return 0;
}

// 实现从文件中读取轨迹数据的函数
TrajectoryType ReadTrajectory(const string &path) {
    // 打开文件流
    ifstream fin(path);
    // 定义一个 TrajectoryType 类型的变量用于存储轨迹数据
    TrajectoryType trajectory;
    // 检查文件是否成功打开
    if (!fin) {
        cerr << "trajectory " << path << " not found." << endl;
        return trajectory;
    }

    double time, tx, ty, tz, qx, qy, qz, qw;
    while (fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
        Sophus::SE3d p1(
            Eigen::Quaterniond(qw, qx, qy, qz),
            Eigen::Vector3d(tx, ty, tz)
        );
        trajectory.push_back(p1);
    }
    // // 循环读取文件直到文件末尾 (end of file)
    // while (!fin.eof()) {
    //     // 定义变量来存储时间、平移向量 (tx, ty, tz) 和四元数 (qx, qy, qz, qw)
    //     double time, tx, ty, tz, qx, qy, qz, qw;
    //     // 从文件流中读取数据
    //     fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    //     // 使用读取的数据创建 Sophus::SE3d 对象：
    //     // Eigen::Quaterniond(qw, qx, qy, qz) 创建四元数，注意顺序为 (w, x, y, z)
    //     // Eigen::Vector3d(tx, ty, tz) 创建平移向量
    //     // 两者共同创建位姿对象 p1
    //     Sophus::SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
    //     // 将创建的位姿对象添加到轨迹向量的末尾
    //     trajectory.push_back(p1);
    // }
    return trajectory;
}



// 实现绘制轨迹的函数
void DrawTrajectory(const TrajectoryType &gt, const TrajectoryType &esti) {
    // 创建一个名为 "Trajectory Viewer" 的 Pangolin 窗口，大小为 1024x768 像素
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    // 启用深度测试，以正确处理物体重叠时的遮挡关系
    glEnable(GL_DEPTH_TEST);
    // 启用混合，用于处理透明效果
    glEnable(GL_BLEND);
    // 设置混合函数，这里是用于实现半透明效果
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 定义相机的 OpenGL 渲染状态
    pangolin::OpenGlRenderState s_cam(
        // 设置投影矩阵：宽度，高度，焦距 (fx, fy)，光心 (cx, cy)，近平面，远平面
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        // 设置模型视图矩阵：相机位置、看向点、上向量
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // 创建一个显示区域
    pangolin::View &d_cam = pangolin::CreateDisplay()
        // 设置显示区域的边界
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        // 设置 3D 交互处理器，允许用户通过鼠标操作相机
        .SetHandler(new pangolin::Handler3D(s_cam));


    // 主循环：只要窗口不关闭，就一直执行
    while (pangolin::ShouldQuit() == false) {
        // 清除颜色和深度缓冲区
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 激活 3D 视图
        d_cam.Activate(s_cam);
        // 设置背景颜色为白色
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        // 设置线条宽度为 2
        glLineWidth(2);
        // 绘制真值轨迹 (蓝色)
        for (size_t i = 0; i < gt.size() - 1; i++) {
            // 设置线条颜色为蓝色 (RGB: 0.0, 0.0, 1.0)
            glColor3f(0.0f, 0.0f, 1.0f);
            // 开始绘制线段
            glBegin(GL_LINES);
            // 获取相邻的两个位姿
            auto p1 = gt[i], p2 = gt[i + 1];
            // 设置线段的起点，使用位姿的平移向量作为坐标
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            // 设置线段的终点
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            // 结束绘制
            glEnd();
        }

        // 绘制估计轨迹 (红色)
        for (size_t i = 0; i < esti.size() - 1; i++) {
            // 设置线条颜色为红色 (RGB: 1.0, 0.0, 0.0)
            glColor3f(1.0f, 0.0f, 0.0f);
            // 开始绘制线段
            glBegin(GL_LINES);
            // 获取相邻的两个位姿
            auto p1 = esti[i], p2 = esti[i + 1];
            // 设置线段的起点
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            // 设置线段的终点
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            // 结束绘制
            glEnd();
        }
        // 结束本帧绘制，交换缓冲区，显示绘制结果
        pangolin::FinishFrame();
        // 暂停 5 毫秒，以控制帧率
        usleep(5000);
    }
}

