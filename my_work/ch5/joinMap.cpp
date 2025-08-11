#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>

#include <boost/format.hpp> // for formating strings
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(){
    // 创建两个动态数组vector 来存储彩色图和深度图
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    // 创建1个动态数组vector 来存储相机位姿
    // Eigen::Isometry3d: 用于表示欧式变换矩阵。在 SLAM 中，它通常用来表示相机的位姿（pose），即相机在世界坐标系中的位置和方向。
    // Eigen::aligned_allocator<...>: 这是一个内存对齐的分配器。由于 Eigen::Isometry3d 类内部使用了 SSE 指令，需要 16 字节的内存对齐才能高效工作。
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;  
    
    ifstream fin("../pose.txt"); //打开一个名为 pose.txt 的文件，并将其与一个输入文件流对象 fin 关联起来。

    if(!fin){
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl;
        return 1;
    }

    // 读取 5 组 (彩色图, 深度图, 位姿) 数据
    for (int i = 0; i < 5; i++)
    {
        // cv::imread() 函数的第二个参数是一个可选的标志，用于指定如何加载图像。以下是几个常用的标志及其作用：
        // cv::IMREAD_UNCHANGED 或 -1：
        // 作用：加载图像时，不改变其原始的通道数和深度。
        // 何时使用：这对于读取深度图像或需要处理带有 Alpha 通道的图像（例如 PNG）至关重要。例如，深度图通常是 16 位的灰度图，如果使用默认模式读取，图像可能会被压缩或转换，导致深度信息丢失。使用 -1 可以确保数据保持其原始格式。
        // cv::IMREAD_GRAYSCALE 或 0：
        // 作用：将图像加载为单通道的灰度图。
        // 何时使用：当你只需要图像的亮度信息，而不需要颜色时。
        // cv::IMREAD_COLOR 或 1：
        // 作用：将图像加载为 3 通道的彩色图像（BGR 格式）。
        // 何时使用：这是最常用的模式，用于处理彩色图像。如果图像是灰度图，它也会被转换为 3 通道
        boost::format fmt("../%s/%d.%s"); // 图像文件格式模板
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); 

        double data[7] = {0};
        // auto& d：d 是一个引用变量，它会依次引用数组 data 中的每一个元素。
        for(auto& d : data)  // 从 pose.txt 文件读取 7 个数（当前帧的位姿）。
            fin >> d; // 这个操作符会从文件流 fin 中读取一个 double 类型的数值，并将其赋值给变量 d 所引用的数组元素。
        
        // pose.txt 中的数据存储格式可能是 [tx, ty, tz, x, y, z, w]  
        //平移向量 (tx,ty,tz) // n*sin(θ/2) (x,y,z) // cos(θ/2) w
        // 标准的数学四元数通常表示为 q= w+ xi + yj + zk
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]); // 创建四元数
        Eigen::Isometry3d T(q); // 将四元数转换为 Isometry3d位姿

        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));  //设置刚体变换的平移
        poses.push_back( T );
    }
    
//* 计算点云并拼接
    // 相机内参 
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;  // depthScale → 把像素值转成米（mm → m）。
 
    cout<<"正在将图像转换为点云..."<<endl;
    
    typedef pcl::PointXYZRGB PointT;  // 定义点的格式：这里用的是XYZRGB
    typedef pcl::PointCloud<PointT> PointCloud; // 定义指定点格式的点云类，用于存放很多点

    // 新建一个点云
    //  new PointCloud ：一个标准的 C++ 动态内存分配操作。它在堆（heap）上分配了一块内存，用于存储一个 PointCloud 对象。
    // pointCloud( new PointCloud ): 是 C++ 的直接初始化语法。它创建了一个名为 pointCloud 的 PointCloud::Ptr 类型对象。
    PointCloud::Ptr pointCloud(new PointCloud); 

    for (int i = 0; i < 5; i++)
    {
        cout<<"转换图像中: "<<i+1<<endl;
        cv::Mat color = colorImgs[i]; // 获取彩色图像
        cv::Mat depth = depthImgs[i]; // 获取深度图像
        Eigen::Isometry3d T = poses[i]; // 获取当前帧的位姿

        for (int v = 0; v < color.rows; v++)
        {
            for (int u = 0; u < color.rows; u++)
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 获取深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                // 创建一个相机坐标系下的 3D 点
                Eigen::Vector3d point; 
                // 反投影公式：
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                // 从相机坐标系变换到世界坐标系
                Eigen::Vector3d pointWorld = T*point; 


                PointT p; // 创建一个点
                p.x = pointWorld[0]; // 设置点的 x 坐标
                p.y = pointWorld[1]; // 设置点的 y 坐标
                p.z = pointWorld[2]; // 设置点的 z 坐标
                // 设置点的颜色
                // color.step：这是图像中一行像素所占用的字节数。这个值通常等于 图像宽度 * 通道数 * 像素深度
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                // 将点添加到点云中
                pointCloud->points.push_back( p );
            }
            
        }
        
    }
    // is_dense = false 表示点云中可能有无效点（NaN）
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;
    // 用 savePCDFileBinary 保存为 PCD 格式的二进制文件（map.pcd）
    pcl::io::savePCDFileBinary("../map.pcd", *pointCloud );
    return 0;

} 