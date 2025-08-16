#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

string image_file = "../distorted.png";   // 请确保路径正确
// ! 如果 image_file = "./distorted.png" 这个路径是错误的，或者文件本身损坏，cv::imread 函数将返回一个空的 cv::Mat 对象。这个空对象的 rows 和 cols 都为 0，并且它的类型是无效的。
//! 这导致:
//! terminate called after throwing an instance of 'cv::Exception'
//!   what():  OpenCV(4.5.4) ./modules/core/src/array.cpp:2494: error: (-206:Bad flag (parameter or structure field)) Unrecognized or unsupported array type in function 'cvGetMat'


//* 本程序实现去畸变部分的代码。尽管我们可以调用OpenCV的去畸变，但自己实现一遍有助于理解。
int main(int argc, char** argv) {
    // 畸变参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // 内参
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image = cv::imread(image_file, 0);  // 读取畸变后的灰度图，CV_8UC1
    int rows = image.rows, cols = image.cols;
    // 这种写法只是分配了内存，但是内容未初始化，更关键的是，图像数据没有被初始化为0，某些OpenCV函数可能对未初始化数据敏感。
    // cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);  // 用于存储去畸变以后的图
    cv::Mat image_undistort = cv::Mat::zeros(rows, cols, CV_8UC1);


    // 计算去畸变后图像的内容
    for (int v = 0; v < rows; v++)
    {
        for (int u = 0; u < cols; u++)
        {
            // 按照公式，计算点(u,v)对应到畸变图像中的坐标(u_distorted, v_distorted)一般是浮点数
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double r = sqrt(x * x + y * y);
            double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
            double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
            double u_distorted = fx * x_distorted + cx;
            double v_distorted = fy * y_distorted + cy;



            // 将畸变坐标(u_distorted, v_distorted)强制转换为最近的浮点数，来找到最接近的畸变图像像素值，然后赋给去畸变图像(u,v)
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
            image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            } else { // 如果畸变图像的坐标越界，则去畸变图像该位置赋为0（黑色）。
            image_undistort.at<uchar>(v, u) = 0;
            }
        }
    }


    // 画图去畸变后图像
    cv::imshow("distorted", image);
    cv::imshow("undistorted", image_undistort);
    cv::waitKey();
    return 0;

}