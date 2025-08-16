#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char const *argv[])
{
    // 读取命令行参数argv[1]指定的图像
    cv::Mat image;
    image = cv::imread(argv[1]); //cv::imread函数读取指定路径下的图像

    // 判断图像文件是否正确读取
    if ( image.data == nullptr ) //图片数据不存在
    {
        cerr<<"文件"<<argv[1]<<"不存在."<<endl;
        return 0;
    }

//* 首先输出一些基本信息
    // w为列数.cols，h为行数.rows
    // 通道数使用.channels()访问
    cout<<"图像宽为"<<image.cols<<",高为"<<image.rows<<",通道数为"<<image.channels()<<endl;
    cv::imshow("image", image);         // cv::imshow显示图像(窗口名称，图像文件)
    cv::waitKey ( 0 );                  // 暂停程序,等待一个按键输入

    // 判断image的类型:CV_8UC1灰度图，CV_8UC3彩色图
    if ( image.type() != CV_8UC1 && image.type() != CV_8UC3 )
    {
        cout<<"请输入一张彩色图或灰度图."<<endl;
        return 0;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (size_t y = 0; y < image.rows; y++) {
        // 用cv::Mat::ptr获得图像的行指针
        unsigned char *row_ptr = image.ptr<unsigned char>(y); // 第y行头指针
        for (size_t x = 0; x < image.cols; x++) {
            unsigned char *data_ptr = &row_ptr[x * image.channels()]; // data_ptr 指向待访问的像素数据
            // 输出该像素的每个通道,如果是灰度图就只有一个通道
            for (int c = 0; c < image.channels(); c++)
            {
                unsigned char data = data_ptr[c]; // data为I(x,y)第c个通道的值
            }
        }
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    // chrono::duration_cast<chrono::duration<double>>(...)：这个函数模板用于将时间间隔从一种单位转换为另一种。
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout<<"遍历图像用时："<<time_used.count()<<" 秒。"<<endl;

//* 关于 cv::Mat 的拷贝
    // 直接赋值并不会拷贝数据，两个变量共享底层数据
    cv::Mat image_another = image;
    image_another( cv::Rect (0,0,100,100) ).setTo (0); // 将左上角100*100的块置零
    // 修改 image_another 会导致 image 发生变化
    cv::imshow ( "image", image );
    cv::waitKey(0);

    //* 应当使用clone函数来拷贝数据
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);

    cv::destroyAllWindows();

    // 对于图像还有很多基本的操作,如剪切,旋转,缩放等,限于篇幅就不一一介绍了,请参看OpenCV官方文档查询每个函数的调用方法.
    return 0;
}
