#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
    // 要求命令行参数指定两张图片路径
    if (argc != 3) {
        cout << "usage: feature_extraction img1 img2" << endl;
        return 1;
    }
    
    Mat img_1 = imread(argv[1], IMREAD_COLOR);
    Mat img_2 = imread(argv[2], IMREAD_COLOR);
    assert(img_1.data != nullptr && img_2.data != nullptr);

    vector<KeyPoint> keypoints_1, keypoints_2; // 存储关键点
    Mat descriptors_1, descriptors_2; // 存储描述子
    Ptr<FeatureDetector> detector = ORB::create(); // 使用 ORB 特征检测器
    Ptr<DescriptorExtractor> descriptor = ORB::create(); // 使用 ORB 描述子提取器
    // 使用暴力匹配器，Hamming 距离
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    //*** */ 第一步: 检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //*** */ 第二步: 根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "extract ORB cost = " << time_used.count() << " seconds." << endl;

    // 查看一下第一张图的特征点
    Mat outimg1;
    drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

    //*** */ 第三步: 匹配两幅图的描述子
    vector<DMatch> matches;  
    t1 = chrono::steady_clock::now();
    matcher->match(descriptors_1, descriptors_2, matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "match ORB cost = " << time_used.count() << " seconds." << endl;
    // cv::DMatch 用于描述两个图像中特征点之间的匹配关系。包含成员：
    // 成员名称	 类型	 描述
    // queryIdx	int	匹配点在查询集（descriptors_1）中的索引。
    // trainIdx	int	匹配点在训练集（descriptors_2）中的索引。
    // imgIdx	int	trainIdx 所在的图像的索引。在单图像匹配中，这通常为 0。在多图像匹配（multi_match）中，这个值会变化。
    // distance	float	描述子之间的距离。这个距离是衡量两个特征点相似度的标准。距离越小，表示两个描述子越相似，匹配的质量越高。
    
    
    //*** */ 第四步: 匹配点对筛选
    auto min_max = minmax_element(
        matches.begin(), matches.end(),
        [](const DMatch &m1, const DMatch &m2)
        { return m1.distance < m2.distance; });
    // minmax_element：返回一个 std::pair，其中包含指向最小值和最大值的两个迭代器。
    double min_dist = min_max.first->distance; 
    double max_dist = min_max.second->distance;
    // std::pair 是标准库的一个简单的键值对容器，可以存储两个不同或相同类型的数据成员：first 和 second
    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);
    // 保留符合要求的good matches
    std::vector<DMatch> good_matches;
    //*当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (matches[i].distance <= max(2 * min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }
    //matches.size()    
    

    //*** */ 第五步:绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    imshow("all matches", img_match);
    imshow("good matches", img_goodmatch);
    waitKey(0);


    return 0;
}
