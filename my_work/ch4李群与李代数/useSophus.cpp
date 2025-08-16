#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

int main(int argc, char** argv) {
    
    

//* Sophus::SO(3)群元素的构造
    // 1.可以直接从旋转矩阵构造
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();// 沿Z轴旋转90度的旋转矩阵
    Sophus::SO3d SO3_R(R);
    // 2. 亦可从旋转向量构造:将旋转向量（李代数）映射到旋转矩阵（李群）。
    Sophus::SO3d SO3_v = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, M_PI/2)); 
    // 3. 四元数构造
    Eigen::Quaterniond q(R);
    Sophus::SO3d SO3_q(q);
//* 上述表达方式都是等价的

//* 使用对数映射获得它的李代数
    Eigen::Vector3d so3 = SO3_R.log();
    // 输出李代数的行向量形式
    cout << "so3 = " << so3.transpose() << endl;
    //* hat 为李代数向量到反对称矩阵
    cout<<"so3 hat=\n"<<Sophus::SO3d::hat(so3)<<endl; 
    //* 相对的，vee为反对称到向量
    cout<<"so3 hat vee= "<<Sophus::SO3d::vee( Sophus::SO3d::hat(so3) ).transpose()<<endl;

//* 增量扰动模型的更新
    Eigen::Vector3d update_so3(1e-4, 0, 0); // 假设更新量
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) = SO3_R;
    cout << "SO3 updated = " << SO3_updated.log().transpose() << endl;



    cout<<"************我是分割线*************"<<endl;

//* 对SE(3)操作大同小异  
    Eigen::Vector3d t(1,0,0);           // 沿X轴平移1
    Sophus::SE3 SE3_Rt(R, t);           // 从R,t构造SE(3)
    Sophus::SE3 SE3_qt(q, t);           // 从q,t构造SE(3)
    cout<<"SE3 from R,t= "<<endl<<SE3_Rt.matrix()<<endl;
    cout<<"SE3 from q,t= "<<endl<<SE3_qt.matrix()<<endl;
//* 李代数se(3) 是一个六维向量，方便起见先typedef一下
    typedef Eigen::Matrix<double,6,1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;
    // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
//* 同样的，有hat和vee两个算
    cout<<"se3 hat = "<<endl<<Sophus::SE3d::hat(se3)<<endl;
    cout<<"se3 hat vee = "<<Sophus::SE3d::vee( Sophus::SE3d::hat(se3) ).transpose()<<endl;
//* 增量扰动模型的更新
    Vector6d update_se3; //更新量
    update_se3.setZero();
    update_se3(0,0) = 1e-4d;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3)*SE3_Rt;
    cout<<"SE3 updated = "<<endl<<SE3_updated.matrix()<<endl;
    return 0;
}

// so3 =      0      0 1.5708
// so3 hat=
//       0 -1.5708       0
//  1.5708       0      -0
//      -0       0       0
// so3 hat vee=      0      0 1.5708
// SO3 updated =      0      0 1.5708
// ************我是分割线*************
// SE3 from R,t= 
// 2.22045e-16          -1           0           1
//           1 2.22045e-16           0           0
//           0           0           1           0
//           0           0           0           1
// SE3 from q,t= 
// 2.22045e-16          -1           0           1
//           1 2.22045e-16           0           0
//           0           0           1           0
//           0           0           0           1
// se3 =  0.785398 -0.785398         0         0         0    1.5708
// se3 hat = 
//         0   -1.5708         0  0.785398
//    1.5708         0        -0 -0.785398
//        -0         0         0         0
//         0         0         0         0
// se3 hat vee =  0.785398 -0.785398         0         0         0    1.5708
// SE3 updated = 
// 2.22045e-16          -1           0      1.0001
//           1 2.22045e-16           0           0
//           0           0           1           0
//           0           0           0           1