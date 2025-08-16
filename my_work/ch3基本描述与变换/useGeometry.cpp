#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

/****************************
* 本程序演示了 Eigen 几何模块的使用方法
****************************/

int main(int argc, char const *argv[])
{
    //* Eigen/Geometry 模块提供了各种旋转和平移的表示

    // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity(); //Identity生成单位矩阵 

    // 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
    cout.precision(3); //设置标准输出流 cout 的浮点数显示精度为 3 位有效数字。

    // 旋转向量可以用matrix()转换成矩阵后，赋给旋转矩阵
    cout<<"rotation matrix =\n"<<rotation_vector.matrix() <<endl;  
    // 也可以.toRotationMatrix()直接赋值
    rotation_matrix = rotation_vector.toRotationMatrix();

    //* 坐标变换
    // 用 AngleAxis旋转向量
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;
    // 或者用旋转矩阵
    v_rotated = rotation_matrix * v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

    //* 欧拉角: 可以将旋转矩阵直接转换成欧拉角
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles ( 2,1,0 ); // ZYX顺序，即roll pitch yaw顺序
    cout<<"yaw pitch roll = "<<euler_angles.transpose()<<endl;

    //* 欧氏变换矩阵使用 Eigen::Isometry
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity(); // !虽然称为3d，实质上是4＊4的矩阵
    T.rotate ( rotation_vector );  // 旋转矩阵部分按rotation_vector设置
    T.pretranslate ( Eigen::Vector3d ( 1,3,4 ) );  // 平移向量设成(1,3,4)
    cout << "Transform matrix = \n" << T.matrix() <<endl;
    // ! 变换类不是矩阵类型，而是封装了变换逻辑的类,打印为矩阵时需要.matrix()

    // *用变换矩阵进行坐标变换
    Eigen::Vector3d v_transformed = T*v;   
    cout<<"v tranformed = "<<v_transformed.transpose()<<endl;

    //* 对于仿射变换 Eigen::Affine3d
    Eigen::Affine3d AF = Eigen::Affine3d::Identity();
    AF.rotate(3 * rotation_matrix); // 仿射变换的3*3矩阵是任意的，包含缩放、剪切
    AF.pretranslate(Eigen::Vector3d(1, 3, 4));
    cout << "Affine3d matrix AF = \n" << AF.matrix() <<endl;// 最后一行任然固定为（0，1，1，1）


    //* 射影变换 Eigen::Projective3d
    // 是更自由的放射变换，允许最后一行不固定
    Eigen::Projective3d P = Eigen::Projective3d::Identity();
    P.rotate(rotation_vector);
    P.pretranslate(Eigen::Vector3d(1, 3, 4));
    // 访问底层矩阵
    Eigen::Matrix4d& mat = P.matrix();
    mat.row(3) << 0.1, 0.2, 0.3, 1.0;
    cout << "Projective3d matrix P = \n" << P.matrix() <<endl;


    //* 四元数
    // 可以直接把AngleAxis赋值给四元数，反之亦然
    Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;  
    // 请注意coeffs(系数数组)的顺序是(x,y,z,w), w为实部，前三者为虚部
    // 也可以把旋转矩阵赋给它
    q = Eigen::Quaterniond ( rotation_matrix );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;
    // *使用四元数旋转一个向量，使用重载的乘法即可
    v_rotated = q*v; // ! 注意数学上是qvq^{-1}
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

    return 0;
}
