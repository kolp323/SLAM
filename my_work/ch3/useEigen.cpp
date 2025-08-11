#include <iostream>
using namespace std;
#include <ctime>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

#define MATRIX_SIZE 50

/****************************
* 本程序演示了 Eigen 基本类型的使用方法
****************************/

int main(int argc, char** argv) {
// *------1. Eigen中各种变量的声明------*
    //* Eigen 中所有向量和矩阵都是Eigen::Matrix，它是一个模板类。
    //* 它的前三个参数为：数据类型，行，列
    Eigen::Matrix<float, 2, 3> matrix_23; // 声明一个2*3的float矩阵

    //* 同时，Eigen 通过 typedef 提供了许多内置类型，不过底层仍是Eigen::Matrix
    //* 例如 Vector3d 实质上是 Eigen::Matrix<double, 3, 1>，即三维列向量
    Eigen::Vector3d v_3d;
    Eigen::Matrix<float,3,1> vd_3d; // 这是一样的

    //* Matrix3d 实质上是 Eigen::Matrix<double, 3, 3>
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero(); //初始化为零

    //* 如果不确定矩阵大小，可以使用动态大小的矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    Eigen::MatrixXd matrix_x; // 更简单的写法


// *------2. 对Eigen阵的操作------*
    //* 输入数据（初始化）
    matrix_23 << 1, 2, 3, 4, 5, 6;
    cout<<"matix_23" << matrix_23 << endl;

    //* 访问矩阵元素
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            cout<< matrix_23(i, j) << "\t";
        }
        cout<<endl; //进行换行
    }

    
    //* 矩阵/向量乘法（实际上仍是矩阵和矩阵）
    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;
    // 但注意Vector3d中的元素都是double类型,不能混合两种不同类型的矩阵，像这样是错的；
    //! Eigen::Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
    // 应该显式转换：.cast<double>(): 用于类型转换的成员函数。它会创建一个新的矩阵，其元素类型被转换成 <double>
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout<< result << endl;
    
    Eigen::Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    cout<< result2 << endl;
    
    
    //*一些矩阵运算
    matrix_33 = Eigen::Matrix3d::Random();  // 产生一个随机数矩阵
    cout << matrix_33 << endl << endl;
    cout << matrix_33.transpose() << endl;      // 转置
    cout << matrix_33.sum() << endl;            // 元素求和
    cout << 10*matrix_33 << endl;               // 数乘
    cout << matrix_33.inverse() << endl;        // 逆
    cout << matrix_33.trace() << endl;          // 迹
    cout << matrix_33.determinant() << endl;    // 行列式

    //* 特征值
    // Eigen::SelfAdjointEigenSolver解释：
    // 实对称矩阵 的特征值一定是实数，特征向量可以正交化。SelfAdjointEigenSolver 利用这一性质，计算更快、更稳定。
    // 如果矩阵不是对称的，就必须用 Eigen::EigenSolver（会得到可能包含复数的特征值）
    // <Eigen::Matrix3d>解释：这个 SelfAdjointEigenSolver 专门用来分解 3×3 的实对称矩阵
    // AᵀA 在数学上是一个Gram矩阵，是半正定的对称矩阵
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl; // eigen_solver.eigenvalues() 返回一个 Eigen::Vector3d（列向量），包含按升序排列的三个特征值。
    cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl; // eigen_solver.eigenvectors() 返回一个 3×3 矩阵，每一列是一个归一化的特征向量。

    // * 解方程
    // 我们求解 matrix_NN * x = v_Nd 这个方程
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);
    // 方法一：直接求逆
    clock_t time_stt = clock();
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time use in normal inverse is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
    // 方法二：矩阵分解，如QR分解
    // 求解Ax=b, 使用Vector3d x = A.colPivHouseholderQr().solve(b);
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time use in Qr decomposition is " << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;

    return 0;
}

