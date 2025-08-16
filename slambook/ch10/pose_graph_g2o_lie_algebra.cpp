#include <iostream>  // 用于标准输入输出，如 cout
#include <fstream>   // 用于文件输入输出，如读取.g2o文件
#include <string>    // 用于处理字符串，如读取文件中的"VERTEX_SE3:QUAT"
#include <Eigen/Core> // Eigen核心库，用于矩阵和向量运算

// g2o库的头文件
#include <g2o/core/base_vertex.h>            // 基础顶点类，用于自定义节点
#include <g2o/core/base_binary_edge.h>         // 基础二元边类，用于自定义两个节点间的边
#include <g2o/core/block_solver.h>           // 块求解器，用于构建Hessian矩阵的稀疏结构
#include <g2o/core/optimization_algorithm_levenberg.h> // Levenberg-Marquardt优化算法
#include <g2o/solvers/eigen/linear_solver_eigen.h>     // 使用Eigen作为线性求解器

// Sophus库的头文件
#include <sophus/se3.hpp>                      // SE3群，用于表示三维空间中的刚体变换（位姿）

using namespace std;
using namespace Eigen;
using Sophus::SE3d;
using Sophus::SO3d;

/************************************************
 * 本程序演示如何用g2o solver进行位姿图优化
 * sphere.g2o是人工生成的一个Pose graph，我们来优化它。
 * 尽管可以直接通过load函数读取整个图，但我们还是自己来实现读取代码，以期获得更深刻的理解
 * 本节使用李代数表达位姿图，节点和边的方式为自定义
 * **********************************************/

// 定义一个 6x6 的矩阵类型，用于李代数运算
typedef Matrix<double, 6, 6> Matrix6d;

// 给定一个SE3变换e，求其在李群上的右雅可比矩阵的逆
// 这里用了一个近似，因为理论上的雅可比矩阵比较复杂
Matrix6d JRInv(const SE3d &e) {
    Matrix6d J;
    // J的左上角3x3块是旋转的 hat 运算
    J.block(0, 0, 3, 3) = SO3d::hat(e.so3().log());
    // J的右上角3x3块是平移的 hat 运算
    J.block(0, 3, 3, 3) = SO3d::hat(e.translation());
    // J的左下角3x3块是零矩阵
    J.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);
    // J的右下角3x3块是旋转的 hat 运算
    J.block(3, 3, 3, 3) = SO3d::hat(e.so3().log());

    // 理论上右雅可比的逆
    J = J * 0.5 + Matrix6d::Identity();
    // 另一种更简单的近似，直接使用单位矩阵
    // J = Matrix6d::Identity();  // try Identity if you want
    return J;
}

// 定义一个 6x1 的向量类型，用于表示李代数
typedef Matrix<double, 6, 1> Vector6d;

// 自定义g2o顶点：李代数SE3
// g2o::BaseVertex<6, SE3d> 表示：优化变量维度为6，实际类型为Sophus::SE3d
class VertexSE3LieAlgebra : public g2o::BaseVertex<6, SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 宏，用于处理Eigen库内存对齐问题

    // 从文件中读取顶点数据
    // override 关键字表示重写基类的虚函数
    virtual bool read(istream &is) override {
        double data[7];
        for (int i = 0; i < 7; i++)
            is >> data[i]; // 读取7个double数据，前3个是平移，后4个是四元数
        
        // 使用读取的数据构建一个SE3d对象作为顶点的初始估计值
        setEstimate(SE3d(
            Quaterniond(data[6], data[3], data[4], data[5]), // w, x, y, z
            Vector3d(data[0], data[1], data[2]) // x, y, z
        ));
        return true;
    }

    // 将顶点数据写入文件
    virtual bool write(ostream &os) const override {
        os << id() << " "; // 写入顶点ID
        Quaterniond q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " "; // 写入平移向量
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << endl; // 写入四元数
        return true;
    }

    // 设置顶点的初始值为单位矩阵（即原点位姿）
    virtual void setToOriginImpl() override {
        _estimate = SE3d(); // 默认构造函数创建一个单位矩阵SE3
    }

    // 顶点的更新操作，这里采用**左乘**更新
    // update是优化算法计算出的增量
    virtual void oplusImpl(const double *update) override {
        Vector6d upd;
        // 将 update 数组中的6个值赋值给李代数向量 upd
        upd << update[0], update[1], update[2], update[3], update[4], update[5];
        // 将李代数向量指数映射为SE3d增量，并左乘到当前估计值上
        _estimate = SE3d::exp(upd) * _estimate;
    }
};

// 自定义g2o边：两个李代数SE3顶点之间的边
// g2o::BaseBinaryEdge<6, SE3d, VertexSE3LieAlgebra, VertexSE3LieAlgebra> 表示：
//   - 误差维度为6
//   - 测量值类型为 Sophus::SE3d
//   - 两个顶点都是 VertexSE3LieAlgebra 类型
class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, SE3d, VertexSE3LieAlgebra, VertexSE3LieAlgebra> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 宏，用于处理Eigen库内存对齐问题

    // 从文件中读取边数据
    virtual bool read(istream &is) override {
        double data[7];
        for (int i = 0; i < 7; i++)
            is >> data[i]; // 读取7个double数据作为测量值
        Quaterniond q(data[6], data[3], data[4], data[5]);
        q.normalize(); // 确保四元数是单位四元数
        setMeasurement(SE3d(q, Vector3d(data[0], data[1], data[2]))); // 设置测量值
        
        // 读取信息矩阵（协方差矩阵的逆），只读取上三角部分
        for (int i = 0; i < information().rows() && is.good(); i++)
            for (int j = i; j < information().cols() && is.good(); j++) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j); // 填充下三角部分
            }
        return true;
    }

    // 将边数据写入文件
    virtual bool write(ostream &os) const override {
        // 获取两个顶点
        VertexSE3LieAlgebra *v1 = static_cast<VertexSE3LieAlgebra *> (_vertices[0]);
        VertexSE3LieAlgebra *v2 = static_cast<VertexSE3LieAlgebra *> (_vertices[1]);
        os << v1->id() << " " << v2->id() << " "; // 写入两个顶点的ID
        SE3d m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        os << m.translation().transpose() << " "; // 写入平移
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " "; // 写入四元数

        // 写入信息矩阵（只写入上三角部分）
        for (int i = 0; i < information().rows(); i++)
            for (int j = i; j < information().cols(); j++) {
                os << information()(i, j) << " ";
            }
        os << endl;
        return true;
    }

    // 计算误差
    // 误差e = ln(Z_ij^-1 * T_i^-1 * T_j)
    virtual void computeError() override {
        // 获取两个顶点的当前估计值
        SE3d v1 = (static_cast<VertexSE3LieAlgebra *> (_vertices[0]))->estimate();
        SE3d v2 = (static_cast<VertexSE3LieAlgebra *> (_vertices[1]))->estimate();
        // 按照李代数形式计算误差，_measurement是测量值Z_ij
        _error = (_measurement.inverse() * v1.inverse() * v2).log();
    }

    // 计算雅可比矩阵
    // 误差对两个顶点的李代数增量求偏导
    virtual void linearizeOplus() override {
        SE3d v1 = (static_cast<VertexSE3LieAlgebra *> (_vertices[0]))->estimate();
        SE3d v2 = (static_cast<VertexSE3LieAlgebra *> (_vertices[1]))->estimate();
        // 右雅可比矩阵的逆，这里用的是一个近似
        Matrix6d J = JRInv(SE3d::exp(_error));
        
        // 误差对第一个顶点（xi）的雅可比
        // 公式为 - J * Ad(T_j^-1)
        _jacobianOplusXi = -J * v2.inverse().Adj();
        // 误差对第二个顶点（xj）的雅可比
        // 公式为 J * Ad(T_j^-1)
        _jacobianOplusXj = J * v2.inverse().Adj();
    }
};


int main(int argc, char **argv) {
    // 检查命令行参数，确保传入了.g2o文件路径
    if (argc != 2) {
        cout << "Usage: pose_graph_g2o_SE3_lie sphere.g2o" << endl;
        return 1;
    }
    
    // 打开文件
    ifstream fin(argv[1]);
    if (!fin) {
        cout << "file " << argv[1] << " does not exist." << endl;
        return 1;
    }
    // printf("1");


    // 配置g2o求解器
    // 定义BlockSolver的类型，6,6表示Hessian矩阵的pose-pose块是6x6的
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    // 定义线性求解器，使用Eigen的Cholesky分解
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    // 创建一个Levenberg-Marquardt优化算法实例
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 创建图模型实例
    optimizer.setAlgorithm(solver);     // 设置求解器
    optimizer.setVerbose(true);         // 打印调试信息

    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的计数器

    // 用于保存顶点和边的指针，方便后续写入文件
    vector<VertexSE3LieAlgebra *> vectices;
    vector<EdgeSE3LieAlgebra *> edges;
    
    // 逐行读取.g2o文件
    while (!fin.eof()) {
        string name;
        fin >> name;
        
        // 如果是顶点
        if (name == "VERTEX_SE3:QUAT") {
            VertexSE3LieAlgebra *v = new VertexSE3LieAlgebra();
            int index = 0;
            fin >> index;
            v->setId(index); // 设置顶点ID
            v->read(fin);    // 读取顶点数据
            optimizer.addVertex(v); // 将顶点添加到优化器中
            vertexCnt++;
            vectices.push_back(v);
            
            // 如果是第一个顶点，固定它的位置（用于消除自由度）
            if (index == 0)
                v->setFixed(true);
        } 
        // 如果是边
        else if (name == "EDGE_SE3:QUAT") {
            EdgeSE3LieAlgebra *e = new EdgeSE3LieAlgebra();
            int idx1, idx2;      // 关联的两个顶点的ID
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++); // 设置边的ID
            e->setVertex(0, optimizer.vertices()[idx1]); // 关联第一个顶点
            e->setVertex(1, optimizer.vertices()[idx2]); // 关联第二个顶点
            e->read(fin);        // 读取边数据
            optimizer.addEdge(e); // 将边添加到优化器中
            edges.push_back(e);
        }
        
        // 如果文件读取失败，跳出循环
        if (!fin.good()) break;
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization(); // 初始化优化器
    optimizer.optimize(30);             // 执行30次迭代优化

    cout << "saving optimization results ..." << endl; 

    // 因为我们使用了自定义的顶点和边，需要手动将结果写入文件
    ofstream fout("../result_lie.g2o");
    for (VertexSE3LieAlgebra *v:vectices) {
        fout << "VERTEX_SE3:QUAT ";
        v->write(fout);
    }
    for (EdgeSE3LieAlgebra *e:edges) {
        fout << "EDGE_SE3:QUAT ";
        e->write(fout);
    }
    fout.close();
    return 0;
}