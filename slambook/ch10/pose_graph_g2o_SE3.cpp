#include <iostream>  // 用于标准输入输出，比如 cout
#include <fstream>   // 用于文件操作，比如读取 .g2o 文件
#include <string>    // 用于处理字符串，比如读取文件中的 "VERTEX_SE3:QUAT"

// g2o 库的头文件，包含了常用的 SLAM 3D 类型
#include <g2o/types/slam3d/types_slam3d.h>
// 用于配置 g2o 求解器的头文件
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

using namespace std;

/************************************************
 * 本程序演示如何用 g2o solver 进行位姿图优化
 * sphere.g2o 是人工生成的一个 Pose graph，我们来优化它。
 * 尽管可以直接通过 load 函数读取整个图，但我们还是自己来实现读取代码，以期获得更深刻的理解。
 * 这里使用 g2o/types/slam3d/ 中的 SE3 表示位姿，它实质上是四元数而非李代数。
 * **********************************************/

int main(int argc, char **argv) {
    // 检查命令行参数，确保用户输入了要优化的文件路径
    if (argc != 2) {
        cout << "Usage: pose_graph_g2o_SE3 sphere.g2o" << endl;
        return 1;
    }
    
    // 打开用户指定的文件
    ifstream fin(argv[1]);
    // 检查文件是否成功打开
    if (!fin) {
        cout << "file " << argv[1] << " does not exist." << endl;
        return 1;
    }

    // --- ## 1. 配置 g2o 求解器

    // 定义块求解器（Block Solver）的类型。
    // g2o::BlockSolverTraits<6, 6> 表示位姿的维度是6，残差的维度也是6。
    // 这里的6是指李代数的维度，对于SE3位姿，是3个旋转+3个平移。
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    
    // 定义线性求解器（Linear Solver）的类型。
    // 使用 Eigen 库来求解线性方程，这是 g2o 默认推荐的求解器之一。
    // BlockSolverType::PoseMatrixType 是 g2o 内部定义的矩阵类型。
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    
    // 创建一个 Levenberg-Marquardt（LM）优化算法实例。
    // 它需要一个求解器实例作为参数。
    // std::make_unique 是 C++14 的语法，用于创建智能指针，确保内存自动管理。
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
        
    g2o::SparseOptimizer optimizer;     // 创建一个稀疏图优化器实例，这就是我们的图模型。
    optimizer.setAlgorithm(solver);     // 将配置好的求解器设置给优化器。
    optimizer.setVerbose(true);         // 开启调试输出，可以看到每次迭代的误差等信息。

    // ---## 2. 从文件读取数据并构建图

    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的计数器
    
    // 循环读取文件，直到文件末尾
    while (!fin.eof()) {
        string name;
        fin >> name; // 读取每行第一个单词，判断是顶点还是边
        
        // 如果是顶点
        if (name == "VERTEX_SE3:QUAT") {
            // 创建一个 g2o 内置的 SE3 顶点
            g2o::VertexSE3 *v = new g2o::VertexSE3();
            int index = 0;
            fin >> index;      // 读取顶点的 ID
            v->setId(index);   // 设置顶点的 ID
            v->read(fin);      // 调用内置的 read 函数，从文件中读取顶点数据（位姿）
            optimizer.addVertex(v); // 将这个顶点添加到图中
            vertexCnt++;
            
            // 如果是第0号顶点，就将它固定住。
            // 这是为了在优化时消除整个图的自由度，防止图“乱跑”。
            if (index == 0)
                v->setFixed(true);
        } 
        // 如果是边
        else if (name == "EDGE_SE3:QUAT") {
            // 创建一个 g2o 内置的 SE3 边
            g2o::EdgeSE3 *e = new g2o::EdgeSE3();
            int idx1, idx2;    // 关联的两个顶点的 ID
            fin >> idx1 >> idx2; // 读取这两个 ID
            e->setId(edgeCnt++); // 设置边的 ID，并让计数器自增
            
            // 将边与对应的两个顶点关联起来
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            
            e->read(fin);      // 调用内置的 read 函数，从文件中读取边的测量值和信息矩阵
            optimizer.addEdge(e); // 将这条边添加到图中
        }
        
        // 如果文件读取失败，就跳出循环
        if (!fin.good()) break;
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    // ---## 3. 执行优化

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization(); // 初始化优化器，准备开始
    optimizer.optimize(30);             // 执行30次迭代优化
    
    // ---## 4. 保存优化结果

    cout << "saving optimization results ..." << endl;
    optimizer.save("../result.g2o"); // 将优化后的图保存到 result.g2o 文件中
    
    return 0;
}