#include <iostream>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 100

int main(int argc, char **argv) {
    //生成一个100×100的随机矩阵
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN= MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
        
    //提取左上角3×3块
    Matrix3d matrix_33;
    cout << "print matrix 3x3: " << endl;
    for (int i = 0; i < 3; i++) 
    {
        for (int j = 0; j < 3; j++) 
	{
	  matrix_33(i,j)=matrix_NN(i, j);
	  cout << matrix_33(i, j) << "\t";
	}
        cout << endl;
    }
    
    //赋值为I
    matrix_33=Matrix3d::Identity();
    cout << "print matrix 3x3 after processing: " << endl;
    for (int i = 0; i < 3; i++) 
    {
        for (int j = 0; j < 3; j++) 
	{
	  cout << matrix_33(i, j) << "\t";
	}
        cout << endl;
    }
    
  return 0;
}