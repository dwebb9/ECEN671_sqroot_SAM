#include <iostream>
#include <Eigen/Dense>
 
using namespace Eigen;
using namespace std;

// strat for adding large A, R, and I. generate I and R inside a while loop and only save 
// the output. When saving input in b and generating A. Simply save in a very large matrix
// and keep track of the number of rows and columns being used. 
int main()
{
    MatrixXf A(3,3);
    A << 1, 0, 0, -1, 1, 0, 0, -1, 1;
    VectorXf b(3);
    b << 0, 2, 3;
    MatrixXf I(3,3);
    I = A.transpose()*A;

    MatrixXf L( I.llt().matrixL() );
    MatrixXf L_T=L.adjoint();//conjugate transpose

    // solves least squares using above L*LT*x = b from cholesky
    Vector3f y = L.colPivHouseholderQr().solve(A.transpose()*b);
    Vector3f x = L_T.colPivHouseholderQr().solve(y);

    // step 2
    A.resize(6,5);
    A << 1,0,0,0,0, -1,1,0,0,0, 0,-1,1,0,0, 0,-1,0,1,0, 0,0,-1,1,0, 0,0,-1,0,1;
    b.resize(6);
    b << 0,2,4,3,-1,1;

    I = A.transpose()*A;

    // only define R once. 
    MatrixXf R( I.llt().matrixL() );
    MatrixXf R_T = R.adjoint();//conjugate transpose
    VectorXf temp = A.transpose()*b;

    // cout << "R" << endl << R << endl;
    // cout << "AT*b" << endl << temp << endl;
    VectorXf y1 = R.colPivHouseholderQr().solve(temp);
    VectorXf x1 = R_T.colPivHouseholderQr().solve(y1);

    cout << "x1" << endl << x1 << endl;

    return 0;
}