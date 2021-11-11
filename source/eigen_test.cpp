#include <iostream>
#include <Eigen/Dense>
 
using namespace Eigen;
using namespace std;
 
int main()
{
    // least squares decomposition using Cholesky. 
    MatrixXf A(3,3);
    Vector3f b;
    A << 4, 6, 4, 6, 25, 18, 4, 18, 22;
    b << 1, 2, 3;
    MatrixXf L( A.llt().matrixL() );
    MatrixXf L_T=L.adjoint();//conjugate transpose

    // solves least squares using above L*LT*x = b from cholesky
    Vector3f y = L.colPivHouseholderQr().solve(b);
    Vector3f x = L_T.colPivHouseholderQr().solve(y);

    cout << "x" << endl;
    cout << x << endl;

    // cout << "L" << endl;
    // cout << L << endl;
    // cout << "L_T" << endl;
    // cout << L_T << endl;
    // cout << "A" << endl;
    // cout << A << endl;
    // cout << "L*L_T" << endl;
    // cout << L*L_T << endl;
}