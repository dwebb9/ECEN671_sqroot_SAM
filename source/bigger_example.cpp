#include <iostream>
#include <Eigen/Dense>
// #include <cmath>
 
using namespace Eigen;
using namespace std;

// strat for adding large A, R, and I. generate I and R inside a while loop and only save 
// the output. When saving input in b and generating A. Simply save in a very large matrix
// and keep track of the number of rows and columns being used. 

// linear example
// |----L-L----L-----L---L---
// 0    5 7    12    18  22

// max measurement range: 6
// max movement : 5

int main()
{
    const int num_landmarks = 2;
    const int num_poses = 3;
    const int max_b_size = num_poses + num_poses*4;
    const int landmark_groundtruth[num_landmarks] = {5, 7};
    const int pose_groundtruth[num_poses] = {0,2,6};

    VectorXf gt(num_poses + num_landmarks);
    gt << 0,2,6, 5,7;


    // randomly initialized as the value will change later
    VectorXf b(max_b_size);

    // temp vector that stores b values
    VectorXf temp_b(max_b_size);
    VectorXf landmark_reference(max_b_size);
    VectorXf pose_reference(max_b_size);
    int z_itt = 0;

    for(int i = 1; i < num_poses; i++)
    {
        temp_b(i) = pose_groundtruth[i] - pose_groundtruth[i-1];
        for(int j = 0; j < num_landmarks; j++)
        {
            int temp = abs(landmark_groundtruth[j] - pose_groundtruth[i]);
            if(temp <= 4)
            {
                temp_b(num_poses+z_itt) = landmark_groundtruth[j] - pose_groundtruth[i];
                landmark_reference(z_itt) = j;
                pose_reference(z_itt) = i;
                z_itt++;
            }
        }
    }

    b.resize(num_poses + z_itt);

    for(int i = 0; i < num_poses + z_itt; i++)
    {
        b(i) = temp_b(i);
    }

    cout << "b" << endl << b << endl;

    
    // next step is to generate A matrix
    MatrixXf A(num_poses + z_itt, num_poses + num_landmarks);
    A(0,0) = 1;
    int pose_itt = 1;
    int measure_itt = 0;
    for (int i = 1; i < num_poses + z_itt; i++)
    {
        if(i < num_poses)
        {
            A(i,pose_itt - 1) = -1;
            A(i,pose_itt) = 1;
            pose_itt++;
        }
        else
        {
            int p = pose_reference(measure_itt);
            int lm = landmark_reference(measure_itt) + num_poses;
            A(i,lm) = 1;
            A(i,p) = -1;
            measure_itt++;
        }
    }
    
    // cout << "A" << endl << A << endl;
    MatrixXf I = A.transpose()*A;

    MatrixXf L( I.llt().matrixL() );
    MatrixXf L_T=L.adjoint();//conjugate transpose

    // // solves least squares using above L*LT*x = b from cholesky
    VectorXf y = L.colPivHouseholderQr().solve(A.transpose()*b);
    VectorXf x = L_T.colPivHouseholderQr().solve(y);

    // cout << "x" << endl << x << endl;

    // error calculator 
    double error = 0.0;
    for(int i = 0; i < num_poses + num_landmarks; i++)
    {
        error += abs(x(i) - gt(i));
    }
    error = error / (num_poses + num_landmarks);
    // cout << "error" << endl << error << endl;

    return 0;
}