#include <iostream>
#include <Eigen/Dense>
 
using namespace Eigen;
using namespace std;

// linear example
// |----L-L----L-----L---L---
// 0    5 7    12    18  22

// max measurement range: 6
// max movement : 5

// Ax = b
// b: measurements 
// A: combined jacobians
// x: trajectory + landmarks

// ***** Code assumes you know what landmark each measurement is associated with

int main()
{
    int n = 2; // starting dimension of A
    int m = 2; // starting dimension of A
    const int max_n = 100;
    const int max_m = 100;
    const int max_z = 6;
    const int num_landmarks = 5;
    const int num_poses = 25;
    const int max_b_size = num_poses + num_poses*4;
    const int landmark_groundtruth[num_landmarks] = {5, 7, 12, 18, 22};
    const int pose_groundtruth[num_poses] = {0,2,6,8,5, 9,12,11,9,13, 18,20,21,20,24, 21,17,18,14,10, 6,2,1,4,5};

    VectorXf gt(num_poses + num_landmarks);
    gt << 0,2,6,8,5, 9,12,11,9,13, 18,20,21,20,24, 21,17,18,14,10, 6,2,1,4,5, 5,7,12,18,22;


    // randomly initialized as the value will change later
    VectorXf b(max_b_size);

    // b = [0, u1, u2 ... um, z1, z2 ... zn]
    // temp vector that stores b values
    VectorXf temp_b(max_b_size);
    VectorXf landmark_reference(max_b_size);
    VectorXf pose_reference(max_b_size);
    int z_itt = 0;

    for(int i = 1; i < num_poses; i++)
    {
        temp_b(i) = pose_groundtruth[i] - pose_groundtruth[i-1]; // odometry 
        for(int j = 0; j < num_landmarks; j++)
        {
            int temp = abs(landmark_groundtruth[j] - pose_groundtruth[i]);
            if(temp <= 6)
            {
                temp_b(num_poses+z_itt) = landmark_groundtruth[j] - pose_groundtruth[i]; // range
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

    
    // next step is to generate A matrix
    // Equations that make up A.
    // h(theta) = [ xt - xt-1] (corresponds to u)
    //            [ L -  xt]   (corresponds to z)

                
    // Jmotion = [-1 1]
    // Jmeasure = [-1 1]

    // A =     x0  x1 x2 ...  l1 l2 ....
    //       0 [1  0   0  ...  0 0 ....]
    //       u1[-1 1   0  ...  0 0 ....]
    //       u2[0  -1  1  ...  0 0 ....]
    //       z1[0  -1  0  ...  1 0 ....]
    //       z2[0   0  -1 ...  1 0 ....]
    //       z3[0   0  -1 ...  0 1 ....]

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
    
    cout << "A: " << endl << A << endl;

    // Squareroot SAM
    MatrixXf I = A.transpose()*A;

    MatrixXf L( I.llt().matrixL() ); //LL' = A'A
    MatrixXf L_T=L.adjoint();//conjugate transpose

    // // solves least squares using above L*LT*x = b from cholesky
    VectorXf y = L.colPivHouseholderQr().solve(A.transpose()*b);
    VectorXf x = L_T.colPivHouseholderQr().solve(y);

    cout << "x" << endl << x << endl;

    // error calculator 
    double error = 0.0;
    for(int i = 0; i < num_poses + num_landmarks; i++)
    {
        error += abs(x(i) - gt(i));
    }
    error = error / (num_poses + num_landmarks);
    cout << "error" << endl << error << endl;

    return 0;
}