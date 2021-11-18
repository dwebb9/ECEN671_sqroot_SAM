#include <iostream>
#include <Eigen/Dense>
 
using namespace Eigen;
using namespace std;

// strat for adding large A, R, and I. generate I and R inside a while loop and only save 
// the output. When saving input in b and generating A. Simply save in a very large matrix
// and keep track of the number of rows and columns being used. 

// 2D example
// 10 |--------------------
//    |--------------------
//    |--------------------
//    |----L---L----L------
//    |--------------------
//  5 |--------------------
//    |--------------------
//    |--------------------
//    |----L----L----L-----
//    |--------------------
//    0    5    10   15   20 

// max measurement range: 6
// max movement : 5

// Ax = b
// b: measurements 
// A: combined jacobians
// x: trajectory + landmarks

// ***** Code assumes you know what landmark each measurement is associated with


int main()
{
    const int num_landmarks = 6;
    const int num_poses = 25;
    const int max_b_size = num_poses + num_poses*4;

    MatrixXf landmark_groundtruth(num_landmarks, 2);
    landmark_groundtruth << 5 , 2 , 10 , 2 , 15 , 2 , 5 , 7 , 10 , 7 , 15 , 7;
    MatrixXf pose_groundtruth(num_poses, 2);
    pose_groundtruth << 0,0, 2,1 ,5,4 ,8,4 ,9,4, 12,4, 16,4, 18,4, 19,5, 17,8, 15,8, 13,8, 11,6, 10,6, 9,6, 7,7, 6,9, 4,9, 3,9, 3,7, 3,4, 5,4, 8,4, 9,4, 12,4;

    // randomly initialized as the value will change later
    VectorXf b(max_b_size);

    // temp vector that stores b values
    VectorXf temp_b(max_b_size);
    VectorXf landmark_reference(max_b_size);
    VectorXf pose_reference(max_b_size);
    int z_itt = 0;

    for(int i = 1; i < num_poses; i++)
    {
        int x_distantce = pose_groundtruth(i,0) - pose_groundtruth(i-1,0);
        int y_distantce = pose_groundtruth(i,1) - pose_groundtruth(i-1,1);
        int distance = sqrt(pow(x_distantce, 2) + pow(y_distantce,2));

        temp_b(i) = distance;
        for(int j = 0; j < num_landmarks; j++)
        {
            int x_lm_dist = landmark_groundtruth(j,0) - pose_groundtruth(i,0);
            int y_lm_dist = landmark_groundtruth(j,1) - pose_groundtruth(i,1);
            int temp = sqrt(pow(x_distantce, 2) + pow(y_distantce,2));
            if(temp <= 6)
            {
                temp_b(num_poses+z_itt) = temp; // update 3 pose measurements as well as 2 lm measurements
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

    
    // // next step is to generate A matrix
    // MatrixXf A(num_poses + z_itt, num_poses + num_landmarks);
    // A(0,0) = 1;
    // int pose_itt = 1;
    // int measure_itt = 0;
    // for (int i = 1; i < num_poses + z_itt; i++)
    // {
    //     if(i < num_poses)
    //     {
    //         A(i,pose_itt - 1) = -1;
    //         A(i,pose_itt) = 1;
    //         pose_itt++;
    //     }
    //     else
    //     {
    //         int p = pose_reference(measure_itt);
    //         int lm = landmark_reference(measure_itt) + num_poses;
    //         A(i,lm) = 1;
    //         A(i,p) = -1;
    //         measure_itt++;
    //     }
    // }
    
    // // Squareroot SAM
    // MatrixXf I = A.transpose()*A;

    // MatrixXf L( I.llt().matrixL() );
    // MatrixXf L_T=L.adjoint();//conjugate transpose

    // // // solves least squares using above L*LT*x = AT*b from cholesky
    // VectorXf y = L.colPivHouseholderQr().solve(A.transpose()*b);
    // VectorXf x = L_T.colPivHouseholderQr().solve(y);

    // cout << "x" << endl << x << endl;

    // error calculator 
    // double error = 0.0;
    // for(int i = 0; i < num_poses + num_landmarks; i++)
    // {
    //     error += abs(x(i) - gt(i));
    // }
    // error = error / (num_poses + num_landmarks);
    // cout << "error" << endl << error << endl;

    return 0;
}