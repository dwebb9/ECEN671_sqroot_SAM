#include <iostream>
#include <Eigen/Dense>
#include <cmath>
 
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

// max measurement range: 3
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
    const int max_b_size = num_poses*10;
    const int initial_theta = 0;

    // angles are represented in radians
    MatrixXf landmark_groundtruth(num_landmarks, 2);
    landmark_groundtruth << 5 , 2 , 10 , 2 , 15 , 2 , 5 , 7 , 10 , 7 , 15 , 7;
    MatrixXf pose_groundtruth(num_poses, 3);
    pose_groundtruth << 0,0,0, 2,1,0.4 ,5,4,0.7 ,8,4,0.8 ,9,4,0.5, 12,4,0.2, 16,4,0.1, 18,4,0.05, 19,5,0.3, 17,8,0.9, 15,8,1.5, 13,8,1.9, 11,6,2.8, 10,6,3.2, 9,6,3.9, 7,7,3.2, 6,9,2.5, 4,9,3.2, 3,9,3.1, 3,7,3.9, 3,4,4.7, 5,4,5.6, 8,4,6.1, 9,4,0, 12,4,0.1;

    // randomly initialized as the value will change later
    VectorXf b(max_b_size);

    // temp vector that stores b values
    VectorXf temp_b(max_b_size);
    VectorXf landmark_reference(max_b_size);
    VectorXf pose_reference(max_b_size);
    int z_itt = 0;
    int b_itt = 3;
    temp_b(0) = 0.0;
    temp_b(1) = 0.0;
    temp_b(2) = 0.0;

    for(int i = 1; i < num_poses; i++)
    {
        double x_distantce = pose_groundtruth(i,0) - pose_groundtruth(i-1,0);
        double y_distantce = pose_groundtruth(i,1) - pose_groundtruth(i-1,1);
        double distance = sqrt(pow(x_distantce, 2) + pow(y_distantce,2));
        double angle_of_movement = atan2(y_distantce, x_distantce);

        // update b matrix from ground truth. 
        temp_b(b_itt) = angle_of_movement - pose_groundtruth(i-1, 2);
        temp_b(b_itt + 1) = distance;
        temp_b(b_itt + 2) = pose_groundtruth(i,2) - angle_of_movement;
        b_itt = b_itt + 3;

        // update landmark measurements
        for(int j = 0; j < num_landmarks; j++)
        {
            double x_lm_dist = landmark_groundtruth(j,0) - pose_groundtruth(i,0);
            double y_lm_dist = landmark_groundtruth(j,1) - pose_groundtruth(i,1);
            double lm_dist = sqrt(pow(x_lm_dist, 2) + pow(y_lm_dist,2));
            if(lm_dist < 3.0)
            {
                double angle_of_observation = atan2(y_lm_dist, x_lm_dist);
                temp_b(num_poses*3 + z_itt*2) = lm_dist;
                temp_b(num_poses*3 + z_itt*2 + 1) =  angle_of_observation - pose_groundtruth(i,2);
                landmark_reference(z_itt) = j;
                pose_reference(z_itt) = i;
                z_itt++;
            }
        }
    }

    int b_size = num_poses*3 + z_itt*2;
    b.resize(b_size);

    for(int i = 0; i < b_size; i++)
    {
        b(i) = temp_b(i);
    }

    // cout << "b:" << endl << b << endl;
    // cout << "last OD: " << endl << b(num_poses*3 - 3) << endl << b(num_poses*3 - 2) << endl << b(num_poses*3 - 1) << endl;

    // next step is to generate A matrix
    MatrixXf A(num_poses + z_itt, num_poses + num_landmarks); // NEEDS UPDATED!
    A(0,0) = 1;// NEEDS UPDATED!
    int pose_itt = 1;// NEEDS UPDATED!
    int measure_itt = 0;// NEEDS UPDATED!
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