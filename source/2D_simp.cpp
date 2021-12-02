#include <iostream>
#include <Eigen/Dense>
#include <random>
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
    const int num_landmarks = 1;
    const int num_poses = 2;
    const int max_b_size = num_poses*10;
    const int initial_theta = 0;

    // angles are represented in radians
    MatrixXf landmark_groundtruth(num_landmarks, 2);
    landmark_groundtruth << 5, 2;//, 3 , 4 , 3, 5, 5 ;//, 10 , 2 , 15 , 2 , 5 , 7 , 10 , 7 , 15 , 7;
    MatrixXf pose_groundtruth(num_poses, 3);
    pose_groundtruth << 0,0,0, 1,2,3;


    // set up random norman number generation
    default_random_engine generator;
    normal_distribution<double> noise(0.0, 1.0);
    normal_distribution<double> angle_noise(0.0, 0.1);

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
        temp_b(b_itt + 1) = distance;// + noise(generator);
        temp_b(b_itt + 2) = pose_groundtruth(i,2) - angle_of_movement ;//+ angle_noise(generator);
        b_itt = b_itt + 3;

        // update landmark measurements
        for(int j = 0; j < num_landmarks; j++)
        {
            double x_lm_dist = landmark_groundtruth(j,0) - pose_groundtruth(i,0);
            double y_lm_dist = landmark_groundtruth(j,1) - pose_groundtruth(i,1);
            double lm_dist = sqrt(pow(x_lm_dist, 2) + pow(y_lm_dist,2));
            cout << "range: " << lm_dist << endl;
            if(lm_dist < 7.0)
            {
                double angle_of_observation = atan2(y_lm_dist, x_lm_dist);
                temp_b(num_poses*3 + z_itt*2) = lm_dist ;//+ noise(generator);
                temp_b(num_poses*3 + z_itt*2 + 1) =  angle_of_observation - pose_groundtruth(i,2); //+ angle_noise(generator);
                landmark_reference(z_itt) = j;
                pose_reference(z_itt) = i;
                z_itt++;
            }
        }
    }

    // z_itt--;

    int b_size = num_poses*3 + z_itt*2;
    b.resize(b_size);

    for(int i = 0; i < b_size; i++)
    {
        b(i) = temp_b(i);
    }

    cout << "b: " << endl << b << endl;

    // predition. 

    VectorXf thetaPred(2); 
    thetaPred(0) = 0;
    b_itt = 3;
    for(int i = 1; i < 2; i++)
    {
        thetaPred(i) = thetaPred(i - 1) + b(b_itt) + b(b_itt + 2);
        b_itt = b_itt + 3;
    }
    cout << "theta_pred" << endl << thetaPred << endl;
    int th_itt = 1;

    VectorXf x_pred(num_poses*3);
    b_itt = 3;
    for(int i = 3; i < num_poses*3; i+= 3)
    {
        x_pred(i) = x_pred(i-3) + b(b_itt + 1)*cos(thetaPred(th_itt - 1) + b(b_itt));
        x_pred(i + 1) = x_pred(i-2) + b(b_itt + 1)*sin(thetaPred(th_itt - 1) + b(b_itt));
        x_pred(i + 2) = thetaPred(th_itt);
        th_itt++;
        b_itt = b_itt + 3;
    }


    b_itt = 3; // reset b itterator to use in A matrix generation


    // // measurement prediction from pose precitions

    // VectorXf b_pred(b_size);
    // int z_itt_pred = 0;
    // int b_itt_pred = 3;
    // for(int i = 1; i < num_poses; i++)
    // {
    //     double x_distantce = x_pred(i*3) - x_pred((i-1)*3);
    //     double y_distantce = x_pred(i*3 + 1) - x_pred((i-1)*3 + 1);
    //     double distance = sqrt(pow(x_distantce, 2) + pow(y_distantce,2));
    //     double angle_of_movement = atan2(y_distantce, x_distantce);

    //     // update b matrix from ground truth. 
    //     b_pred(b_itt_pred) = angle_of_movement - x_pred((i-1)*3 + 2);
    //     b_pred(b_itt_pred + 1) = distance;
    //     b_pred(b_itt_pred + 2) = x_pred(i*3 + 2) - angle_of_movement ;
    //     b_itt_pred = b_itt_pred + 3;

    //     // update landmark measurements
    //     for(int j = 0; j < num_landmarks; j++)
    //     {
    //         double x_lm_dist = landmark_groundtruth(j,0) - x_pred(i*3);
    //         double y_lm_dist = landmark_groundtruth(j,1) - x_pred(i*3 + 1);
    //         double lm_dist = sqrt(pow(x_lm_dist, 2) + pow(y_lm_dist,2));
    //         cout << "range: " << lm_dist << endl;
    //         if(lm_dist < 7.0)
    //         {
    //             double angle_of_observation = atan2(y_lm_dist, x_lm_dist);
    //             b_pred(num_poses*3 + z_itt_pred*2) = lm_dist ;
    //             b_pred(num_poses*3 + z_itt_pred*2 + 1) =  angle_of_observation - x_pred(i*3 + 2);
    //             z_itt_pred++;
    //         }
    //     }
    // }

    // cout << "b_pred" << endl << b_pred << endl;

    // next step is to generate A matrix
    MatrixXf A(num_poses*3 + z_itt*2, num_poses*3 + num_landmarks*2);
    A(0,0) = 1;
    A(1,1) = 1;
    A(2,2) = 1;
    int pose_itt = 0;
    int theta_itt = 0;
    int measure_itt = 0;// NEEDS UPDATED!
    for (int i = 1; i < num_poses + z_itt; i++)
    {
        
        if(i < num_poses)
        {
            // J motion
            // cout << "hello" << endl;
            Vector2f Sigma; 
            Sigma(0) = b(b_itt + 1)*cos(thetaPred(theta_itt) + b(b_itt)); // sigma x
            Sigma(1) = b(b_itt + 1)*sin(thetaPred(theta_itt) + b(b_itt)); // sigma y
            double q = Sigma.transpose()*Sigma;

            // cout << "motion Sigma" << endl << Sigma << endl;
            // cout << "q: " << q << endl;
            A(pose_itt + 3, pose_itt) = Sigma(1)/q;
            A(pose_itt + 3, pose_itt + 1) = -Sigma(0)/q;
            A(pose_itt + 3, pose_itt + 2) = -1;
            A(pose_itt + 3, pose_itt + 3) = -Sigma(1)/q;
            A(pose_itt + 3, pose_itt + 4) = Sigma(0)/q;

            A(pose_itt + 4, pose_itt) = -sqrt(q)*Sigma(0)/q;
            A(pose_itt + 4, pose_itt + 1) = -sqrt(q)*Sigma(1)/q;
            A(pose_itt + 4, pose_itt + 3) = sqrt(q)*Sigma(0)/q;
            A(pose_itt + 4, pose_itt + 4) = sqrt(q)*Sigma(1)/q;

            A(pose_itt + 5, pose_itt) = -Sigma(1)/q;
            A(pose_itt + 5, pose_itt + 1) = Sigma(0)/q;
            A(pose_itt + 5, pose_itt + 3) = Sigma(1)/q;
            A(pose_itt + 5, pose_itt + 4) = -Sigma(0)/q;
            A(pose_itt + 5, pose_itt + 5) = 1;

            pose_itt = pose_itt + 3;
            theta_itt++;
            b_itt = b_itt + 3;
        }
        else
        {
            // J observation 
            int p = 3*pose_reference(measure_itt); // needs updated
            int p_theta = pose_reference(measure_itt);
            int lm = 2*landmark_reference(measure_itt) + num_poses*3; // needs updated
            int meas_row = num_poses*3 + 2*measure_itt;

            Vector2f Sigma; 
            Sigma(0) = b(meas_row)*cos(thetaPred(p_theta) + b(meas_row + 1)); // sigma x
            Sigma(1) = b(meas_row)*sin(thetaPred(p_theta) + b(meas_row + 1)); // sigma y
            double q = Sigma.transpose()*Sigma;

            // cout << "sigma" << endl << Sigma << endl;
            // cout << "Range: " << b(meas_row) << endl;

            // z
            A(meas_row, p) = -sqrt(q)*Sigma(0)/q; // xt
            A(meas_row, p + 1) = -sqrt(q)*Sigma(1)/q; // yx
            A(meas_row, lm) = sqrt(q)*Sigma(0)/q; // Lx
            A(meas_row, lm + 1) = sqrt(q)*Sigma(1)/q; // Ly

            // row
            A(meas_row + 1, p) = Sigma(1)/q; // xt
            A(meas_row + 1, p + 1) = -Sigma(0)/q; // yt
            A(meas_row + 1, p + 2) = -1; // Lx
            A(meas_row + 1, lm) = -Sigma(1)/q; // Ly
            A(meas_row + 1, lm + 1) = Sigma(0)/q;

            measure_itt++;
        }
    }
    
    cout << "A: " << endl << A << endl; 
    // cout << "theta_itt: " << endl << theta_itt << endl;
    // Squareroot SAM
    MatrixXf I = A.transpose()*A;

    // cout << "I" << endl << I << endl;

    MatrixXf L( I.llt().matrixL() );
    MatrixXf L_T=L.adjoint();//conjugate transpose

    // cout << "L" << endl << L << endl; 

    // // solves least squares using above L*LT*x = AT*b from cholesky

    // NEED TO DO // Redefine b to be measuerments - predicted measurment. 
    VectorXf y = L.colPivHouseholderQr().solve(A.transpose()*b);
    // cout << "y" << endl << y << endl;
    VectorXf x = L_T.colPivHouseholderQr().solve(y);

    cout << "x" << endl << x << endl;
    cout << "x_pred" << endl << x_pred << endl;

    // // prediction error
    // double pred_error = 0.0;
    // int x_itt = 0;
    // for(int i = 0; i < num_poses; i++)
    // {
    //     for(int j = 0; j < 3; j++)
    //     {
    //         pred_error += abs(x(x_itt) - pose_groundtruth(i,j));
    //         x_itt++;
    //     }
    // }
    // pred_error = pred_error / (num_poses*3);
    // cout << "pred_error: " << pred_error << endl;

    // // error calculator 
    // double error = 0.0;
    // VectorXf error_list(x.size());
    // x_itt = 0;
    // for(int i = 0; i < num_poses + num_landmarks; i++)
    // {
    //     if(i< num_poses)
    //     {
    //         for(int j = 0; j < 3; j++)
    //         {
    //             error += abs(x(x_itt) - pose_groundtruth(i,j));
    //             error_list(x_itt) = error;
    //             x_itt++;
    //         }
    //     }
    //     else
    //     {
    //         for(int j = 0; j < 2; j++)
    //         {
    //             error += abs(x(x_itt) - landmark_groundtruth(i-25,j));
    //             x_itt++;
    //         }
    //     }
    // }
    // error = error / (num_poses*3 + num_landmarks*2);
    // // cout << "error list" << endl << error_list << endl;
    // cout << "error" << endl << error << endl;

    return 0;
}