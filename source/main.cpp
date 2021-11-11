#include <iostream>
#include <Eigen/Cholesky>
using namespace std;

int main()
{
    int n = 2; // number of dimentions
    int m = 2; // 
    const int max_z = 6;
    const int num_landmarks = 5;
    const int landmark_groundtruth[num_landmarks] = {5, 7, 12, 15, 18};


    int const j_size = 2;
    int j_mot[j_size] = {-1, 1};
    int j_obs[j_size] = {-1, 1};

    cout << "hello world!" << "\n";
    return 0;
}