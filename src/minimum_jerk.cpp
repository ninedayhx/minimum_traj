/**
 * @file minimum_jerk.cpp
 * @author ninedayhx (1170535490@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-04-15
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "minimum_traj.hpp"
// #include <iostream>
// #include <eigen3/Eigen/Eigen>

using namespace std;

int main()
{
    Eigen::MatrixXd postion(3, 5);
    Eigen::Vector3d start_v, end_v, start_a, end_a;
    Eigen::VectorXd Time(5);
    postion << 0, 1, 2, 3, 5,
        0, 1, 2, 3, 6,
        0, 1, 2, 3, 7;
    start_v << 0, 0, 0;
    end_v << 0, 0, 0;
    start_a << 0, 0, 0;
    end_a << 0, 0, 0;
    Time << 0, 1, 2, 3, 4;
    for (int i = 0; i < 5; i++)
    {
        cout << Time(i) << " ";
    }

    cout << "pos" << endl
         << postion << endl
         << "start_v" << endl
         << start_v << endl;

    minimum_traj jerk(postion, start_v, end_v, start_v, start_v, Time);
    return 0;
}