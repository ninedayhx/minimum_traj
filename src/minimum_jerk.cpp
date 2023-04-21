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
#include "../include/minimum_traj.hpp"
#include "../include/matplotlibcpp.h"
#include <vector>

#define VAR_STR(var) (#var)

using namespace std;
namespace plt = matplotlibcpp;

void plot_traj(Eigen::MatrixXd pos, Eigen::MatrixXd traj);

int main()
{
    Eigen::MatrixXd postion(3, 5);
    Eigen::Vector3d start_v, end_v, start_a, end_a;
    Eigen::VectorXd Time(4);
    postion << 0, 1, 2, 3, 5,
        0, 1, 6, 3, 6,
        1, 1, 2, 3, 2;
    start_v << 0, 0, 0;
    end_v << 0, 0, 0;
    start_a << 0, 0, 0;
    end_a << 0, 0, 0;
    Time << 1, 1, 1, 1;

    minimum_traj jerk(JERK, 5, 3, postion, start_v, end_v, start_a, end_a, Time);

    plot_traj(postion, jerk.Cal_minimum_traj(Time));

    return 0;
}

void plot_traj(Eigen::MatrixXd pos, Eigen::MatrixXd traj)
{
    long fig1 = plt::figure();
    int seg_num = (int)(traj.cols() / 100);
    vector<double> pos_x, pos_y, pos_z;
    vector<double> pos_x1, pos_y1, pos_z1;
    std::map<std::string, std::string> keywords, kws2;
    keywords.insert(std::pair<std::string, std::string>("label", VAR_STR(pos)));
    for (int i = 0; i < pos.cols(); i++)
    {
        pos_x1.push_back(pos(0, i));
        pos_y1.push_back(pos(1, i));
        pos_z1.push_back(pos(2, i));
    }
    plt::plot3(pos_x1, pos_y1, pos_z1, keywords, fig1);

    kws2.insert(std::pair<std::string, std::string>("label", VAR_STR(traj)));
    for (int i = 0; i < seg_num; i++)
    {
        for (int j = 0; j < 100; j++)
        {
            pos_x.push_back(traj(0, i * 100 + j));
            pos_y.push_back(traj(1, i * 100 + j));
            pos_z.push_back(traj(2, i * 100 + j));
        }
    }
    plt::plot3(pos_x, pos_y, pos_z, kws2, fig1);
    plt::xlabel("x label");
    plt::ylabel("y label");
    plt::set_zlabel("z label"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
    plt::legend();
    plt::show();
}
