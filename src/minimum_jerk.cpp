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

void plot_3Dtraj(Eigen::MatrixXd pos, Eigen::MatrixXd traj);
void plot_2Dtraj(Eigen::MatrixXd pos, Eigen::MatrixXd traj);

int main()
{
    Eigen::MatrixXd postion3d(3, 5);
    Eigen::MatrixXd postion2d(2, 5);
    Eigen::MatrixXd dstart3d(4, 3), dend3d(4, 3);
    Eigen::MatrixXd dstart2d(4, 2), dend2d(4, 2);
    Eigen::VectorXd Time(4);
    postion3d << 0, 1, 2, 3, 5,
        0, 1, 6, 3, 6,
        1, 1, 2, 3, 2;

    postion2d << 0, 0, 3, 2, 5,
        0, 2, 3, 4, 6;

    dstart3d << 0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0;
    dend3d << 0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0;

    dstart2d << 0, 0,
        0, 0,
        0, 0,
        0, 0;
    dend2d << 0, 0,
        0, 0,
        0, 0,
        0, 0;
    Time << 1, 1, 1, 1;

    minimum_traj jerk(JERK, 3, 5, 3, postion3d, dstart3d, dend3d, Time);

    minimum_traj snap(SNAP, 3, 7, 4, postion3d, dstart3d, dend3d, Time);

    // minimum_traj jerk(JERK, 2, 5, 3, postion2d, dstart2d, dend2d, Time);

    // plot_2Dtraj(postion2d, jerk.Cal_minimum_traj(Time, 2));
    plot_3Dtraj(postion3d, jerk.Cal_minimum_traj(Time, 3));

    return 0;
}

void plot_3Dtraj(Eigen::MatrixXd pos, Eigen::MatrixXd traj)
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

void plot_2Dtraj(Eigen::MatrixXd pos, Eigen::MatrixXd traj)
{
    int seg_num = (int)(traj.cols() / 100);
    vector<double> pos_x, pos_y;
    vector<double> pos_x1, pos_y1;
    std::map<std::string, std::string> keywords, kws2;
    keywords.insert(std::pair<std::string, std::string>("label", VAR_STR(pos)));
    for (int i = 0; i < pos.cols(); i++)
    {
        pos_x1.push_back(pos(0, i));
        pos_y1.push_back(pos(1, i));
    }
    plt::plot(pos_x1, pos_y1, keywords);

    kws2.insert(std::pair<std::string, std::string>("label", VAR_STR(traj)));
    for (int i = 0; i < seg_num; i++)
    {
        for (int j = 0; j < 100; j++)
        {
            pos_x.push_back(traj(0, i * 100 + j));
            pos_y.push_back(traj(1, i * 100 + j));
        }
    }
    plt::plot(pos_x, pos_y, kws2);
    plt::xlabel("x label");
    plt::ylabel("y label");
    plt::legend();
    plt::show();
}
