/**
 * @file minimum_traj.hpp
 * @author ninedayhx (1170535490@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-04-15
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __MINI_TRAJ_H
#define __MINI_TRAJ_H
#include <iostream>
#include "eigen3/Eigen/Eigen"
#include <cmath>

using namespace std;

class minimum_traj
{
private:
    /* data */

    // store the points data provided by people,
    //
    //      the coordinate of start point,waypoints and end points
    //      the velocity of start points and end point
    //      the acceleration of start point and end point
    //
    // eg: a trajectory with n segments i.e. n+1 points; n-1 waypoints
    //    the structure of D_Fixed will be a (2*3+n)*3 matrix
    //    x    y    z
    //    x0   y0   z0
    //    vx0  vy0  vz0
    //    ax0  ay0  az0
    //    xn   yn   zn
    //    vxn  vyn  vzn
    //    axn  ayn  azn
    //    x1   y1   z1
    //    x2   y2   z2
    //        .....
    //    xn-1 yn-1 zn-1
    //    which contains all the data people defined
    Eigen::MatrixXd D_Fixed;
    // close form solution optimal result
    Eigen::MatrixXd D_P_optimal;
    // the solved waypoints of all trajectory
    Eigen::MatrixXd D_total;
    // the mapping matrix of all trajectory
    Eigen::MatrixXd A_total;
    // the hessis matrix of all trajectory
    Eigen::MatrixXd Q_total;
    // the select matrix to devide the waypoints data into fixed and free
    Eigen::MatrixXd C_select;

    /* function */
    int Factorial(int x);

public:
    /**
     * @brief Construct a new minimum traj object
     *
     * @param Pos
     * @param Start_val
     * @param Start_acc
     * @param End_val
     * @param End_acc
     */
    minimum_traj(
        const Eigen::MatrixXd &Pos,
        const Eigen::Vector3d &Start_val,
        const Eigen::Vector3d &Start_acc,
        const Eigen::Vector3d &End_val,
        const Eigen::Vector3d &End_acc,
        const Eigen::VectorXd &Time);
    ~minimum_traj();
};

/**
 * @brief Construct a new minimum traj::minimum traj object
 *
 * @param waypoints
 * @param start_val
 * @param start_acc
 * @param end_val
 * @param end_acc
 */
minimum_traj::minimum_traj(
    const Eigen::MatrixXd &Pos,
    const Eigen::Vector3d &Start_val,
    const Eigen::Vector3d &Start_acc,
    const Eigen::Vector3d &End_val,
    const Eigen::Vector3d &End_acc,
    const Eigen::VectorXd &Time)
{
    // pos is a 3*n matrix D_Fixed is a n*3 matrix
    // for convenience， transform the pos to n*3
    Eigen::MatrixXd Pos_tmp = Pos.transpose();

    int points_num = Pos_tmp.rows();
    int seg_num = points_num - 1;

    cout << "Pos_tmp" << endl
         << Pos_tmp << endl;
    cout << "points_num" << endl
         << points_num << endl;

    // init D_fixed
    D_Fixed.resize(points_num + 4, 3);
    D_Fixed.row(0) = Pos_tmp.row(0);
    D_Fixed.row(1) = Start_val.transpose();
    D_Fixed.row(2) = Start_acc.transpose();
    D_Fixed.row(3) = Pos_tmp.row(1);
    D_Fixed.row(4) = End_val.transpose();
    D_Fixed.row(5) = End_acc.transpose();
    for (int i = 2; i < points_num; i++)
    {
        D_Fixed.row(i - 2 + 6) = Pos_tmp.row(i);
    }

    cout << "D_Fixed" << endl
         << D_Fixed << endl;

    D_Fixed.bottomRows(0) = Start_val.transpose();
    cout << "D_Fixed" << endl
         << D_Fixed << endl;

    // init A_total
    // A_total.resize(seg_num * 6, seg_num * 6);
    cout << Time << endl;
    A_total = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);
    Eigen::MatrixXd A_one = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd A_one_t = Eigen::MatrixXd::Zero(6, 6);
    A_one << 1, 1, 1, 1, 1, 1,
        0, 1, 2, 3, 4, 5,
        0, 0, 2, 6, 12, 20,
        1, 1, 1, 1, 1, 1,
        0, 1, 2, 3, 4, 5,
        0, 0, 2, 6, 12, 20;
    for (int M = 0; M < seg_num; M++)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                int order = j - i;
                if (order < 0)
                {
                    order = 0;
                }
                A_one_t(i, j) = A_one(i, j) * pow(Time(M), order);
                A_one_t(i + 3, j) = A_one(i + 3, j) * pow(Time(M + 1), order);
            }
        }
        // cout << "A_one_t" << endl
        //      << A_one_t << endl;
        A_total.block(M * 6, M * 6, 6, 6) = A_one_t;
        A_one_t = Eigen::MatrixXd::Zero(6, 6);
    }

    // init Q
    Q_total = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);
    Eigen::MatrixXd Q_one = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd Q_one_t = Eigen::MatrixXd::Zero(6, 6);
    Q_one << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 36, 144, 360,
        0, 0, 0, 144, 576, 1440,
        0, 0, 0, 360, 1440, 3600;
    for (int M = 0; M < seg_num;M++)
    {
        for (int i = 3; i < 6;i++)
        {
            for (int j = 3; j < 6;j++)
            {
                Q_one_t(i,j) = Q_one(i, j) * pow(Time(M + 1), i + j - 6);
            }
        }
    }

    // init select matrix
    C_select = Eigen::MatrixXd::Identity(seg_num * 6, seg_num * 6);
    for (int i = 0; i < 3;i++)
    {
        C_select.row(3+i).swap(C_select.row(seg_num * 6 - 2+i));
    }
    for (int i = 0; i < points_num - 2;i++)
    {
        int cnt_tmp = 6;
        if(6 * (i + 1) )
        C_select.row(6 * (i + 1)).swap(C_select.row(seg_num * 6 - 2 + i));
    }
}

minimum_traj::~minimum_traj()
{
}

int Factorial(int x)
{
    int fac = 1;
    for (int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}
#endif