#include "../include/minimum_traj.hpp"

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
    // pos is a 3*n matrix D_fixed is a n*3 matrix
    // for convenience， transform the pos to n*3
    Eigen::MatrixXd Pos_tmp = Pos.transpose();

    poly_order = 5;
    poly_coff_num = poly_order + 1;
    physical_order = 3;

    points_num = Time.size() + 1;
    seg_num = Time.size();

    fixed_coff_num = 3 * 2 + (points_num - 2) * 2;
    all_coff_num = 3 * 2 * seg_num;

    D_fixed = Eigen::MatrixXd::Zero(fixed_coff_num, 3);
    D_P_optimal = Eigen::MatrixXd::Zero(all_coff_num - fixed_coff_num, 3);
    D_total = Eigen::MatrixXd::Zero(all_coff_num, 3);
    D_total_selected = Eigen::MatrixXd::Zero(all_coff_num, 3);

    A_total = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);
    A_one = Eigen::MatrixXd::Zero(6, 6);
    A_one_t = Eigen::MatrixXd::Zero(6, 6);

    Q_total = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);
    Q_one = Eigen::MatrixXd::Zero(6, 6);
    Q_one_t = Eigen::MatrixXd::Zero(6, 6);

    C_select_T = Eigen::MatrixXd::Zero(2 * physical_order * seg_num, 2 * physical_order * seg_num);
    R = Eigen::MatrixXd::Zero(2 * physical_order * seg_num, 2 * physical_order * seg_num);

    // init D_fixed
    // 3 for x,y,z
    D_fixed.row(0) = Pos_tmp.row(0);
    D_fixed.row(1) = Start_val.transpose();
    D_fixed.row(2) = Start_acc.transpose();
    D_fixed.row(3) = Pos_tmp.row(points_num - 1);
    D_fixed.row(4) = End_val.transpose();
    D_fixed.row(5) = End_acc.transpose();
    for (int i = 0; i < points_num - 2; i++)
    {
        D_fixed.row(2 * i + 6) = Pos_tmp.row(i + 1);
        D_fixed.row(2 * i + 6 + 1) = Pos_tmp.row(i + 1);
    }

    D_total.row(0) = Pos_tmp.row(0);
    D_total.row(1) = Start_val.transpose();
    D_total.row(2) = Start_acc.transpose();
    D_total.row(all_coff_num - 3) = Pos_tmp.row(points_num - 1);
    D_total.row(all_coff_num - 2) = End_val.transpose();
    D_total.row(all_coff_num - 1) = End_acc.transpose();
    for (int i = 0; i < points_num - 2; i++)
    {
        D_total.row(2 * physical_order * i + 3) = Pos_tmp.row(i + 1);
        D_total.row(2 * physical_order * i + 3 + 3) = Pos_tmp.row(i + 1);
    }

    // init A_total
    // A_total.resize(seg_num * 6, seg_num * 6);
    // caluate A by time segments
    A_one << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0,
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
                A_one_t(i, j) = A_one(i, j) * pow(0, order);
                A_one_t(i + 3, j) = A_one(i + 3, j) * pow(Time(M), order);
            }
        }
        // cout << "A_one_t" << endl
        //      << A_one_t << endl;
        A_total.block(M * 6, M * 6, 6, 6) = A_one_t;
        A_one_t = Eigen::MatrixXd::Zero(6, 6);
    }

    // init Q
    Q_one << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 36, 72, 120,
        0, 0, 0, 72, 192, 360,
        0, 0, 0, 120, 360, 720;
    for (int M = 0; M < seg_num; M++)
    {
        for (int i = 3; i < 6; i++)
        {
            for (int j = 3; j < 6; j++)
            {
                Q_one_t(i, j) = Q_one(i, j) * pow(Time(M), i + j - 6);
            }
        }
        Q_total.block(M * 6, M * 6, 6, 6) = Q_one_t;
        Q_one_t = Eigen::MatrixXd::Zero(6, 6);
    }

    // init select matrix
    if (!Cal_C_select_T(C_select_T))
    {
        return;
    }

    D_total_selected = C_select_T * D_total;

    cout << endl
         << D_total_selected << endl;

    R = C_select_T.transpose() * A_total.transpose().inverse() * Q_total * A_total.inverse() * C_select_T;

    R_FF = R.block(0, 0, fixed_coff_num, fixed_coff_num);
    R_FP = R.block(0, fixed_coff_num, fixed_coff_num, R.cols() - fixed_coff_num);
    R_PF = R.block(fixed_coff_num, 0, R.rows() - fixed_coff_num, fixed_coff_num);
    R_PP = R.block(fixed_coff_num, fixed_coff_num, R.rows() - fixed_coff_num, R.rows() - fixed_coff_num);

    D_total_selected.block(0, 0, fixed_coff_num, 3) = D_fixed;
    D_total_selected.block(fixed_coff_num, 0, all_coff_num - fixed_coff_num, 3) = D_P_optimal;

    Poly_coff_total = A_total.inverse() * C_select_T.transpose() * D_total_selected;
}

minimum_traj::~minimum_traj()
{
}

bool minimum_traj::Cal_C_select_T(Eigen::MatrixXd &C_T)
{
    C_T = Eigen::MatrixXd::Zero(2 * physical_order * seg_num, 2 * physical_order * seg_num);
    for (int i = 0; i < 2 * physical_order * seg_num; i++)
    {
        static int tmp1 = 0;
        static int tmp2 = 0;
        static int tmp3 = 0;
        // start point vel acc
        if (i < physical_order)
        {
            C_T(i, i) = 1;
            continue;
        }
        // end point vel acc
        if ((i >= physical_order) && (i < 2 * physical_order))
        {
            C_T(i, (C_T.cols() - 3) + (i - 3)) = 1;
            continue;
        }
        // waypoints
        if ((i >= 2 * physical_order) && (i < fixed_coff_num))
        {
            tmp1++;
            C_T(i, tmp1 * physical_order) = 1;
            continue;
        }
        // waypoint vel
        if ((i >= fixed_coff_num) && (i < fixed_coff_num + (all_coff_num - fixed_coff_num) / 2))
        {
            tmp2++;
            C_T(i, 1 + tmp2 * 3) = 1;
            continue;
        }
        // waypoint acc
        if (i >= fixed_coff_num + (all_coff_num - fixed_coff_num) / 2)
        {
            tmp3++;
            C_T(i, 2 + tmp3 * 3) = 1;
            continue;
        }
    }
    // cout << C_T << endl;
    if (C_T.determinant() == 0)
    {
        cout << "C_select is error";
        return false;
    }
    return true;
}

int minimum_traj::Factorial(int x)
{
    int fac = 1;
    for (int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}