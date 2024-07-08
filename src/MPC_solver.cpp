/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     MPC_solver.cpp
  @brief    MPC_solver
  Details.
  @author   Hua Wang
  @email    wangh@bit.edu.cn
  @version  1.0.0
  @date     2023/12/25
  @license  NONE

-----------------------------------------------------------------------------
  Remark         : Description
-----------------------------------------------------------------------------
  Change History :
  <Date>     | <Version> | <Author>       | <Description>
-----------------------------------------------------------------------------
  2023/12/25 | 1.0.0     | Hua Wang       | Create file
-----------------------------------------------------------------------------

*****************************************************************************/

#include "../include/MPC_solver.h"

MPC_solver::MPC_solver(/* args */)
{
  LegForces.setZero();
  initflag = 1;
}

void MPC_solver::computeContactForces(
    Robot_Parameters &Para, const Matrix3d &Rz, const Vector4i &LegPhase,
    const Mat43 &ToePos_wB, const Matrix3d &I_w,
    const Matrix<double, robot_state_dim, 1> &x0,
    const Matrix<double, robot_state_dim * mpc_Horizon, 1> &X_d, const bool &isStop)
{
  if (mpcEnable && initflag)
  {
    initflag = 0;
    thread thread_mpc =
        mpc_thread(Para, Rz, LegPhase, ToePos_wB, I_w, x0, X_d, isStop);
    thread_mpc.detach();
    cout << "MPC thread is created!" << endl;
  }
}

void MPC_solver::mpcLoop(Robot_Parameters &Para, const Matrix3d &Rz,
                         const Vector4i &LegPhase, const Mat43 &ToePos_wB,
                         const Matrix3d &I_w,
                         const Matrix<double, robot_state_dim, 1> &x0,
                         const Matrix<double, robot_state_dim * mpc_Horizon, 1> &X_d,
                         const bool &isStop)
{
  while (1)
  {
    defineContinuousSystem(Para, Rz, LegPhase, ToePos_wB, I_w);

    Discretization(Ac, Bc, dt_MPC);

    computeQPform(Ad, Bd);

    standardization(A_qp, B_qp, x0, X_d);

    defineConstraints(friction_coeff, f_max, f_min);

    qpOASES_Solve(H_qp, g_qp, C_qp, d_qp, LegPhase);

    if (isStop)
    {
      cout << "[MPC solver] Stop!" << endl;
      break;
    }
  }
}

void MPC_solver::defineContinuousSystem(Robot_Parameters &Para,
                                        const Matrix3d &Rz,
                                        const Vector4i &LegPhase,
                                        const Mat43 &ToePos_wB,
                                        const Matrix3d &I_w)
{
  Ac.setZero();
  Ac.block<3, 3>(0, 6) = Rz.transpose();
  Ac.block<3, 3>(3, 9).setIdentity();
  Ac(11, 12) = 1.0;

  contact_num = LegPhase.sum();
  Bc.setZero(robot_state_dim, 3 * contact_num);
  int num = 0;
  for (int i = 0; i < 4; i++)
  {
    if (LegPhase[i] == 1)
    {
      Bc.block<3, 3>(6, 3 * num) =
          I_w.inverse() * cross_product(ToePos_wB.row(i).transpose());
      Bc.block<3, 3>(9, 3 * num) = I / Para.BQR3_mass;
      num += 1;
    }
  }
  // printinfo(Ac);
  // printinfo(Bc);
}

void MPC_solver::Discretization(const Mat_dim &_Ac,
                                const Matrix<double, robot_state_dim, Eigen::Dynamic> &_Bc,
                                double _dt)
{
  Bd.resize(robot_state_dim, contact_num * 3);
  if (mpc_discrete_switch == 1)
  {
    Matrix<double, Eigen::Dynamic, Eigen::Dynamic> abmat(robot_state_dim + 3 * contact_num,
                                                         robot_state_dim + 3 * contact_num);
    abmat.setZero();
    abmat.block<robot_state_dim, robot_state_dim>(0, 0) = _Ac * _dt;
    abmat.block(0, robot_state_dim, robot_state_dim, 3 * contact_num) = _Bc * _dt;
    Matrix<double, Eigen::Dynamic, Eigen::Dynamic> abmat_exp = abmat.exp();
    Ad = abmat_exp.block<robot_state_dim, robot_state_dim>(0, 0);
    Bd = abmat_exp.block(0, robot_state_dim, robot_state_dim, 3 * contact_num);
  }

  if (mpc_discrete_switch == 2)
  {
    Ad = (_Ac * _dt + MatrixXd::Identity(robot_state_dim, robot_state_dim));
    Bd = _Bc * _dt;
  }
}

void MPC_solver::computeQPform(const Mat_dim &_Ad,
                               const Matrix<double, robot_state_dim, Eigen::Dynamic> &_Bd)
{
  A_qp.block<robot_state_dim, robot_state_dim>(0, 0) = _Ad;
  for (int i = 1; i < mpc_Horizon; i++)
  {
    A_qp.block<robot_state_dim, robot_state_dim>(robot_state_dim * i, 0) =
        A_qp.block<robot_state_dim, robot_state_dim>(robot_state_dim * (i - 1), 0) * _Ad;
  }
  B_qp.setZero().resize(robot_state_dim * mpc_Horizon, 3 * contact_num * mpc_Horizon);
  for (int i = 0; i < mpc_Horizon; i++)
  {
    for (int j = 0; j < mpc_Horizon; j++)
    {
      if (i == j) // 主对角线
      {
        B_qp.block(i * robot_state_dim, j * 3 * contact_num, robot_state_dim,
                   3 * contact_num) = _Bd;
      }
      else if (i > j) // 下三角区域
      {
        B_qp.block(i * robot_state_dim, j * 3 * contact_num, robot_state_dim,
                   3 * contact_num) =
            A_qp.block<robot_state_dim, robot_state_dim>((i - j - 1) * robot_state_dim, 0) * _Bd;
      }
    }
  }
}

void MPC_solver::standardization(
    const Matrix<double, robot_state_dim * mpc_Horizon, robot_state_dim> &A_qp,
    const Matrix<double, robot_state_dim * mpc_Horizon, Eigen::Dynamic> &B_qp,
    const Matrix<double, robot_state_dim, 1> &x0,
    const Matrix<double, robot_state_dim * mpc_Horizon, 1> &X_d)
{
  // R_qp.setIdentity(3 * contact_num * mpc_Horizon,
  //                  3 * contact_num * mpc_Horizon);
  // R_qp = weight_f * R_qp.setIdentity(3 * contact_num * mpc_Horizon,
  //                                    3 * contact_num * mpc_Horizon);
  Q_qp.setZero().resize(robot_state_dim * mpc_Horizon, robot_state_dim * mpc_Horizon);
  for (int i = 0; i < mpc_Horizon; i++)
  {
    Q_qp.block<robot_state_dim, robot_state_dim>(i * robot_state_dim, i * robot_state_dim) =
        Q_qp_sub.asDiagonal();
  }
  R_qp.setZero().resize(3 * contact_num * mpc_Horizon, 3 * contact_num * mpc_Horizon);
  for (int i = 0; i < contact_num * mpc_Horizon; i++)
  {
    R_qp.block<3, 3>(i * 3, i * 3) = weight_f * Q_qp.asDiagonal();
  }

  H_qp.resize(3 * contact_num * mpc_Horizon, 3 * contact_num * mpc_Horizon);
  g_qp.resize(3 * contact_num * mpc_Horizon, 1);
  H_qp = B_qp.transpose() * Q_qp * B_qp + R_qp;
  g_qp = B_qp.transpose() * Q_qp * (A_qp * x0 - X_d);
}

void MPC_solver::defineConstraints(double &u, double &fmax, double &fmin)
{
  C_qp.setZero().resize(6 * contact_num * mpc_Horizon,
                        3 * contact_num * mpc_Horizon);
  d_qp.resize(6 * contact_num * mpc_Horizon);
  C1 << 0, 0, 1, 0, 0, -1, 1, 0, -u, 0, 1, -u, -1, 0, -u, 0, -1, -u;
  d1 << fmax, -fmin, 0, 0, 0, 0;
  for (int i = 0; i < contact_num * mpc_Horizon; i++)
  {
    C_qp.block<6, 3>(i * 6, i * 3) = C1;
    d_qp.block<6, 1>(i * 6, 0) = d1;
  }
}

void MPC_solver::qpOASES_Solve(
    const MatrixXd &_H_qp, const MatrixXd &_g_qp,
    const Matrix<double, Eigen::Dynamic, Eigen::Dynamic, RowMajor> &_C_qp,
    const Matrix<double, Eigen::Dynamic, 1> &_d_qp, const Vector4i &LegPhase)
{
  if (contact_num == 0) // 没有腿支撑时，处于飞相状态不用计算足端力
    return;

  // dimension：3nh x 3nh
  qpOASES::real_t
      H_qpoases[3 * contact_num * mpc_Horizon * 3 * contact_num * mpc_Horizon];
  // dimension：3nh x 1
  qpOASES::real_t g_qpoases[3 * contact_num * mpc_Horizon * 1];
  // 摩擦约束的系数阵C
  qpOASES::real_t
      C_qpoases[6 * contact_num * mpc_Horizon * 3 * contact_num * mpc_Horizon];
  // 摩擦约束的上界
  qpOASES::real_t d_qpoases[6 * contact_num * mpc_Horizon * 1];

  memcpy(H_qpoases, H_qp.data(), 8 * (H_qp.size()));
  memcpy(C_qpoases, C_qp.data(), 8 * (C_qp.size()));
  memcpy(g_qpoases, g_qp.data(), 8 * (g_qp.size()));
  memcpy(d_qpoases, d_qp.data(), 8 * (d_qp.size()));

  qpOASES::QProblem MPC(3 * contact_num * mpc_Horizon,
                        6 * contact_num * mpc_Horizon);
  qpOASES::Options mpc_options;
  mpc_options.setToMPC();
  mpc_options.printLevel = qpOASES::PL_NONE;
  MPC.setOptions(mpc_options);
  qpOASES::int_t iter_max = 200;
  qpOASES::real_t cpu_time = 3 * dt_MPC;

  // example.init（H，g，A，lb，ub，lbA，ubA，nWSR，cputime）
  //  H：hessian矩阵
  //  g：梯度向量
  //  A：约束矩阵
  //  lb ub：自变量的下边界和上边界向量
  //  lbA ubA：上下限约束向量
  //
  // 这个向量中不仅包含了不等式，也包括了等式的约束,将上限和下限设置为相同数值即可表示为等式约束！
  //  nWSR： 最大迭代次数
  //  cputime：如果输入不为空，就会输出整个初始化和求解所花的时间。 Input:
  //  Maximum CPU time allowed for QP initialisation. Output: CPU time spent
  // for
  //  QP initialisation (if pointer passed). */
  //
  // 如果哪一项是没有的(
  //     例：没有上边界)，就看可以传入一个空指针NULL。解完后所有的向量需要自己释放内存。但是矩阵H
  // A不需要。 所有init函数都会对所有向量参数进行深拷贝（deep
  // copy），因此必须自己释放内存
  // 矩阵参数H和A不会被深拷贝，因此连续调用qpOASES时不能更改它们

  int rval = MPC.init(H_qpoases, g_qpoases, C_qpoases, NULL, NULL, NULL,
                      d_qpoases, iter_max, &cpu_time);

  if (rval != qpOASES::SUCCESSFUL_RETURN) // 如果没有初始化qp,则报错
  {
    printf("failed to init!\n");
    if (rval == qpOASES::RET_MAX_NWSR_REACHED) // 达到最大迭代次数
    {
      printf("RET_MAX_NWSR_REACHED\n");
    }
  }

  qpOASES::real_t xOpt[3 * contact_num * mpc_Horizon];
  MPC.getPrimalSolution(xOpt);

  int k = 0;
  LegForces.setZero();
  for (int i = 0; i < 4; i++)
  {
    if (LegPhase(i) == 1)
    {
      LegForces.row(i) << xOpt[k], xOpt[k + 1], xOpt[k + 2];
      k += 3;
    }
  }
}

// 暂时未使用
void MPC_solver::comf(Matrix<double, Eigen::Dynamic, 1> &f, Matrix3d &I_w)
{
  Matrix<double, 6, Eigen::Dynamic> a;
  a.resize(6, Bc.cols());
  a = Bc.block(6, 0, 6, Bc.cols());

  for (int i = 0; i < int(Bc.cols() / 3); i++)
  {
    a.block(0, i * 3, 3, 3) = I_w * a.block(0, i * 3, 3, 3).eval();
    a.block(3, i * 3, 3, 3).setIdentity();
  }
  mpcCoMF = a * f;
}

MPC_solver::~MPC_solver() {}