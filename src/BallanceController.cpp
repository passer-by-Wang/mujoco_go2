/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     BallanceController.cpp
  @brief    vmc
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
#include "../include/BallanceController.h"
BallanceController::BallanceController(/* args */)
{
  S_track.setIdentity();
  S_min_f.setIdentity();
  // A_eq.setZero();
  // b_eq.setZero();
  A_ieq.setZero();
  b_ieq.setZero();
  Body_gravity.setZero();
  ContactLegForces.setZero();
}

/**
 * @brief:
 * @return {*}
 */
void BallanceController::computeContactForces(Robot_Parameters &Para,
                                              const Vector4i &LegPhase,
                                              const Mat43 &ToePos_wB,
                                              Matrix<double, 6, 1> &Gen_vf_d)
{
  cal_Related_matrix(Para, LegPhase, ToePos_wB, Gen_vf_d);

  solve_Leg_Forces(LegPhase);

  // printinfocolor(ToePos_wB,boldblue);
}

void BallanceController::cal_Related_matrix(Robot_Parameters &Para,
                                            const Vector4i &LegPhase,
                                            const Mat43 &ToePos_wB,
                                            Matrix<double, 6, 1> &Gen_vf_d)
{
  k = 0;
  int sum = LegPhase.sum();
  b_ieq.setZero();
  S_min_f.setIdentity(sum * 3, sum * 3);

  A.setZero();
  b.setZero();
  A_ieq.setZero(sum * 6, sum * 3);
  b_ieq.setZero(sum * 6, 1);
  A_eq.resize(0, sum * 3);

  A_ieq_sub << 0, 0, 1, 0, 0, -1, 1, 0, -0.5 * sqrt(2) * u, -1, 0,
      -0.5 * sqrt(2) * u, 0, 1, -0.5 * sqrt(2) * u, 0, -1, -0.5 * sqrt(2) * u;
  for (int i = 0; i < 4; i++)
  {
    if (LegPhase[i] == 1)
    {
      A.block<3, 3>(0, 3 * k) = I;
      A.block<3, 3>(3, 3 * k) = cross_product(ToePos_wB.row(i).transpose());
      A_ieq.block<6, 3>(6 * k, 3 * k) = A_ieq_sub;
      b_ieq(6 * k) = f_max;
      b_ieq(6 * k + 1) = -f_min;
      k++;
    }
  }
  Body_gravity(2) = Para.BQR3_mass * Para.grav;
  // b = Gen_vf - Body_gravity;
  b = Gen_vf_d - Body_gravity;
  QP_H =
      A.block(0, 0, 6, k * 3).transpose() * S_track * A.block(0, 0, 6, k * 3) +
      alpha_W * S_min_f;
  QP_g = -A.block(0, 0, 6, k * 3).transpose() * S_track.transpose() * b;
}

void BallanceController::solve_Leg_Forces(const Vector4i &LegPhase)
{
  VectorXd F_ext;   // 优化得到的足端力
  QuadProgDense qp; // 构造qp问题
  qp.problem(k * 3, 0, k * 6);
  int flag = qp.solve(QP_H, QP_g, A_eq, b_eq, A_ieq, b_ieq);
  F_ext = qp.result();

  int p = 0;
  ContactLegForces.setZero();
  for (int i = 0; i < 4; i++)
    if (LegPhase(i) == 1)
    {
      ContactLegForces.row(i) << F_ext[p + 0], F_ext[p + 1], F_ext[p + 2];
      p += 3;
    }
  p = 0;
  // printinfo(ContactLegForces);
}

BallanceController::~BallanceController() {}