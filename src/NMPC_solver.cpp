/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     NMPC_solver.cpp
  @brief    NMPC_solver
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
#include "../include/NMPC_solver.h"

NMPC_solver::NMPC_solver(/* args */) {
  LegForces.setZero();
  init_flag = 1;
}

void NMPC_solver::run(const Matrix<double, robot_state_dim, 1> &state,
                      const Matrix<double, robot_state_dim, 1> &state_d,
                      const Mat43 &footpos, const Vector4i &contact_state,const Matrix<double, 6, 1> arm2body_Forces) {}

NMPC_solver::~NMPC_solver() {}
