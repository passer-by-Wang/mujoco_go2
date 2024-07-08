/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     NMPC_solver_DRBM_rpy.cpp
  @brief    NMPC_solver_DRBM_rpy
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
#include "../include/NMPC_solver_DRBM_rpy.h"

NMPC_solver_DRBM_rpy::NMPC_solver_DRBM_rpy(/* args */)
{
  LegForces.setZero();
  init_flag = 1;
}

void NMPC_solver_DRBM_rpy::run(const Matrix<double, robot_state_dim + object_state_dim, 1> &state,
                               const Matrix<double, robot_state_dim + object_state_dim, 1> &state_d,
                               const Mat43 &footpos, const Vector3d &objectpos, const Vector4i &contact_state, const Matrix<double, 6, 1> arm2body_Forces)
{}

NMPC_solver_DRBM_rpy::~NMPC_solver_DRBM_rpy() {}
