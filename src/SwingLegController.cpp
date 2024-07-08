/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     SwingLegController.cpp
  @brief    SwingLegController
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
#include "../include/SwingLegController.h"
SwingLegController::SwingLegController(/* args */) {
  SwingLegForce.setZero();
  k_p_toe.setZero();
  k_v_toe.setZero();
}

void SwingLegController::computeSwingLegForces(const Mat43& ToePos_bH_d,
                                               const Mat43& ToePos_bH,
                                               const Mat43& ToeVel_bH_d,
                                               const Mat43& ToeVel_bH) {
  for (int i = 0; i < 4; i++) {
    SwingLegForce.row(i) = (ToePos_bH_d - ToePos_bH).row(i) * k_p_toe +
                           (ToeVel_bH_d - ToeVel_bH).row(i) * k_v_toe;
  }
}

SwingLegController::~SwingLegController() {}