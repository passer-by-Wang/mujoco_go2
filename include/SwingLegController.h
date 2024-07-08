/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     SwingLegController.h
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
#pragma once
#include "FiniteStateMachine.h"
#include "common.h"
class SwingLegController {
 private:
  /* data */
 public:
  Mat43 SwingLegForce;
  Matrix3d k_p_toe;
  Matrix3d k_v_toe;

  /**
   * @brief   计算摆动腿足端力
   * @param  ToePos_bH_d    目标足端位置   
   * @param  ToePos_bH      当前足端位置  
   * @param  ToeVel_bH_d     目标足端速度 
   * @param  ToeVel_bH        当前足端速度
   */
  void computeSwingLegForces(const Mat43& ToePos_bH_d, const Mat43& ToePos_bH,
                             const Mat43& ToeVel_bH_d, const Mat43& ToeVel_bH);
  SwingLegController(/* args */);
  ~SwingLegController();
};
