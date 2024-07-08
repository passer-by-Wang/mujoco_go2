/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     FiniteStateMachine.h
  @brief    FiniteStateMachine
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
#include "StateEstimator.h"
#include "common.h"

class GaitSchedule {
 public:
  Matrix<int, Eigen::Dynamic, 4> GaitSequence;  // 步态序列
  Vector3d SwingHeight;                  // 抬腿高度
  double stand_T;                        // 支撑相时间
  double swing_T;                        // 摆动相时间
  Vector3d StandHeight;                  // 站立高度
};

class FiniteStateMachine {
 private:
  /* data */
 public:
  double time;
  bool ischangephase;
  bool ischangegait;
  bool ischangegait1;

  FiniteStateMachine(/* args */);

  /**
   * @brief 设置步态序列
   * @param  Gait   步态标志位          
   * @param  SE      状态估计器         
   */
  void setLegPhase(int& Gait, StateEstimator& SE);

  /**
   * @brief 更新接触序列
   */
  
  void PhaseUpdate();
  int GaitTag;  // 步态标签
  int currentGaitTag;
  int phase;
  Matrix<int, 4, 1> LegPhase;  // 四条腿的状态，处于支撑相（1）还是摆动相（0）
  GaitSchedule stand, trot, bound, flytrot, pace, walk;
  GaitSchedule current_gait;
  int lastGaitTag;
  ~FiniteStateMachine();
};
