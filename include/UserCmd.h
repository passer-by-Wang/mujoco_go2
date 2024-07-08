/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     UserCmd.h
  @brief    UserCmd
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
#include "TrajectoryGenerator.h"
#include "common.h"
class UserCmd {
 private:
  /* data */
 public:
  int KeyCmd;
  int LastKeyCmd;
  bool ispressed;
  bool isStop;
  bool Record_flag;  // 记录仿真数据标志位

  /**
   * @brief 根据输入映射下层目标
   * @param  Trj  轨迹生成器
   * @param  FSM  状态机
   */
  void GetUserInput(TrajectoryGenerator& Trj, FiniteStateMachine& FSM);
  UserCmd(/* args */);
  ~UserCmd();
};
