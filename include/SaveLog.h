/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     SaveLog.h
  @brief    SaveLog
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
#include "BallanceController.h"
#include "Dynamic.h"
#include "FiniteStateMachine.h"
#include "MPC_solver.h"
#include "NMPC_solver.h"
#include "Robot_Parameters.h"
#include "StateEstimator.h"
#include "SwingLegController.h"
#include "TrajectoryGenerator.h"
#include "UserCmd.h"
#include "WholeBodyController.h"
#include "WholeBodyController_arm.h"
#include "common.h"
#include "EEF.h"
class SaveLog {
 private:
  /* data */
 public:
  static constexpr int RunNum =
      600000;  // 60000=60秒，仿真时间尽量不要超过这个时间值，超过后前面的数据会被舍弃掉。最多保存的数据行数
  static constexpr int VarNum = 300;  // 一行最多能保存的数据个数
  int VarCount;  // 运行一次（1ms）保存的变量的数量，即csv文件中一行的数据数量
  int RunCount;  // 运行的次数，1ms一次
  bool isSaveLog;
  deque<double>
      LogDataBuff[VarNum];  // 队列容器，用来保存运行中的所有需要保存的数据

  deque<string>
      LogDataNameBuff[VarNum];  // 队列容器，用来保存运行中的所有需要保存的数据

/**
 * @brief   存储数据
 * @param  FSM      状态机        
 * @param  SE       状态估计器        
 * @param  Trj       轨迹生成器            
 * @param  SWC        摆动腿控制器      
 * @param  MPC        模型预测控制器      
 * @param  NMPC       非线性模型预测控制器      
 * @param  VMC         虚拟模型控制器     
 * @param  WBC         全身控制器     
 * @param  Dyn         动力学     
 */
  void SaveData(FiniteStateMachine& FSM, StateEstimator& SE,
                       TrajectoryGenerator& Trj,
                       SwingLegController& SLC, MPC_solver& MPC,
                       NMPC_solver& NMPC, BallanceController& VMC,
                       WholeBodyController& WBC, Robot_Dynamic& Dyn,EEF& eef);
  /**
   * 用于添加矩阵数据
   */
  template <typename T>
  void append(const DenseBase<T>& mat,const string Name);

  /**
   * 用于添加单个数据
   */
  template <typename T>
  void append_c(const T& var,const string Name);

  /**
   * @brief SaveLog
   * 将LogDataBuff中的所有数据输出到文件中，保存下来。只在空格键按下后运行一次
   */
  void SaveData2Log();
  SaveLog(/* args */);
  ~SaveLog();
};
