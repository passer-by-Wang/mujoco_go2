/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     ClassManager.h
  @brief    Manager
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
#include <yaml-cpp/yaml.h>

#include "BallanceController.h"
#include "Dynamic.h"
#include "FiniteStateMachine.h"
#include "Info.h"
#include "MPC_solver.h"
#include "NMPC_solver.h"
#include "NMPC_solver_DRBM.h"
#include "NMPC_solver_DRBM_rpy.h"
#include "Robot_Parameters.h"
#include "SaveLog.h"
#include "StateEstimator.h"
#include "SwingLegController.h"
#include "TrajectoryGenerator.h"
#include "UserCmd.h"
#include "WholeBodyController.h"
#include "WholeBodyController_arm.h"
#include "WholeBodyPlanner.h"
#include "common.h"
#include "test.h"
#include "ObjectController.h"
#include "NMPC_solver_rpy.h"
#include "Dynamic_arm.h"
#include "MujocoInterface.h"

#include "EEF.h"

class ClassManager {
 private:
  /* data */
 public:
  /**
   * @brief 从配置文件中读取参数
   */
  void Configure_para();

  /**
   * @brief 进行
   */
  void run(MujocoJoint *bodyjoint[4][3],MujocoImu *imu,
                           MujocoForceSensor *ForceSensor[4]);

  /**
   * @brief
   * 执行计算结果，如果是nmpc且还未进入计算循环，那么就保持初始位置，否则执行计算扭矩
   * @param  isStop          是否退出
   * @param  Robot_controller_switch    执行控制器选择
   * @param  isStart          nmpc是否进入主循环
   * @param  BodyJointTorque      计算得到的目标关节扭矩
   * @param  initPos          初始位置
   */
  void Execute_Instructions(MujocoJoint *bodyjoint[4][3],
                            const bool& isStop,
                            const int& Robot_controller_switch,
                            const bool& isStart,
                            const Mat43& BodyJointTorque,
                            const Mat43& BodyinitPos);

  /**
   * @brief   控制器选择
   * @param  _Robot_controller_switch   执行控制器选择
   * @param  _RotationMatrix  机身姿态旋转矩阵
   * @param  LegPhase     足端接触序列
   * @param  _vmc_LegForces   vmc计算足端力结果
   * @param  _SLC_LegForces   slc计算足端力结果
   * @param  _MPC_LegForces   mpc计算足端力结果
   * @param  _NMPC_LegForces  nmpc计算足端力结果
   * @param  _WBC_LegForces   wbc使用的足端力
   */
  void controllerSwitch2Use(
      const int& _Robot_controller_switch, const Matrix3d& _RotationMatrix,
      const Vector4i& LegPhase, const Mat43& _vmc_LegForces,
      const Mat43& _SLC_LegForces, const Mat43& _MPC_LegForces,
      const Mat43& _NMPC_LegForces, const Mat43& _NMPC_RPY_LegForces,const Mat43& _NMPC_D_LegForces,
      const Mat43& _NMPC_D_RPY_LegForces,Mat43& _WBC_LegForces);
  int Robot_controller_switch;
  int count;
  double time;
  BallanceController go2_vmc;
  Robot_Dynamic go2_dynamic;
  Robot_Dynamic_arm go2_dynamic_arm;
  FiniteStateMachine go2_FSM;
  MPC_solver go2_mpc;
  NMPC_solver go2_nmpc;
  NMPC_solver_rpy go2_nmpc_rpy;
  NMPC_solver_DRBM go2_nmpc_DRBM;
  NMPC_solver_DRBM_rpy go2_nmpc_DRBM_rpy;
  Robot_Parameters go2_Para;
  SaveLog go2_Log;
  StateEstimator go2_SE;
  SwingLegController go2_SLC;
  TrajectoryGenerator go2_Trj;
  UserCmd go2_cmd;
  WholeBodyController go2_wbc;
  // WholeBodyPlanner BQR3_wbp;
  Info go2_Info;
  test go2_test;
  ObjectController go2_OC;
  WholeBodyController_arm go2_wbc_arm;
  MujocoInterface go2_Mjc;
  EEF go2_eef;

  YAML::Node go2_para_config;
  ClassManager();
  ~ClassManager();
};