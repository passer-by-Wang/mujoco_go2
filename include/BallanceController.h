/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     BallanceController.h
  @brief    VMC
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
#include "../external/quadprog/QuadProg.h"
#include "Robot_Parameters.h"
#include "../include/common.h"
using namespace Eigen;
class BallanceController {
 private:
  /* data */
 public:
  Matrix<double, 6, 12> A;
  Matrix<double, 6, 1> b;
  Matrix<double, 6, 1> Body_gravity;
  Matrix<double, 6, 6> S_track;
  Matrix<double, Eigen::Dynamic, Eigen::Dynamic> S_min_f;
  Matrix<double, Eigen::Dynamic, Eigen::Dynamic> QP_H;
  Matrix<double, Eigen::Dynamic, 1> QP_g;
  Matrix<double, 0, Eigen::Dynamic> A_eq;
  Matrix<double, 0, 1> b_eq;
  Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_ieq;
  Matrix<double, Eigen::Dynamic, 1> b_ieq;
  Matrix<double, 6, 3> A_ieq_sub;
  int k;

  double f_min;
  double f_max;
  double alpha_W;
  double u;  // 摩擦系数
  Mat43 ContactLegForces;

  /**
   * @brief 虚拟模型控制计算足端力
   * @param  Para         参数器
   * @param  LegPhase     足端接触序列
   * @param  ToePos_wB    足端相对于质心的位置在世界坐标系下的坐标
   * @param  Gen_vf_d     广义质心虚拟力
   */
  void computeContactForces(Robot_Parameters& Para, const Vector4i& LegPhase,
                            const Mat43& ToePos_wB,
                            Matrix<double, 6, 1>& Gen_vf_d);

  /**
   * @brief 计算相关矩阵，H，g，Ax ≤ b
   * @param  Para         参数器
   * @param  LegPhase     足端接触序列
   * @param  ToePos_wB    足端相对于质心的位置在世界坐标系下的坐标
   * @param  Gen_vf_d     广义质心虚拟力
   */
  void cal_Related_matrix(Robot_Parameters& Para, const Vector4i& LegPhase,
                          const Mat43& ToePos_wB,
                          Matrix<double, 6, 1>& Gen_vf_d);
  /**
   * @brief 求解足端力
   * @param  LegPhase   足端接触序列
   */
  void solve_Leg_Forces(const Vector4i& LegPhase);

  BallanceController(/* args */);

  ~BallanceController();
};