/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     WholeBodyPlanner.h
  @brief    WholeBodyPlanner
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
  2024/3/4   | 1.0.0     | Hua Wang       | Create file
-----------------------------------------------------------------------------

*****************************************************************************/
#pragma once

#include "common.h"
#include "nlopt.h"
class WholeBodyPlanner
{
private:
  /* data */
public:
  nlopt_opt opter;
  int iter_num;
  double tol;
  double lb[12];
  double ub[12];
  double x[12];
  int number_variables;
  double cost;
  int iteration_count;

  Vector3d p_body;
  Vector3d Q_body;
  Vector3d p_arm;
  Vector3d Q_arm;

  double const_height;

  static double objectFunction(unsigned n, const double *x, double *grad, void *data);

  static double equalityConstraints_pos_x(unsigned n, const double *x, double *grad, void *data);

  static double equalityConstraints_pos_y(unsigned n, const double *x, double *grad, void *data);

  static double equalityConstraints_pos_z(unsigned n, const double *x, double *grad, void *data);

  static double equalityConstraints_orien_x(unsigned n, const double *x, double *grad, void *data);

  static double equalityConstraints_orien_y(unsigned n, const double *x, double *grad, void *data);

  static double equalityConstraints_orien_z(unsigned n, const double *x, double *grad, void *data);

  static double inequalityConstraints_arm_upper(unsigned n, const double *x, double *grad, void *data);

  static double inequalityConstraints_arm_lower(unsigned n, const double *x, double *grad, void *data);

  void config_target_state(const Vector3d &_p_arm_wd, const Vector3d &_Q_arm_wd, const Vector3d &_p_body
  , const Vector3d &_Q_body,const Vector3d &_p_arm , const Vector3d &_Q__arm);

  bool opp_solve();

  void create_QCP_Planner();

  bool isCreatedPlanner;

  Vector3d p_arm_w_d;
  Vector3d Q_arm_w_d;
  Vector3d weight_p_body;
  Vector3d weight_Q_body;
  Vector3d weight_p_arm;
  Vector3d weight_Q_arm;
  WholeBodyPlanner(/* args */);
  ~WholeBodyPlanner();
};
