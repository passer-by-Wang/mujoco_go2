/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     Robot_Parameters.h
  @brief    Robot_Parameters
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
#include <atomic>

#include "common.h"
class Robot_Parameters {
 private:
  /* data */
 public:
  double BodyLength;
  double BodyWidth;
  double BodyMass;
  double abd_offset;
  double Thigh_Length;
  double Calf_Length;
  double friction_coeff;
  Matrix3d Inertia;
  double grav;
  double BQR3_mass;

  Robot_Parameters(/* args */);
  ~Robot_Parameters();
};
