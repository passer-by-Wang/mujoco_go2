/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     ObjectController.h
  @brief    ObjectController
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
#include "common.h"

class ObjectController{
    public:
        Matrix3d object_k_p;
        Matrix3d object_k_v;
        Matrix3d object_k_theta;
        Matrix3d object_k_w;
        double object_mass;
        Vector3d gravity;
        void computeObjectForces(const Matrix<double, robot_state_dim + object_state_dim, 1> &state,
                            const Matrix<double, robot_state_dim + object_state_dim, 1> &state_d,const Vector3d &Object_to_Body_Pos);
        Matrix<double, 6,1> Object_Forces;
        Matrix<double, 6,1> Object_to_Body_Forces;
        ObjectController();
        ~ObjectController();
    private:
};