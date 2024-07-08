/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     WholeBodyController.h
  @brief    WholeBodyController
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
#include "FiniteStateMachine.h"
#include "HoQp.h"
#include "MPC_solver.h"
#include "NMPC_solver.h"
#include "QuadProg++.hh"
#include "Robot_Parameters.h"
#include "StateEstimator.h"
#include "SwingLegController.h"
#include "TrajectoryGenerator.h"
#include "common.h"
#include "eigen3/Eigen/Dense"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"  //计算正运动学
#include "pinocchio/algorithm/rnea.hpp"  //Recursive Newton-Euler Algorithm (RNEA) 递归牛顿-欧拉算法（RNEA）来计算机器人逆动力学
#include "pinocchio/parsers/urdf.hpp"    //加载urdf所需的

using namespace pinocchio;
class WholeBodyController {
 private:
  /* data */
 public:
  Mat43 LegForces;
  int method;
  quadprogpp::Matrix<double> G_quad, CE_quad, CI_quad;
  quadprogpp::Vector<double> g0_quad, ce_quad, ci_quad, x_quad;
  Matrix<double, 18, 1> ddq_d;
  Matrix<double, 6, 1> CoM;
  Matrix<double, 6, 6> M_f;
  matrix_t M_f_b;
  Matrix<double, 12, 18> M_f_j;
  Matrix<double, 12, 6> Jocabian_f;
  Matrix<double, 12, 12> Jocabian_j;
  Matrix<double, 3, 6> Jocabian_arm_t;
  Matrix<double, 6, 1> C_f;
  Matrix<double, 12, 1> C_j;

  Matrix<double, 6, 18> CE;
  Matrix<double, 6, 1> ce;

  Matrix<double, 6, 3> CI_sub;
  Matrix<double, 6, 1> ci_sub;

  Matrix<double, 24, 12> CA;
  Matrix<double, 24, 1> ca;

  Matrix<double, 24, 18> CI;
  Matrix<double, 24, 1> ci;

  Matrix<double, 18, 18> G;
  Matrix<double, 18, 1> g0;

  Matrix<double, 12, 1> f_ext;

  Matrix<double, 30, 1> wbc_input;

  vector_t solution;

  Matrix<double, 6, 1> arm_Gen_Forces;

  double u;  // 摩擦系数
  double f_min;
  double f_max;
  Matrix<double, 6, 18> Jf[5];
  Matrix<double, 6, 18> dJf[5];

  VectorXd q, dq;  // 关节位置 速度 加速度

  pinocchio::Model model;  // 模型

  pinocchio::Data data;  // 机器人模型数据

  double ratio;

  std::vector<std::shared_ptr<HoQp>> ConstructOptProblems(
      std::vector<Task>& tasks);

  std::vector<Task> ConstructTasks(const Vector4i& LegPhase);

  Task ConstructFloatingBaseEomTask(const matrix_t M_fb, const matrix_t J_sfb,
                                    const vector_t c_fb);

  Task ConstructBaseMotionTask();

  Task ConstructJointTorqueTask(const Matrix<double, 12, 18> M_j,
                                const Matrix<double, 12, 12> J_j,
                                const Matrix<double, 12, 1> c_j);

  Task ConstructFrictionConeTask(const Vector4i& _LegPhase);

  Task ConstructNoContactMotionTask(const Vector4i& _LegPhase);

  Task ConstructContactForceTask();

  /**
   * @brief 定义约束
   * @param  _M_f             质量矩阵前6行
   * @param  _Jocabian_f      雅克比矩阵前6行
   * @param  _f_ext           足端外力
   * @param  _C_f             动力学非线性项前6行（离心力、重力和科式力）
   * @param  _ddq_d           目标加速度
   * @param  _CA              不等式约束矩阵
   * @param  _ca              不等式约束上界
   */
  void defineConstraints(const Matrix<double, 6, 6>& _M_f,
                         const Matrix<double, 12, 6>& _Jocabian_f,
                         const Matrix<double, 12, 1>& _f_ext,
                         const Matrix<double, 6, 1>& _C_f,
                         const Matrix<double ,6, 1>& _arm_forces,
                         const Matrix<double, 18, 1>& _ddq_d,
                         const Matrix<double, 24, 12>& _CA,
                         const Matrix<double, 24, 1>& _ca);

  /**
   * @brief 更新动力学输入
   * @param  _x_quad      全身控制优化结果
   */
  void updateDynamicInput(quadprogpp::Vector<double> _x_quad);

  /**
   * @brief QP问题求解（使用Quadprog++库）
   * @param  _G_quad       海森矩阵
   * @param  _g0_quad       梯度矩阵
   * @param  _CE_T       等式约束矩阵
   * @param  _ce             等式约束边界
   * @param  _CI_T           不等式约束矩阵
   * @param  _ci             不等式约束边界
   */
  void Quadprog_Solve(const Matrix<double, 18, 18> _G_quad,
                      const Matrix<double, 18, 1> _g0_quad,
                      const Matrix<double, 18, 6> _CE_T,
                      const Matrix<double, 6, 1> _ce,
                      const Matrix<double, 18, 24> _CI_T,
                      const Matrix<double, 24, 1> _ci);

  /**
   * @brief WBC求解
   * @param  body_joint_acc_d      目标关节加速度
   * @param  GenCoMAcc        广义质心加速度
   * @param  LegForces        足端外力
   * @param  LegPhase         步态序列
   * @param  body_joint_angle      关节角度
   * @param  body_joint_vel        关节速度
   * @param  body_CoMAngle         质心欧拉角
   * @param  body_CoMAngularVel_w  质心角速度
   */
  void WBC_solve(const Mat43& body_joint_acc_d,
                 const Matrix<double, 6, 1>& GenCoMAcc, Mat43& LegForces,
                 const Vector4i& LegPhase, const Mat43& body_joint_angle,
                 const Mat43& body_joint_vel, const Vector3d& _CoMPos,
                 const Vector3d& _CoMVel,const Vector3d& body_CoMAngle,
                 const Vector3d& body_CoMAngularVel_w,const Matrix<double,6,1> arm_joint_angle,
                 const Matrix<double,6,1> arm_joint_vel);

  /**
   * @brief 计算相关矩阵
   * @param  _joint_acc_d     目标关节加速度
   * @param  _GenCoMAcc       广义质心加速度
   * @param  _LegForces       足端外力
   * @param  _LegPhase        步态序列
   * @param  _joint_angle     关节角度
   * @param  _joint_vel       关节速度
   * @param  _CoMAngle        质心欧拉角
   * @param  _CoMAngularVel_w 质心角速度
   */
  void compute_Related_Matrix(
      const Mat43& _joint_acc_d, const Matrix<double, 6, 1>& _GenCoMAcc,
      const Mat43& _LegForces, const Vector4i& _LegPhase,
      const Mat43& _joint_angle, const Mat43& _joint_vel,const Vector3d& _CoMPos,const Vector3d& _CoMVel,
      const Vector3d& _CoMAngle, const Vector3d& _CoMAngularVel_w,const Matrix<double,6,1> arm_joint_angle,
      const Matrix<double,6,1> arm_joint_vel);

  std::string urdf_filename = std::string(
      "/home/wh/simulation/Legged_robot/models/go2/urdf/go2.urdf");
  WholeBodyController(/* args */);
  ~WholeBodyController();
};
