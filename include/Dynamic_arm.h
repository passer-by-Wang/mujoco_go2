/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     Dynamic_arm.h
  @brief    Dynamic_arm
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
#include <Eigen/StdVector>

#include "FiniteStateMachine.h"
#include "Robot_Parameters.h"
#include "StateEstimator.h"
#include "TrajectoryGenerator.h"
#include "WholeBodyController.h"
#include "common.h"
#include "eigen3/Eigen/Dense"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp" //计算正运动学
#include "pinocchio/algorithm/rnea.hpp"       //Recursive Newton-Euler Algorithm (RNEA) 递归牛顿-欧拉算法（RNEA）来计算机器人逆动力学
#include "pinocchio/parsers/urdf.hpp"         //加载urdf所需的
using namespace pinocchio;
class Robot_Dynamic_arm
{
private:
  /* data */
public:
  Matrix<double, 6, 6> temp_trans;
  bool isUseDynamic;
  Matrix<double, 6, 1> ArmJointFeedbackTorque;
  Mat43 BodyJointFeedbackTorque;
  Mat43 BodyJointFeedforwardTorque;
  Mat43 BodyJointTorque;
  Matrix<double, 6, 1> ArmJointTorque;
  Matrix3d k_p_joint;
  Matrix3d k_v_joint;
  Mat43 BodyJointTorque_limit_MAX;

  Mat43 BodyJointTorque_limit_MIN;

  VectorXd q, dq, ddq_d; // 关节位置 速度 加速度

  pinocchio::Model model; // 模型

  pinocchio::Data data; // 机器人模型数据

  Matrix<double, 18+6, 1> ExtTorque; // 足端力引起的扭矩那项

  

  std::string urdf_filename = std::string(
      "/home/wh/simulation/Legged_robot/models/go2/urdf/go2.urdf");

  void setTorqueLimit(Mat43 &Torque);

  Matrix<double,6,1> k_p_armjoint;
  Matrix<double,6,1> k_v_armjoint;

  const DiagonalMatrix<double, 3> kp_joint = {100, 100,
                                              100}; // 关节pd反馈控制器增益系数
  const DiagonalMatrix<double, 3> kd_joint = {1, 1, 1};
  void JointPdController(const Mat43 &q, const Mat43 &q_d, const Mat43 &dq,
                         const Mat43 &dq_d);
  Mat43 JointddqPD; // 关节pd得到的值

  /**
   * @brief   计算动力学前馈
   * @param  wbc_ddq       目标加速度
   * @param  wbc_LegForces      目标足端外力
   * @param  body_joint_angle      关节角度
   * @param  body_joint_vel        关节速度
   * @param  body_CoMAngle         质心欧拉角
   * @param  body_CoMAngularVel_w  质心角速度
   */
  void computeJointForwardTorque(StateEstimator &SE,
                                 const Matrix<double, 18, 1> wbc_ddq,
                                 const Matrix<double, 6, 1> _arm_ddq, 
                                 const Mat43 &wbc_LegForces,
                                 const Matrix<double, 6, 1> arm_Gen_F,
                                 const Mat43 &body_joint_angle,
                                 const Mat43 &body_joint_vel,
                                 const Vector3d &body_CoMPos,
                                 const Vector3d &body_CoMVel,
                                 const Vector3d &body_CoMAngle,
                                 const Vector3d &body_CoMAngularVel_w,
                                 const Matrix<double,6,1> arm_joint_angle,
                                 const Matrix<double,6,1> arm_joint_vel);

  void computeArmJointPdForces(const Matrix<double, 6, 1> &arm_joint,
                               const Matrix<double, 6, 1> &arm_joint_vel,
                               const Matrix<double, 6, 1> &arm_joint_d,
                               const Matrix<double, 6, 1> &arm_joint_vel_d);

  /**
   * @brief 计算动力学反馈
   * @param  body_joint_angle   关节角度
   * @param  body_joint_vel     关节速度
   * @param  body_joint_angle_d   目标关节角度
   * @param  body_joint_vel_d      目标关节角速度
   */
  void computeJointPdForces(const Mat43 &body_joint_angle, const Mat43 &body_joint_vel,
                            const Mat43 &body_joint_angle_d,
                            const Mat43 &body_joint_vel_d);
  /**
   * @brief   计算关节扭矩
   * @param  wbc_ddq   目标加速度
   * @param  wbc_LegForces    目标足端外力
   * @param  body_joint_angle      关节角度
   * @param  body_joint_vel       关节角速度
   * @param  body_joint_angle_d    目标关节角度
   * @param  body_joint_vel_d      目标关节角速度
   * @param  body_CoMPos           质心
   * @param  body_CoMVel           质心
   * @param  body_CoMAngle         质心欧拉角
   * @param  body_CoMAngularVel_w  质心角速度
   */
  void computeJointTorque(StateEstimator &SE,
                          const Matrix<double, 18, 1> wbc_ddq, const Matrix<double, 6, 1> arm_ddq,
                          const Mat43 &wbc_LegForces, const Matrix<double, 6, 1> _arm_Gen_F,
                          const Mat43 &body_joint_angle, const Mat43 &body_joint_vel,
                          const Mat43 &body_joint_angle_d,
                          const Mat43 &body_joint_vel_d, const Vector3d &body_CoMPos,
                          const Vector3d &body_CoMVel, const Vector3d &body_CoMAngle,
                          const Vector3d &body_CoMAngularVel_w, const Matrix<double, 6, 1> arm_joint_angle,
                          const Matrix<double, 6, 1> arm_joint_vel, const Matrix<double, 6, 1> &arm_joint_d,
                          const Matrix<double, 6, 1> &arm_joint_vel_d);
  /**
   * @brief 计算动力学
   * @param  wbc_ddq    目标加速度
   * @param  wbc_LegForces   目标足端外力
   * @param  body_joint_angle     关节角度
   * @param  body_joint_vel       关节角速度
   * @param  body_CoMAngle         质心欧拉角
   * @param  body_CoMAngularVel_w  质心角速度
   */
  void computeDynamic(const Matrix<double, 18, 1> wbc_ddq,const Matrix<double, 6, 1> arm_ddq, 
                      const Mat43 &wbc_LegForces,const Matrix<double, 6, 1> arm_Gen_F, const Mat43 &body_joint_angle,
                      const Mat43 &body_joint_vel, const Vector3d &_CoMPos,
                      const Vector3d &_CoMVel, const Vector3d &body_CoMAngle,
                      const Vector3d &body_CoMAngularVel_w,const Matrix<double,6,1> arm_joint_angle,
                                   const Matrix<double,6,1> arm_joint_vel);
  /**
   * @brief 将足端力映射成质心广义力
   * @param  extF
   */
  void mapF2T(const Mat43 &extF,const Matrix<double, 6, 1> arm_Gen_F);
  Robot_Dynamic_arm(/* args */);
  ~Robot_Dynamic_arm();
};