/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     StateEsimator.h
  @brief    StateEsimator
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
#include "Robot_Parameters.h"

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp" //计算正运动学
#include "pinocchio/algorithm/rnea.hpp"       //Recursive Newton-Euler Algorithm (RNEA) 递归牛顿-欧拉算法（RNEA）来计算机器人逆动力学
#include "pinocchio/parsers/urdf.hpp"         //加载urdf所需的
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "common.h"
using namespace pinocchio;
class StateEstimator
{
private:
  /* data */
public:
  StateEstimator(/* args */);
  ~StateEstimator();

  void CollisionCheck();

  int LegForce_estimate_method;

  bool init_Read_Joint;
  Mat43 init_body_joint_angle;


  /**
   * @brief 状态估计
   * @param  LegPhase 步态序列
   * @param  Para 参数器
   */
  void state_calc(Vector4i &LegPhase, Robot_Parameters &Para);

  /**
   * @brief 计算旋转矩阵（无yaw角）
   * @param  _CoMAngle    质心欧拉角
   * @return Matrix3d
   */
  Matrix3d calc_RotationMatrix(Vector3d &_CoMAngle);

  Matrix3d calc_RotationMatrix1(Vector3d &_CoMAngle);

  /**
   * @brief 计算足端位置
   * @param  q  关节角度
   * @param  Para 参数器
   */
  void calc_Toe_Pos(const Mat43 &q, Robot_Parameters &Para);

  /**
   * @brief 计算雅克比矩阵和雅克比矩阵的微分
   * @param  q  关节角度
   * @param  dq 关节角速度
   * @param  Para   参数器
   */
  void calc_Jocabian_dJacobian(const Mat43 &q, const Mat43 &dq,
                               Robot_Parameters &Para);

  /**
   * @brief 计算足端速度
   * @param  dq 关节角速度
   */
  void calc_Toe_Vel(const Mat43 &dq);

  /**
   * @brief 映射至世界坐标系
   * @param  Para 参数器
   */
  void convert2worldframe(Robot_Parameters &Para);

  Matrix<double,6,1> updateObject(const Matrix<double,6,1> &arm_joint);

  /**
   * @brief 足端从髋关节坐标系到世界坐标系的旋转矩阵
   * @param  q  关节角度
   */
  void bH2Toe_RotationMatrix(const Mat43 &q);

  /**
   * @brief 其他估计
   */
  void otherEstimation();

  /**
   * @brief   卡尔曼滤波
   * @param  LegPhase   步态序列
   * @param  _CoMAcc  质心加速度
   * @param  _CoMAngularVel_w 质心角速度
   * @param  _CoMAngle  质心欧拉角
   * @param  _CoMVel  质心速度
   * @param  _ToeVel_wH 足端速度
   */
  void KalmanFilter(Vector4i &LegPhase, Vector3d &_CoMAcc,
                    Vector3d &_CoMAngularVel_w, Vector3d &_CoMAngle,
                    Vector3d &_CoMVel, Mat43 &_ToeVel_wH);

  /**
   * @brief 逆运动学
   * @param  p  足端位置
   * @param  Para 参数器
   * @return Mat43
   */
  Mat43 IK(Mat43 &p, Robot_Parameters &Para);

  /**
   * @brief 估计地形坡度
   * @param  _ToePos_wB 足端位置
   */
  void slope_angle_cal(const Mat43 &_ToePos_wB);

  void Estimate_Object_State(const Robot_Parameters &Para);

  void Estimate_Object2Body_F(const Robot_Parameters &Para);
  Vector4d LegForce_z_w_estimate;
  Matrix<double, robot_state_dim, 1> vmc_state;
  Matrix<double, robot_state_dim, 1> mpc_state;
  Matrix<double, robot_state_dim, 1> nmpc_state;
  Matrix<double, robot_state_dim+object_state_dim, 1> nmpc_DRBM_state;
  Matrix3d RotationMatrix;
  Matrix3d RotationMatrix1;
  Matrix3d bH2Toe_R[4];
  Matrix3d Rz;
  Matrix3d Inertia_w;
  int cycle;
  Vector3d body_CoM_offset;
  Matrix<double, 3, 1> CoMVeloSim;
  Vector3d body_CoMAngle;
  Vector4d body_CoM_Quaternion;
  Vector3d Last_body_CoMAngle;
  Vector3d initbodyCoMAngle;
  Vector3d body_CoMPos;
  Vector3d initbodyCoMPos;
  Vector3d body_CoMAngularVel;
  Vector3d body_CoMAngularVel_w;
  Vector3d body_CoMVel;
  Vector3d body_CoMAcc;
  Vector3d body_CoMAcc_w;
  Vector3d body_CoMVel_fom_toe;
  Vector4i LegContactState;
  Vector4i lastLegContactState;
  Mat43 last_body_joint_angle;
  Mat43 body_joint_angle;
  Mat43 body_joint_vel;

  Matrix<double, 6, 1> arm_joint_angle;
  Matrix<double, 6, 1> arm_joint_vel;
  Matrix<double, 6, 1> last_arm_joint_angle;
  Mat43 RealLegContactForces;
  Mat43 LastRealLegContactForces;
  Mat43 RealLegContactForces_w;
  Mat43 RealJointTorque;
  Mat43 BaseMat;
  Matrix<double, 6, 1> Arm2BodyGForces;
  int ForceCount[4] = {0};

  Vector3d object_CoMPos;
  Vector3d object_CoMAngle;
  Vector3d object_CoMVel;
  Vector3d object_CoMAngularVel;
  Vector3d object_CoMAcc;
  Vector3d object_CoMAngularAcc;
  Vector4d object_CoM_Quaternion;
  

  Vector3d object_CoMPos_wrtB;

  Matrix<double, 6, 18> Jf[4];
  Matrix<double, 3, 3> Jocabian_leg[4];

  Vector3d Arm_link_mass[7];
  Vector3d Arm_link_com_pos_wtb[7];
  Vector3d Arm_link_com_wtlastf[7];

  Mat43 ToePos_bH;
  Mat43 ToePos_wH;
  Mat43 ToePos_wB;

  Mat43 ToeVel_bH;
  Mat43 ToeVel_wH;
  Mat43 ToeVel_wB;

  Matrix<double,6,1> C_arm;

  Matrix<double, 6, 18 + 6> J_object;
  Matrix<double, 6, 18 + 6> J_object_temp;
  Matrix<double, 6, 18 + 6> dJ_object;

  double ground_pitch; // 地形角度
  double ground_roll;

  Mat99 A;                // 状态转移矩阵，固定值
  Mat99 P;                // 误差协方差矩阵，需要有初值，一般为单位阵
  Mat99 Q;                // 预测误差协方差矩阵，固定值
  Mat99 P_pre;            // 先验误差协方差矩阵，无需固定值
  Mat99 H;                // 预测值转换为测量值形式，固定值
  Mat99 R;                // 测量误差协方差矩阵，固定值
  Mat99 K;                // 增益矩阵
  Mat99 I;                // 单位阵
  Matrix<double, 9, 6> B; // 控制矩阵
  Matrix<double, 6, 1> U; // 输入

  Matrix<double, 9, 1> X_last_k_1; // 后验上一个状态
  Matrix<double, 9, 1> X_k;        // 后验当前状态
  Matrix<double, 9, 1> X_pre_k;    // 先验状态
  Matrix<double, 9, 1> Z_k;        // 测量值

  Matrix3d Jacobian[4]; // 四条腿的雅克比矩阵

  Matrix3d dJacobian[4];

  std::string urdf_filename = std::string(
      "/home/wh/simulation/Legged_robot/models/go2/urdf/go2.urdf");

  VectorXd q, dq, ddq; // 关节位置 速度 加速度

  pinocchio::Model model; // 模型

  pinocchio::Data data; // 机器人模型数据
};
