/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     TrajectoryGenerator.h
  @brief    TrajectoryGenerator
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
#include "StateEstimator.h"
#include "common.h"

class TrajectoryGenerator {
 private:
  /* data */
 public:
   bool UpOrDown_Flag;
   Matrix<double, robot_state_dim, 1> vmc_state_d;
   Matrix<double, robot_state_dim * mpc_Horizon, 1> mpc_state_d;
   Matrix<double, robot_state_dim, 1> nmpc_state_d;
   Matrix<double, robot_state_dim + object_state_dim, 1> nmpc_DRBM_state_d;
   Vector3d TargetV;
   Vector3d TargetW;
   Vector3d TargetV_user;
   Vector3d TargetW_user;
   Vector3d object_TargetV;
   Vector3d object_TargetW;
   Vector3d TargetP;
   Vector3d TargetQ;
   Vector3d object_TargetP;
   Vector3d object_TargetQ;
   Vector3d vel2pos;
   Vector3d angularvel2angle;
   Vector3d body_CoMAngle_d;
   Vector3d body_CoMPos_d;
   Vector3d nmpc_body_CoMAngle_d;
   Vector3d nmpc_body_CoMPos_d;
   Vector3d body_CoMAngle_d_tmp;
   Vector3d body_CoMPos_d_tmp;
   Vector3d nmpc_body_CoMAngle_d_tmp;
   Vector3d nmpc_body_CoMPos_d_tmp;
   Vector3d body_CoMAngle_d_mpc;
   Vector3d body_CoMPos_d_mpc;
   Vector3d last_body_CoMAngle_d_mpc;
   Vector3d last_body_CoMPos_d_mpc;
   Vector3d body_CoMAcc_d;
   Vector3d body_CoMAngularAcc_d;
   Vector3d body_CoMAngleVel_d;
   Vector3d body_CoMVel_d;
   Mat43 TargetToePos;
   Mat43 TargetToeVel;
   Mat43 TargetToeAcc;
   Mat43 SwingOffset;
   Mat43 body_joint_angle_d;
   Matrix<double, 6, 1> arm_joint_angle_d;
   Matrix<double, 6, 1> arm_joint_angularvel_d;
   Matrix<double, 6, 1> arm_joint_angularacc_d;
   Mat43 body_joint_vel_d;
   Mat43 body_joint_acc_d;
   Mat43 initToePos;
   Mat43 initToeVel;
   Mat43 ToePos_bH_d;
   Mat43 ToeVel_bH_d;
   Mat43 ToeAcc_bH_d;
   Vector4d body_CoM_Quaternion_d;
   Vector4d object_CoM_Quaternion_d;

   Vector3d ObjectAngle_d;
   Vector3d ObjectPos_d;
   Vector3d ObjectAngularVel_d;
   Vector3d ObjectVel_d;
   Vector3d ObjectAcc_d;
   Vector3d ObjectAngularAcc_d;
   Matrix<double, 6, 1> ObjectDeltaError;
   Matrix<double, 6, 1> temp_Object_joint_d;

   Vector3d initObjectAngle_d;
   Vector3d initObjectPos_d;
   Vector3d initObjectAngularVel_d;
   Vector3d initObjectVel_d;
   Vector3d object_CoMAngle_d_tmp;
   Vector3d object_CoMPos_d_tmp;

   Matrix<double, 6, 1> GenCoMAcc;
   Matrix<double, 6, 1> temp_arm_joint;
   Matrix<double, 6, 1> temp_arm_joint_vel;

   Matrix<double, 6, 1> temp_object_state_error;
   Matrix<double, 6, 1> temp_object_state;
   Matrix<double, 6, 1> last_temp_object_state;

   Mat43 ToePos_wH_d;
   Mat43 ToeVel_wH_d;
   Mat43 ToeAcc_wH_d;
   Matrix<double, 6, 1> Gen_vf_d;
   Matrix3d k_p;
   Matrix3d k_v;
   Matrix3d k_theta;
   Matrix3d k_w;
   bool init_flag;
   bool init_flag_object;
   bool init_flag_object_iter;

   void LinearAcc(const double Acc);

    void LinearVelocityCmd(const double Acc);

   /**
    * @brief 生成单个足端轨迹
    * @param  initToePos     初始足端位置
    * @param  initToeVel       初始足端速度
    * @param  TargetToePos_d   目标足端位置
    * @param  TargetToeVel_d   目标足端速度
    * @param  i  第i条腿
    * @param  LegPhase   步态序列
    * @param  SwingHeight  摆动高度
    * @param  time   当前时间
    * @param  swing_T  摆动时间
    */
   void ToeTraj(StateEstimator &SE, Mat43 &initToePos, Mat43 &initToeVel, Mat43 &TargetToePos_d,
                Mat43 &TargetToeVel_d, int &i, const Vector4i &LegPhase,
                const Vector3d &SwingHeight, double &time, double &swing_T);

   TrajectoryGenerator(/* args */);

   void setObjectTrajectory(const Vector3d &_object_TargetV, const Vector3d &_object_TargetW, StateEstimator &SE, Robot_Parameters &Para);

   /**
    * @brief 轨迹生成
    * @param  SE     状态估计器
    * @param  Para     参数器
    * @param  ischangegait       是否改变步态
    * @param  ischangephase    是否改变相位
    * @param  LegPhase         步态序列
    * @param  SwingHeight      摆腿高度
    * @param  time           当前时间
    * @param  swing_T        摆腿时间
    * @param  count          主循环计数
    */
   void setTrajectory(StateEstimator &SE, Robot_Parameters &Para,
                      bool &ischangegait, bool &ischangephase,
                      const Vector4i &LegPhase, const Vector3d &SwingHeight,
                      double &time, double &swing_T, int &count);

   /**
    * @brief 生成质心轨迹
    * @param  _TargetV     质心目标速度
    * @param  _TargetW       质心目标角速度
    * @param  _TargetP        质心目标位置
    * @param  _TargetQ        质心目标欧拉角
    * @param  Para     参数器
    * @param  SE     状态估计器
    * @param  count    主循环计数
    */
   void setCoMTrajectory(const Vector3d &_TargetV, const Vector3d &_TargetW,
                         const Vector3d &_TargetP, const Vector3d &_TargetQ,
                         Robot_Parameters &Para, StateEstimator &SE, int &count);

   /**
    * @brief 生成足端轨迹
    * @param  SE           状态估计器
    * @param  ischangegait   是否改变步态
    * @param  ischangephase    是否改变相位
    * @param  LegPhase   步态序列
    * @param  SwingHeight  摆腿高度
    * @param  time   当前时间
    * @param  swing_T    摆腿时间
    */
   void setToeTrajectory(StateEstimator &SE, bool &ischangegait,
                         bool &ischangephase, const Vector4i &LegPhase,
                         const Vector3d &SwingHeight, double &time,
                         double &swing_T);

   /**
    * @brief 计算关节目标
    * @param  SE      状态估计器
    * @param  Para       参数器
    * @param  _ToePos_bH_d   目标足端位置
    * @param  _ToeVel_bH_d   目标足端速度
    * @param  _ToeAcc_bH_d   目标足端加速度
    */
   void compute_joint_target(StateEstimator &SE, Robot_Parameters &Para,
                             const Vector4i &LegPhase,
                             Mat43 &_ToePos_bH_d, Mat43 &_ToeVel_bH_d,
                             Mat43 &_ToeAcc_bH_d);

   /**
    * @brief 计算目标质心虚拟力
    * @param  _vmc_state_d   目标状态
    * @param  _vmc_state      当前状态
    */
   void computeTargetVirtualForces(
       const Matrix<double, robot_state_dim, 1> &_vmc_state_d,
       const Matrix<double, robot_state_dim, 1> &_vmc_state);

   /**
    * @brief 计算质心广义角速度
    * @param  VF   质心虚拟力
    * @param  SE     状态估计器
    * @param  Para    参数器
    */
   void computeCoMGenAcc(const Matrix<double, 6, 1> &VF, StateEstimator &SE,
                         Robot_Parameters &Para);

   ~TrajectoryGenerator();
};
