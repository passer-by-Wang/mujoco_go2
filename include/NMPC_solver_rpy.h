/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     NMPC_solver_rpy.h
  @brief    NMPC_solver_rpy
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
class NMPC_solver_rpy {
 private:
  /* data */
 public:
  bool nmpcEnable;
  Mat43 LegForces;
  Eigen::Index max_iter;
  double friction_coe;
  bool isrecompile;
  double min_friction;
  double BQR3_mass;
  Eigen::Vector3d inertia_parameter;
  Matrix<double, 3, 1> W_position;
  Matrix<double, 3, 1> W_orientation;
  Matrix<double, 3, 1> W_velocity;
  Matrix<double, 3, 1> W_omega;
  Matrix<double, 3, 1> W_min_force;
  
  Matrix<double, robot_state_dim, 1> _state;
  Matrix<double, robot_state_dim, 1> _state_d;
  Mat43 _ToePos_wB;
  Vector4i _LegPhase;
  std::thread thread_nmpc;
  bool _isStop;
  bool is_start = false;
  bool init_flag;

  /**
   * @brief 计算足端接触力
   * @param  state  当前状态
   * @param  state_d  目标状态
   * @param  ToePos_wB  足端相对质心位置
   * @param  LegPhase 步态序列
   * @param  isStop 是否退出
   */
  void computeContactForces(const Matrix<double, robot_state_dim, 1>& state,
                            const Matrix<double, robot_state_dim, 1>& state_d,
                            const Mat43& ToePos_wB, const Vector4i& LegPhase,
                            const Matrix<double, 6, 1> _Arm2Body_F,
                            const bool& isStop) {
    _state = state;
    _state_d = state_d;
    _LegPhase = LegPhase;
    _ToePos_wB = ToePos_wB;
    _isStop = isStop;
    if (nmpcEnable && init_flag) {
      init_flag = 0;
      thread_nmpc =
          nmpc_thread(_state, _state_d, _ToePos_wB, _Arm2Body_F,_LegPhase, _isStop);
      thread_nmpc.detach();
      cout << "NMPC_rpy thread is created!" << endl;
    }
  }

  /**
   * @brief   创建nmpc线程
   * @param  _State_Var 当前状态
   * @param  _State_dVar  目标状态
   * @param  _Foot_Pos_Base 足端相对质心位置
   * @param  _Leg_Contact_State 步态序列
   * @param  isStop 是否退出
   * @return thread
   */
  thread nmpc_thread(const Matrix<double, robot_state_dim, 1>& _State_Var,
                     const Matrix<double, robot_state_dim, 1>& _State_dVar,
                     const Mat43& _Foot_Pos_Base,
                     const Matrix<double, 6, 1> _Arm2Body_Forces,
                     const Vector4i& _Leg_Contact_State, const bool& isStop) {
    return thread(&NMPC_solver_rpy::nmpcLoop, this, ref(_State_Var),
                  ref(_State_dVar), ref(_Foot_Pos_Base),
                  ref(_Leg_Contact_State), ref(_Arm2Body_Forces),ref(isStop));
  };

  /** nmpc主函数
   * @brief
   * @param  state  当前状态
   * @param  state_d  目标状态
   * @param  footpos  足端相对质心位置
   * @param  contact_state  步态序列
   */
  void run(const Matrix<double, robot_state_dim, 1>& state,
           const Matrix<double, robot_state_dim, 1>& state_d, const Mat43& footpos,
           const Vector4i& contact_state,const Matrix<double, 6, 1> arm2body_Forces);

  /**
   * @brief nmpc循环
   * @param  _State_Var 当前状态
   * @param  _State_dVar  目标状态
   * @param  _Foot_Pos_Base 足端相对质心位置
   * @param  _Leg_Contact_State   步态序列
   * @param  isStop   是否退出
   */
  void nmpcLoop(const Matrix<double, robot_state_dim, 1>& _State_Var,
                const Matrix<double, robot_state_dim, 1>& _State_dVar,
                const Mat43& _Foot_Pos_Base, const Vector4i& _Leg_Contact_State,
                const Matrix<double, 6, 1> _arm2body_Forces,
                const bool& isStop) {
    while (1) {
      // struct timeval t1, t2;
      // gettimeofday(&t1, NULL);
      run(_State_Var, _State_dVar, _Foot_Pos_Base, _Leg_Contact_State,_arm2body_Forces);

      // gettimeofday(&t2, NULL);

      // cout << "[NMPC solver]: "
      //  << " ";
      // cout << double(t2.tv_usec - t1.tv_usec) / 1000 << " " << endl;
      is_start = true;
      if (isStop)  // 停止MPC时，退出while 1 ，终止足端力的计算
      {
        cout << "[NMPC solver rpy] Stop!" << endl;
        break;
      }
      // cout << "isStop:" << *isStop << endl;
    }
  }

  NMPC_solver_rpy(/* args */);
  ~NMPC_solver_rpy();
};
