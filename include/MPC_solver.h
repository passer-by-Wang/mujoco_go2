/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     MPC_solver.h
  @brief    MPC
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
#include <thread>

#include "Robot_Parameters.h"
#include "common.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/unsupported/Eigen/MatrixFunctions"
#include "qpOASES.hpp"
class MPC_solver {
 private:
  /* data */
 public:
  bool initflag;
  double f_max;
  double f_min;
  double friction_coeff;
  Mat_dim Ac;
  Matrix<double, robot_state_dim, Eigen::Dynamic> Bc;
  Mat_dim Ad;
  Matrix<double, robot_state_dim, Eigen::Dynamic> Bd;
  Matrix<double, mpc_Horizon * robot_state_dim, robot_state_dim> A_qp;
  Matrix<double, mpc_Horizon * robot_state_dim, Eigen::Dynamic> B_qp;
  Mat43 LegForces;
  bool mpcEnable;
  int mpc_discrete_switch;
  int contact_num;
  // 此处使用动态，因为eigen对单个矩阵有大小限制
  Matrix<double, Eigen::Dynamic, Eigen::Dynamic, RowMajor> Q_qp;
  Matrix<double, robot_state_dim, 1> Q_qp_sub;
  Matrix<double, 3, 1> R_qp_sub;
  MatrixXd R_qp;

  MatrixXd H_qp;
  VectorXd g_qp;
  double weight_f;
  Matrix<double, 6, 3> C1;
  Matrix<double, 6, 1> d1;

  Matrix<double, Eigen::Dynamic, Eigen::Dynamic, RowMajor> C_qp;
  Matrix<double, Eigen::Dynamic, 1> d_qp;
  Matrix<double, 6, 1> mpcCoMF;
  /**
   * @brief 计算广义质心力
   * @param  f    足端外力
   * @param  I_w  惯性矩阵
   */
  void comf(Matrix<double, Eigen::Dynamic, 1>& f, Matrix3d& I_w);

  /**
   * @brief 创建mpc线程
   * @param  Para   参数器
   * @param  Rz   yaw角旋转矩阵
   * @param  LegPhase 步态序列
   * @param  ToePos_wB  足端相对质心位置
   * @param  I_w  惯性矩阵
   * @param  x0 当前状态
   * @param  X_d  预测周期目标状态
   * @param  isStop 是否退出
   * @return thread
   */
  thread mpc_thread(Robot_Parameters& Para, const Matrix3d& Rz,
                    const Vector4i& LegPhase, const Mat43& ToePos_wB,
                    const Matrix3d& I_w, const Matrix<double, robot_state_dim, 1>& x0,
                    const Matrix<double, robot_state_dim * mpc_Horizon, 1>& X_d,
                    const bool& isStop) {
    return thread(&MPC_solver::mpcLoop, this, ref(Para), ref(Rz), ref(LegPhase),
                  ref(ToePos_wB), ref(I_w), ref(x0), ref(X_d), ref(isStop));
  };

  /**
   * @brief mpc循环
   * @param  Para 参数器
   * @param  Rz   yaw角旋转矩阵
   * @param  LegPhase   步态序列
   * @param  ToePos_wB  足端相对质心位置
   * @param  I_w  惯性矩阵
   * @param  x0   当前状态
   * @param  X_d  预测周期目标状态
   * @param  isStop   是否退出
   */
  void mpcLoop(Robot_Parameters& Para, const Matrix3d& Rz,
               const Vector4i& LegPhase, const Mat43& ToePos_wB,
               const Matrix3d& I_w, const Matrix<double, robot_state_dim, 1>& x0,
               const Matrix<double, robot_state_dim * mpc_Horizon, 1>& X_d,
               const bool& isStop);

  /**
   * @brief 定义约束
   * @param  u  摩擦因数
   * @param  fmax 足端z方向最大力
   * @param  fmin 足端z方向最小力
   */
  void defineConstraints(double& u, double& fmax, double& fmin);

  /**
   * @brief   计算足端接触力
   * @param  Para   参数器
   * @param  Rz   yaw角旋转矩阵
   * @param  LegPhase   步态序列
   * @param  ToePos_wB  足端相对质心位置
   * @param  I_w  惯性矩阵
   * @param  x0  当前状态
   * @param  X_d    预测周期目标状态
   * @param  isStop   是否退出
   */
  void computeContactForces(
      Robot_Parameters& Para, const Matrix3d& Rz, const Vector4i& LegPhase,
      const Mat43& ToePos_wB, const Matrix3d& I_w,
      const Matrix<double, robot_state_dim, 1>& x0,
      const Matrix<double, robot_state_dim * mpc_Horizon, 1>& X_d,
      const bool& isStop);

  /**
   * @brief 定义单刚体连续系统
   * @param  Para   参数器
   * @param  Rz   yaw角旋转矩阵
   * @param  LegPhase   步态序列
   * @param  ToePos_wB  足端相对质心位置
   * @param  I_w    惯性矩阵
   */
  void defineContinuousSystem(Robot_Parameters& Para, const Matrix3d& Rz,
                              const Vector4i& LegPhase, const Mat43& ToePos_wB,
                              const Matrix3d& I_w);

  /**
   * @brief   连续时域状态转移矩阵离散化
   * @param  _Ac  连续时域状态转移矩阵
   * @param  _Bc  连续时域控制矩阵
   * @param  _dt  时间步长
   */
  void Discretization(const Mat_dim& _Ac,
                      const Matrix<double, robot_state_dim, Eigen::Dynamic>& _Bc,
                      double _dt);
  /**
   * @brief 计算QP形式
   * @param  _Ad  离散时域状态转移矩阵
   * @param  _Bd   离散时域控制矩阵
   */
  void computeQPform(const Mat_dim& _Ad,
                     const Matrix<double, robot_state_dim, Eigen::Dynamic>& _Bd);

  /**
   * @brief QP问题标准化为qpOASES求解形式
   * @param  A_qp 多周期状态转移矩阵
   * @param  B_qp 多周期控制矩阵
   * @param  x0 当前状态
   * @param  X_d  预测周期目标状态
   */
  void standardization(
      const Matrix<double, robot_state_dim * mpc_Horizon, robot_state_dim>& A_qp,
      const Matrix<double, robot_state_dim * mpc_Horizon, Eigen::Dynamic>& B_qp,
      const Matrix<double, robot_state_dim, 1>& x0,
      const Matrix<double, robot_state_dim * mpc_Horizon, 1>& X_d);

  /**
   * @brief   QP问题求解
   * @param  _H_qp  海森矩阵
   * @param  _g_qp  梯度矩阵
   * @param  _C_qp  不等式矩阵
   * @param  _d_qp  不等式上界
   * @param  LegPhase
   */
  void qpOASES_Solve(const MatrixXd& _H_qp, const MatrixXd& _g_qp,
                     const Matrix<double, Eigen::Dynamic, Eigen::Dynamic, RowMajor>& _C_qp,
                     const Matrix<double, Eigen::Dynamic, 1>& _d_qp,
                     const Vector4i& LegPhase);

  MPC_solver(/* args */);
  ~MPC_solver();
};
