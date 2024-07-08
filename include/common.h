/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     common.h
  @brief    common_use
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

#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <fstream>
#include <iostream>
#include <queue>
#include <thread>

#include "Eigen/Dense"

using namespace std;
using namespace Eigen;
using Mat43 = Matrix<double, 4, 3>;
using matrix_t = Matrix<double, Dynamic, Dynamic>;
using vector_t = Matrix<double, Dynamic, 1>;

using Mat99 = Matrix<double, 9, 9>;

#define INF (1.0/0.0)

const Matrix3d I = Matrix3d::Identity();
const int kNumJoints = 12;
// const int a = 0;

// raibert提出的slip落足点计算公式中的比例系数，用来修正速度跟踪误差
const double kv = 0.005;

const double pi = 3.1415926;
const int mpc_Horizon = 15;
const double dt_MPC = 0.01;
const double dt = 0.001;
const double dt_NMPC=0.001;
const int timestep = 1;    // 仿真中的周期，伺服周期 ms
const int robot_state_dim = 13;  // 状态变量的维度
const int object_state_dim = 12;  // 状态变量的维度
const Vector3d zero = Vector3d::Zero();

#define defaultcolor   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BIUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define bolddblack   "\033[1m\033[30m"      /* Bold Black */
#define boldred     "\033[1m\033[31m"      /* Bold Red */
#define boldgreen   "\033[1m\033[32m"      /* Bold Green */
#define boldyellow  "\033[1m\033[33m"      /* Bold Yellow */
#define boldblue    "\033[1m\033[34m"      /* Bold Blue */
#define boldmagenta "\033[1m\033[35m"      /* Bold Magenta */
#define boldcyan    "\033[1m\033[36m"      /* Bold Cyan */
#define boldwhite   "\033[1m\033[37m"      /* Bold White */

using Mat_dim = Matrix<double, robot_state_dim, robot_state_dim>;

/**
 * @brief 符号函数
 * @param  number 数字
 * @return int  正负1或者0
 */
int sign(const double number);

/**
 * @brief   计算叉乘矩阵
 * @return Matrix3d
 */
Matrix3d cross_product(Vector3d);

void Linearize(double& state, const double& state_d, const double& Acc);

// 定义宏 printinfo 来输出变量名和值
#define printinfocolor(x,color) cout << color << x << "\t\t " << #x << endl << endl;

#define printinfo(x) cout << defaultcolor << x << "\t\t " << #x << endl << endl;

/**
 * @brief 四次多项式插值
 * @param  p0         初始位置
 * @param  v0         初始速度
 * @param  a0         初始加速度
 * @param  p2         目标位置
 * @param  v2         目标速度
 * @param  a2         目标加速度
 * @param  t2         目标时间
 * @param  cont_T     当前时间
 * @param  p          当前时刻位置
 * @param  v          当前时刻速度
 * @param  a          当前时刻加速度
 */
void QuinticPolynomial(double p0, double v0, double a0, double p2, double v2,
                       double a2, double t2, double cont_T, double& p,
                       double& v, double& a);
/**
 * @brief 五次多项式插值
 * @param  p0     初始位置
 * @param  v0     初始速度
 * @param  a0     初始加速度
 * @param  t0     初始时间
 * @param  p1     中间位置
 * @param  t1     中间时间
 * @param  p2     目标位置
 * @param  v2     目标速度
 * @param  a2     目标加速度
 * @param  t2     目标时间
 * @param  step   时间步长
 * @param  cont_T 当前时间
 * @param  p      当前位置
 * @param  v      当前速度
 * @param  a      当前加速度
 */
void TSpline_S_V_A(double p0, double v0, double a0, double t0, double p1,
                   double t1, double p2, double v2, double a2, double t2,
                   double step, double cont_T, double& p, double& v, double& a);
/**
 * @brief  欧拉角转换为四元数
 * @param  CoMAngle 欧拉角
 * @return Vector4d
 */
Vector4d EulerToQuaternion(const Vector3d CoMAngle);

Matrix3d EulerToRotation(const Vector3d body_CoMAngle);

Vector3d QuaterniondToEuler(const Quaterniond q);

/**
 * @brief Get the Time object
 * @return string
 */
string getTime();  // 得到电脑系统的时间，年月日时分秒
