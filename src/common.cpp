/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     common.cpp
  @brief    common
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
#include "../include/common.h"

int sign(const double number)
{
  if (number > 0)
  {
    return 1;
  }
  else if (number < 0)
  {
    return -1;
  }
  else
  {
    return 0; // 如果是零，返回0
  }
}

Matrix3d cross_product(Vector3d p)
{
  Matrix3d temp;
  temp << 0.0, -p(2), p(1), p(2), 0.0, -p(0), -p(1), p(0), 0.0;
  return temp;
}

string getTime() // 得到电脑系统的时间，年月日时分秒
{
  time_t timep;
  time(&timep);
  char tmp[64];
  strftime(tmp, sizeof(tmp), "%Y-%m-%d %H_%M_%S", localtime(&timep));
  return tmp;
}

void Linearize(double& state,const double& state_d,const double& Acc){
  if (state < state_d)
    state += abs(Acc) * 0.001;
  else if (state > state_d)
    state -= abs(Acc) * 0.001;
}

void QuinticPolynomial(double p0, double v0, double a0, double p2, double v2,
                       double a2, double t2, double cont_T, double &p,
                       double &v, double &a)
{
  double A[6];
  A[0] = p0;
  A[1] = v0;
  A[2] = a0 / 2;
  A[3] =
      (20 * p2 - 20 * p0 - (8 * v2 + 12 * v0) * t2 - (3 * a0 - a2) * t2 * t2) /
      (2 * pow(t2, 3));
  A[4] = (30 * p0 - 30 * p2 + (14 * v2 + 16 * v0) * t2 +
          (3 * a0 - 2 * a2) * t2 * t2) /
         (2 * pow(t2, 4));
  A[5] = (12 * p2 - 12 * p0 - (6 * v2 + 6 * v0) * t2 - (a0 - a2) * t2 * t2) /
         (2 * pow(t2, 5));

  p = A[0] + A[1] * cont_T + A[2] * pow(cont_T, 2) + A[3] * pow(cont_T, 3) +
      A[4] * pow(cont_T, 4) +
      A[5] *
          pow(cont_T, 5); // p(t)=a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4+ a5*t^5
  v = A[1] + 2 * A[2] * cont_T + 3 * A[3] * pow(cont_T, 2) +
      4 * A[4] * pow(cont_T, 3) + 5 * A[5] * pow(cont_T, 4); // v(t)
  a = 2 * A[2] + 6 * A[3] * cont_T + 12 * A[4] * pow(cont_T, 2) +
      20 * A[5] * pow(cont_T, 3); // a(t)
}

Vector4d EulerToQuaternion(const Vector3d CoMAngle)
{
  Vector4d CoMQuaternion;
  double cy = cos(CoMAngle(2) * 0.5);
  double sy = sin(CoMAngle(2) * 0.5);

  double cp = cos(CoMAngle(1) * 0.5);
  double sp = sin(CoMAngle(1) * 0.5);

  double cr = cos(CoMAngle(0) * 0.5);
  double sr = sin(CoMAngle(0) * 0.5);

  // x y z w
  CoMQuaternion(0) = cy * cp * sr - sy * sp * cr; // x
  CoMQuaternion(1) = sy * cp * sr + cy * sp * cr; // y
  CoMQuaternion(2) = sy * cp * cr - cy * sp * sr; // z
  CoMQuaternion(3) = cy * cp * cr + sy * sp * sr; // w

  return CoMQuaternion;
}

Matrix3d EulerToRotation(const Vector3d body_CoMAngle)
{
  Matrix3d Rotation;
  double cy = cos(body_CoMAngle(2));
  double sy = sin(body_CoMAngle(2));

  double cp = cos(body_CoMAngle(1));
  double sp = sin(body_CoMAngle(1));

  double cr = cos(body_CoMAngle(0));
  double sr = sin(body_CoMAngle(0));

  Rotation(0, 0) = cp * cy;
  Rotation(0, 1) = cy * sp * sr - sy * cr;
  Rotation(0, 2) = cy * sp * cr + sy * sr;
  Rotation(1, 0) = cp * sy;
  Rotation(1, 1) = sy * sp * sr + cy * cr;
  Rotation(1, 2) = sy * sp * cr - cy * sr;
  Rotation(2, 0) = -sp;
  Rotation(2, 1) = cp * sr;
  Rotation(2, 2) = cp * cr;

  return Rotation;
};

Vector3d QuaterniondToEuler(const Quaterniond q)
{
  Vector3d yaw_pitch_roll;
  // roll (x-axis rotation)
  double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  yaw_pitch_roll(0) = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1.0)
    yaw_pitch_roll(1) = std::copysign(M_PI / 2.0, sinp); // use 90 degrees if out of range
  else
    yaw_pitch_roll(1) = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw_pitch_roll(2) = std::atan2(siny_cosp, cosy_cosp);

  return yaw_pitch_roll;
}

// 狮山中的狮山
void TSpline_S_V_A(double p0, double v0, double a0, double t0, double p1,
                   double t1, double p2, double v2, double a2, double t2,
                   double step, double cont_T, double &p, double &v,
                   double &a)
{
  double P[10];

  // 多项式的系数，四次多项式，五个系数
  P[0] =
      (a2 * (pow(t0, 3.0) * pow(t1, 2.0) - t2 * pow(t0, 3.0) * t1)) /
          (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0))) -
      (v0 * (2 * pow(t0, 3.0) * pow(t1, 2.0) + t2 * pow(t0, 3.0) * t1 -
             pow(t0, 2.0) * pow(t1, 3.0) -
             3 * t2 * pow(t0, 2.0) * pow(t1, 2.0) + t2 * t0 * pow(t1, 3.0))) /
          (pow(t0, 4.0) - 3 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) +
           3 * pow(t0, 2.0) * pow(t1, 2.0) + 3 * t2 * pow(t0, 2.0) * t1 -
           t0 * pow(t1, 3.0) - 3 * t2 * t0 * pow(t1, 2.0) + t2 * pow(t1, 3.0)) -
      (p0 *
       (4 * pow(t0, 3.0) * pow(t1, 2.0) + 2 * t2 * pow(t0, 3.0) * t1 -
        4 * pow(t0, 2.0) * pow(t1, 3.0) - 6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) +
        t0 * pow(t1, 4.0) + 4 * t2 * t0 * pow(t1, 3.0) - t2 * pow(t1, 4.0))) /
          (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) -
           6 * pow(t0, 3.0) * pow(t1, 2.0) - 4 * t2 * pow(t0, 3.0) * t1 +
           4 * pow(t0, 2.0) * pow(t1, 3.0) +
           6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) - t0 * pow(t1, 4.0) -
           4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) +
      (a0 * (2 * pow(t0, 3.0) * pow(t1, 2.0) + t2 * pow(t0, 3.0) * t1 -
             3 * t2 * pow(t0, 2.0) * pow(t1, 2.0))) /
          (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0))) -
      (p1 * (pow(t0, 5.0) - 4 * pow(t0, 4.0) * t1 - t2 * pow(t0, 4.0) +
             2 * pow(t0, 3.0) * pow(t1, 2.0) + 2 * t2 * pow(t0, 3.0) * t1)) /
          (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) -
           6 * pow(t0, 3.0) * pow(t1, 2.0) - 4 * t2 * pow(t0, 3.0) * t1 +
           4 * pow(t0, 2.0) * pow(t1, 3.0) +
           6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) - t0 * pow(t1, 4.0) -
           4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) +
      (pow(t0, 3.0) * t1 * v2) /
          (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
           t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0)) -
      (2 * p1 * pow(t0, 3.0) * t1) /
          (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 -
           2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
           pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) +
      (2 * p2 * pow(t0, 3.0) * t1) /
          (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 -
           2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
           pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0));
  P[1] =
      (2 * p1 * (pow(t0, 3.0) + 3 * t1 * pow(t0, 2.0))) /
          (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 -
           2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
           pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) -
      (2 * p2 * (pow(t0, 3.0) + 3 * t1 * pow(t0, 2.0))) /
          (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 -
           2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
           pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) -
      (v0 * (-5 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) +
             3 * t2 * pow(t0, 2.0) * t1 + t0 * pow(t1, 3.0) +
             3 * t2 * t0 * pow(t1, 2.0) - t2 * pow(t1, 3.0))) /
          (pow(t0, 4.0) - 3 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) +
           3 * pow(t0, 2.0) * pow(t1, 2.0) + 3 * t2 * pow(t0, 2.0) * t1 -
           t0 * pow(t1, 3.0) - 3 * t2 * t0 * pow(t1, 2.0) + t2 * pow(t1, 3.0)) -
      (a2 * (pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) +
             3 * pow(t0, 2.0) * pow(t1, 2.0) - 3 * t2 * pow(t0, 2.0) * t1)) /
          (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0))) -
      (a0 * (5 * pow(t0, 3.0) * t1 + t2 * pow(t0, 3.0) +
             3 * pow(t0, 2.0) * pow(t1, 2.0) - 3 * t2 * pow(t0, 2.0) * t1 -
             6 * t2 * t0 * pow(t1, 2.0))) /
          (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0))) -
      (v2 * (pow(t0, 3.0) + 3 * t1 * pow(t0, 2.0))) /
          (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
           t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0)) -
      (2 * p0 *
       (-5 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) +
        3 * pow(t0, 2.0) * pow(t1, 2.0) + 3 * t2 * pow(t0, 2.0) * t1)) /
          (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) -
           6 * pow(t0, 3.0) * pow(t1, 2.0) - 4 * t2 * pow(t0, 3.0) * t1 +
           4 * pow(t0, 2.0) * pow(t1, 3.0) +
           6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) - t0 * pow(t1, 4.0) -
           4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) +
      (2 * p1 *
       (-5 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) +
        3 * pow(t0, 2.0) * pow(t1, 2.0) + 3 * t2 * pow(t0, 2.0) * t1)) /
          (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) -
           6 * pow(t0, 3.0) * pow(t1, 2.0) - 4 * t2 * pow(t0, 3.0) * t1 +
           4 * pow(t0, 2.0) * pow(t1, 3.0) +
           6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) - t0 * pow(t1, 4.0) -
           4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0));
  P[2] =
      (6 * p2 * (pow(t0, 2.0) + t1 * t0)) /
          (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 -
           2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
           pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) -
      (6 * p1 * (pow(t0, 2.0) + t1 * t0)) /
          (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 -
           2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
           pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) +
      (3 * v2 * (pow(t0, 2.0) + t1 * t0)) /
          (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
           t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0)) +
      (6 * p0 *
       (-pow(t0, 3.0) - pow(t0, 2.0) * t1 + t0 * pow(t1, 2.0) + t2 * t0 * t1)) /
          (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) -
           6 * pow(t0, 3.0) * pow(t1, 2.0) - 4 * t2 * pow(t0, 3.0) * t1 +
           4 * pow(t0, 2.0) * pow(t1, 3.0) +
           6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) - t0 * pow(t1, 4.0) -
           4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) -
      (6 * p1 *
       (-pow(t0, 3.0) - pow(t0, 2.0) * t1 + t0 * pow(t1, 2.0) + t2 * t0 * t1)) /
          (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) -
           6 * pow(t0, 3.0) * pow(t1, 2.0) - 4 * t2 * pow(t0, 3.0) * t1 +
           4 * pow(t0, 2.0) * pow(t1, 3.0) +
           6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) - t0 * pow(t1, 4.0) -
           4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) +
      (a2 * (pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) + t0 * pow(t1, 2.0) -
             t2 * t0 * t1)) /
          (2 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0))) +
      (3 * v0 *
       (-pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 + t0 * pow(t1, 2.0) +
        2 * t2 * t0 * t1)) /
          (pow(t0, 4.0) - 3 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) +
           3 * pow(t0, 2.0) * pow(t1, 2.0) + 3 * t2 * pow(t0, 2.0) * t1 -
           t0 * pow(t1, 3.0) - 3 * t2 * t0 * pow(t1, 2.0) + t2 * pow(t1, 3.0)) +
      (a0 * (pow(t0, 3.0) + 3 * pow(t0, 2.0) * t1 - 3 * t2 * t0 * t1 -
             t2 * pow(t1, 2.0))) /
          (2 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0)));
  P[3] =
      (2 * p1 * (3 * t0 + t1)) /
          (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 -
           2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
           pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) -
      (v0 * (-5 * pow(t0, 2.0) + 2 * t2 * t0 + pow(t1, 2.0) + 2 * t2 * t1)) /
          (pow(t0, 4.0) - 3 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) +
           3 * pow(t0, 2.0) * pow(t1, 2.0) + 3 * t2 * pow(t0, 2.0) * t1 -
           t0 * pow(t1, 3.0) - 3 * t2 * t0 * pow(t1, 2.0) + t2 * pow(t1, 3.0)) -
      (2 * p2 * (3 * t0 + t1)) /
          (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 -
           2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
           pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) -
      (2 * p0 *
       (-4 * pow(t0, 2.0) + t0 * t1 + t2 * t0 + pow(t1, 2.0) + t2 * t1)) /
          (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) -
           6 * pow(t0, 3.0) * pow(t1, 2.0) - 4 * t2 * pow(t0, 3.0) * t1 +
           4 * pow(t0, 2.0) * pow(t1, 3.0) +
           6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) - t0 * pow(t1, 4.0) -
           4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) +
      (2 * p1 *
       (-4 * pow(t0, 2.0) + t0 * t1 + t2 * t0 + pow(t1, 2.0) + t2 * t1)) /
          (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) -
           6 * pow(t0, 3.0) * pow(t1, 2.0) - 4 * t2 * pow(t0, 3.0) * t1 +
           4 * pow(t0, 2.0) * pow(t1, 3.0) +
           6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) - t0 * pow(t1, 4.0) -
           4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) -
      (v2 * (3 * t0 + t1)) /
          (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
           t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0)) -
      (a2 * (3 * t0 * t1 - 3 * t0 * t2 - t1 * t2 + pow(t1, 2.0))) /
          (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0))) +
      (a0 * (-6 * pow(t0, 2.0) - 3 * t0 * t1 + 3 * t2 * t0 + pow(t1, 2.0) +
             5 * t2 * t1)) /
          (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0)));
  P[4] =
      (2 * p2) / (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 -
                  2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
                  pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) +
                  t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
                  pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) -
      (2 * p1) / (pow(t0, 3.0) * t1 - pow(t0, 3.0) * t2 -
                  2 * pow(t0, 2.0) * pow(t1, 2.0) + pow(t0, 2.0) * t1 * t2 +
                  pow(t0, 2.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) +
                  t0 * pow(t1, 2.0) * t2 - 2 * t0 * t1 * pow(t2, 2.0) -
                  pow(t1, 3.0) * t2 + pow(t1, 2.0) * pow(t2, 2.0)) +
      v2 / (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
            t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0)) -
      (a0 * (t1 - 3 * t0 + 2 * t2)) /
          (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0))) +
      (a2 * (t1 - t2)) /
          (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0) * t1 - t2 * pow(t0, 2.0) +
                t0 * pow(t1, 2.0) + 2 * t2 * t0 * t1 - t2 * pow(t1, 2.0))) +
      (v0 * (t1 - 2 * t0 + t2)) /
          (pow(t0, 4.0) - 3 * pow(t0, 3.0) * t1 - t2 * pow(t0, 3.0) +
           3 * pow(t0, 2.0) * pow(t1, 2.0) + 3 * t2 * pow(t0, 2.0) * t1 -
           t0 * pow(t1, 3.0) - 3 * t2 * t0 * pow(t1, 2.0) + t2 * pow(t1, 3.0)) +
      (p0 * (2 * t1 - 3 * t0 + t2)) /
          (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) -
           6 * pow(t0, 3.0) * pow(t1, 2.0) - 4 * t2 * pow(t0, 3.0) * t1 +
           4 * pow(t0, 2.0) * pow(t1, 3.0) +
           6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) - t0 * pow(t1, 4.0) -
           4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0)) -
      (p1 * (2 * t1 - 3 * t0 + t2)) /
          (-pow(t0, 5.0) + 4 * pow(t0, 4.0) * t1 + t2 * pow(t0, 4.0) -
           6 * pow(t0, 3.0) * pow(t1, 2.0) - 4 * t2 * pow(t0, 3.0) * t1 +
           4 * pow(t0, 2.0) * pow(t1, 3.0) +
           6 * t2 * pow(t0, 2.0) * pow(t1, 2.0) - t0 * pow(t1, 4.0) -
           4 * t2 * t0 * pow(t1, 3.0) + t2 * pow(t1, 4.0));
  P[5] =
      (p2 *
       (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) -
        4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
        4 * pow(t1, 2.0) * pow(t2, 3.0) - 6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) +
        2 * t0 * t1 * pow(t2, 3.0))) /
          (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) -
           4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
           6 * pow(t1, 2.0) * pow(t2, 3.0) -
           6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
           4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) -
      (a2 * (2 * pow(t1, 2.0) * pow(t2, 3.0) -
             3 * t0 * pow(t1, 2.0) * pow(t2, 2.0) + t0 * t1 * pow(t2, 3.0))) /
          (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
                2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0))) -
      (v2 * (-pow(t1, 3.0) * pow(t2, 2.0) + t0 * pow(t1, 3.0) * t2 +
             2 * pow(t1, 2.0) * pow(t2, 3.0) -
             3 * t0 * pow(t1, 2.0) * pow(t2, 2.0) + t0 * t1 * pow(t2, 3.0))) /
          (-pow(t1, 3.0) * t2 + t0 * pow(t1, 3.0) +
           3 * pow(t1, 2.0) * pow(t2, 2.0) - 3 * t0 * pow(t1, 2.0) * t2 -
           3 * t1 * pow(t2, 3.0) + 3 * t0 * t1 * pow(t2, 2.0) + pow(t2, 4.0) -
           t0 * pow(t2, 3.0)) +
      (p1 * (2 * pow(t1, 2.0) * pow(t2, 3.0) - 4 * t1 * pow(t2, 4.0) +
             2 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0))) /
          (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) -
           4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
           6 * pow(t1, 2.0) * pow(t2, 3.0) -
           6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
           4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) -
      (a0 * (pow(t1, 2.0) * pow(t2, 3.0) - t0 * t1 * pow(t2, 3.0))) /
          (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
                2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0))) -
      (t1 * pow(t2, 3.0) * v0) /
          (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
           2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0)) +
      (2 * p0 * t1 * pow(t2, 3.0)) /
          (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) - t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
           pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) +
           t1 * pow(t2, 3.0)) -
      (2 * p1 * t1 * pow(t2, 3.0)) /
          (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) - t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
           pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) +
           t1 * pow(t2, 3.0));
  P[6] =
      (2 * p1 * (pow(t2, 3.0) + 3 * t1 * pow(t2, 2.0))) /
          (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) - t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
           pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) +
           t1 * pow(t2, 3.0)) -
      (2 * p0 * (pow(t2, 3.0) + 3 * t1 * pow(t2, 2.0))) /
          (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) - t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
           pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) +
           t1 * pow(t2, 3.0)) +
      (v2 * (-pow(t1, 3.0) * t2 + t0 * pow(t1, 3.0) -
             3 * t0 * pow(t1, 2.0) * t2 + 5 * t1 * pow(t2, 3.0) -
             3 * t0 * t1 * pow(t2, 2.0) + t0 * pow(t2, 3.0))) /
          (-pow(t1, 3.0) * t2 + t0 * pow(t1, 3.0) +
           3 * pow(t1, 2.0) * pow(t2, 2.0) - 3 * t0 * pow(t1, 2.0) * t2 -
           3 * t1 * pow(t2, 3.0) + 3 * t0 * t1 * pow(t2, 2.0) + pow(t2, 4.0) -
           t0 * pow(t2, 3.0)) -
      (2 * p1 *
       (3 * pow(t1, 2.0) * pow(t2, 2.0) - 5 * t1 * pow(t2, 3.0) +
        3 * t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0))) /
          (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) -
           4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
           6 * pow(t1, 2.0) * pow(t2, 3.0) -
           6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
           4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) +
      (2 * p2 *
       (3 * pow(t1, 2.0) * pow(t2, 2.0) - 5 * t1 * pow(t2, 3.0) +
        3 * t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0))) /
          (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) -
           4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
           6 * pow(t1, 2.0) * pow(t2, 3.0) -
           6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
           4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) +
      (a0 * (3 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0) -
             3 * t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0))) /
          (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
                2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0))) +
      (a2 * (3 * pow(t1, 2.0) * pow(t2, 2.0) - 6 * t0 * pow(t1, 2.0) * t2 +
             5 * t1 * pow(t2, 3.0) - 3 * t0 * t1 * pow(t2, 2.0) +
             t0 * pow(t2, 3.0))) /
          (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
                2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0))) +
      (v0 * (pow(t2, 3.0) + 3 * t1 * pow(t2, 2.0))) /
          (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
           2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0));
  P[7] =
      (6 * p0 * (pow(t2, 2.0) + t1 * t2)) /
          (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) - t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
           pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) +
           t1 * pow(t2, 3.0)) -
      (6 * p1 * (pow(t2, 2.0) + t1 * t2)) /
          (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) - t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
           pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) +
           t1 * pow(t2, 3.0)) +
      (a0 * (-pow(t1, 2.0) * t2 - t1 * pow(t2, 2.0) + t0 * t1 * t2 +
             t0 * pow(t2, 2.0))) /
          (2 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
                2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0))) -
      (3 * v2 *
       (-pow(t1, 2.0) * t2 + 2 * t1 * pow(t2, 2.0) - 2 * t0 * t1 * t2 +
        pow(t2, 3.0))) /
          (-pow(t1, 3.0) * t2 + t0 * pow(t1, 3.0) +
           3 * pow(t1, 2.0) * pow(t2, 2.0) - 3 * t0 * pow(t1, 2.0) * t2 -
           3 * t1 * pow(t2, 3.0) + 3 * t0 * t1 * pow(t2, 2.0) + pow(t2, 4.0) -
           t0 * pow(t2, 3.0)) -
      (3 * v0 * (pow(t2, 2.0) + t1 * t2)) /
          (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
           2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0)) +
      (a2 * (t0 * pow(t1, 2.0) - 3 * t1 * pow(t2, 2.0) + 3 * t0 * t1 * t2 -
             pow(t2, 3.0))) /
          (2 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
                2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0))) -
      (6 * p1 *
       (-pow(t1, 2.0) * t2 + t1 * pow(t2, 2.0) - t0 * t1 * t2 + pow(t2, 3.0))) /
          (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) -
           4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
           6 * pow(t1, 2.0) * pow(t2, 3.0) -
           6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
           4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) +
      (6 * p2 *
       (-pow(t1, 2.0) * t2 + t1 * pow(t2, 2.0) - t0 * t1 * t2 + pow(t2, 3.0))) /
          (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) -
           4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
           6 * pow(t1, 2.0) * pow(t2, 3.0) -
           6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
           4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0));
  P[8] =
      (2 * p1 * (t1 + 3 * t2)) /
          (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) - t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
           pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) +
           t1 * pow(t2, 3.0)) -
      (2 * p0 * (t1 + 3 * t2)) /
          (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 +
           pow(t0, 2.0) * pow(t2, 2.0) - t0 * pow(t1, 3.0) +
           t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) - t0 * pow(t2, 3.0) +
           pow(t1, 3.0) * t2 - 2 * pow(t1, 2.0) * pow(t2, 2.0) +
           t1 * pow(t2, 3.0)) -
      (v2 * (pow(t1, 2.0) + 2 * t0 * t1 - 5 * pow(t2, 2.0) + 2 * t0 * t2)) /
          (-pow(t1, 3.0) * t2 + t0 * pow(t1, 3.0) +
           3 * pow(t1, 2.0) * pow(t2, 2.0) - 3 * t0 * pow(t1, 2.0) * t2 -
           3 * t1 * pow(t2, 3.0) + 3 * t0 * t1 * pow(t2, 2.0) + pow(t2, 4.0) -
           t0 * pow(t2, 3.0)) -
      (a2 * (pow(t1, 2.0) - 3 * t1 * t2 + 5 * t0 * t1 - 6 * pow(t2, 2.0) +
             3 * t0 * t2)) /
          (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
                2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0))) -
      (a0 * (t0 * t1 + 3 * t0 * t2 - 3 * t1 * t2 - pow(t1, 2.0))) /
          (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
                2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0))) +
      (v0 * (t1 + 3 * t2)) /
          (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
           2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0)) -
      (2 * p1 *
       (pow(t1, 2.0) + t1 * t2 + t0 * t1 - 4 * pow(t2, 2.0) + t0 * t2)) /
          (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) -
           4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
           6 * pow(t1, 2.0) * pow(t2, 3.0) -
           6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
           4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) +
      (2 * p2 *
       (pow(t1, 2.0) + t1 * t2 + t0 * t1 - 4 * pow(t2, 2.0) + t0 * t2)) /
          (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) -
           4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
           6 * pow(t1, 2.0) * pow(t2, 3.0) -
           6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
           4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0));
  P[9] =
      (2 * p0) / (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 +
                  pow(t0, 2.0) * pow(t2, 2.0) - t0 * pow(t1, 3.0) +
                  t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) -
                  t0 * pow(t2, 3.0) + pow(t1, 3.0) * t2 -
                  2 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0)) -
      v0 / (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
            2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0)) -
      (2 * p1) / (pow(t0, 2.0) * pow(t1, 2.0) - 2 * pow(t0, 2.0) * t1 * t2 +
                  pow(t0, 2.0) * pow(t2, 2.0) - t0 * pow(t1, 3.0) +
                  t0 * pow(t1, 2.0) * t2 + t0 * t1 * pow(t2, 2.0) -
                  t0 * pow(t2, 3.0) + pow(t1, 3.0) * t2 -
                  2 * pow(t1, 2.0) * pow(t2, 2.0) + t1 * pow(t2, 3.0)) +
      (p1 * (t0 + 2 * t1 - 3 * t2)) /
          (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) -
           4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
           6 * pow(t1, 2.0) * pow(t2, 3.0) -
           6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
           4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) -
      (p2 * (t0 + 2 * t1 - 3 * t2)) /
          (pow(t1, 4.0) * t2 - t0 * pow(t1, 4.0) -
           4 * pow(t1, 3.0) * pow(t2, 2.0) + 4 * t0 * pow(t1, 3.0) * t2 +
           6 * pow(t1, 2.0) * pow(t2, 3.0) -
           6 * t0 * pow(t1, 2.0) * pow(t2, 2.0) - 4 * t1 * pow(t2, 4.0) +
           4 * t0 * t1 * pow(t2, 3.0) + pow(t2, 5.0) - t0 * pow(t2, 4.0)) +
      (v2 * (t0 + t1 - 2 * t2)) /
          (-pow(t1, 3.0) * t2 + t0 * pow(t1, 3.0) +
           3 * pow(t1, 2.0) * pow(t2, 2.0) - 3 * t0 * pow(t1, 2.0) * t2 -
           3 * t1 * pow(t2, 3.0) + 3 * t0 * t1 * pow(t2, 2.0) + pow(t2, 4.0) -
           t0 * pow(t2, 3.0)) +
      (a2 * (2 * t0 + t1 - 3 * t2)) /
          (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
                2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0))) +
      (a0 * (t0 - t1)) /
          (6 * (-pow(t1, 2.0) * t2 + t0 * pow(t1, 2.0) + 2 * t1 * pow(t2, 2.0) -
                2 * t0 * t1 * t2 - pow(t2, 3.0) + t0 * pow(t2, 2.0)));

  if (cont_T <= t1) // 前半摆动时间
  {
    p = P[0] + P[1] * cont_T + P[2] * cont_T * cont_T +
        P[3] * cont_T * cont_T * cont_T +
        P[4] * cont_T * cont_T * cont_T *
            cont_T; // p(t)=a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4
    v = P[1] + 2 * P[2] * cont_T + 3 * P[3] * cont_T * cont_T +
        4 * P[4] * cont_T * cont_T *
            cont_T; // v(t)=a1 + 2*a2*t + 3*a3*t^2+ 4*a4*t^3
    a = 2 * P[2] + 6 * P[3] * cont_T +
        12 * P[4] * cont_T * cont_T; // a(t)=2*a2 + 6*a3*t + 12*a4*t^2
  }

  if ((cont_T > t1) && (cont_T <= t2)) // 后半摆动时间
  {
    p = P[5] + P[6] * cont_T + P[7] * cont_T * cont_T +
        P[8] * cont_T * cont_T * cont_T +
        P[9] * cont_T * cont_T * cont_T * cont_T;
    v = P[6] + 2 * P[7] * cont_T + 3 * P[8] * cont_T * cont_T +
        4 * P[9] * cont_T * cont_T * cont_T;
    a = 2 * P[7] + 6 * P[8] * cont_T + 12 * P[9] * cont_T * cont_T;
  }

  if (cont_T > t2) // 超出规划的摆动时间时,传期望落足点数据
  {
    p = p2;
    v = v2;
    a = a2;
  }
}