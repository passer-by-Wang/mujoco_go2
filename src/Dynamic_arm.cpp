/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     Dynamic_arm.cpp
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
#include "../include/Dynamic_arm.h"
Robot_Dynamic_arm::Robot_Dynamic_arm(/* args */)
{
  isUseDynamic = 0;
  k_p_joint.setZero();
  k_v_joint.setZero();
  BodyJointFeedbackTorque.setZero();
  BodyJointFeedforwardTorque.setZero();
  ArmJointFeedbackTorque.setZero();

  BodyJointTorque_limit_MAX.setConstant(200);
  BodyJointTorque_limit_MIN.setConstant(-200);

  pinocchio::urdf::buildModel(urdf_filename, model); // 加载urdf模型
  data = pinocchio::Data(model);
  q.setZero(model.nq);  // np：Dimension of the configuration vector
                        // representation.配置向量表示的维度
  dq.setZero(model.nv); // nv：Dimension of the velocity vector
                        // space.速度向量空间的维度
  ddq_d.setZero(model.nv);
}

void Robot_Dynamic_arm::computeJointTorque(StateEstimator &SE,
                                           const Matrix<double, 18, 1> wbc_ddq, const Matrix<double, 6, 1> arm_ddq,
                                           const Mat43 &wbc_LegForces,
                                           const Matrix<double, 6, 1> _arm_Gen_F,
                                           const Mat43 &body_joint_angle, const Mat43 &body_joint_vel,
                                           const Mat43 &body_joint_angle_d, const Mat43 &body_joint_vel_d,
                                           const Vector3d &body_CoMPos, const Vector3d &body_CoMVel,
                                           const Vector3d &body_CoMAngle, const Vector3d &body_CoMAngularVel_w,
                                           const Matrix<double, 6, 1> arm_joint_angle,
                                           const Matrix<double, 6, 1> arm_joint_vel,
                                           const Matrix<double, 6, 1> &arm_joint_d,
                                           const Matrix<double, 6, 1> &arm_joint_vel_d)
{
  // computeArmJointPdForces(arm_joint_angle, arm_joint_vel, arm_joint_d, arm_joint_vel_d);
  computeJointPdForces(body_joint_angle, body_joint_vel, body_joint_angle_d, body_joint_vel_d);
  computeJointForwardTorque(SE, wbc_ddq, arm_ddq, wbc_LegForces, _arm_Gen_F, body_joint_angle, body_joint_vel, body_CoMPos, body_CoMVel,
                            body_CoMAngle, body_CoMAngularVel_w, arm_joint_angle, arm_joint_vel);
  BodyJointTorque = BodyJointFeedforwardTorque + BodyJointFeedbackTorque;
  ArmJointTorque = ArmJointTorque + ArmJointFeedbackTorque;
  for (int i = 0; i < 6; i++)
  {
    if (i == 1)
    {
      if (abs(ArmJointTorque(i)) > 60)
      {
        ArmJointTorque(i) = sign(ArmJointTorque(i)) * 60;
      }
    }
    else
    {
      if (abs(ArmJointTorque(i)) > 30)
      {
        ArmJointTorque(i) = sign(ArmJointTorque(i)) * 30;
      }
    }
  }
  // setTorqueLimit(BodyJointTorque);
  // printinfo(BodyJointFeedforwardTorque);
}

void Robot_Dynamic_arm::computeJointForwardTorque(StateEstimator &SE, const Matrix<double, 18, 1> wbc_ddq,
                                                  const Matrix<double, 6, 1> _arm_ddq, const Mat43 &wbc_LegForces, const Matrix<double, 6, 1> arm_Gen_F,
                                                  const Mat43 &body_joint_angle, const Mat43 &body_joint_vel, const Vector3d &body_CoMPos,
                                                  const Vector3d &body_CoMVel, const Vector3d &body_CoMAngle,
                                                  const Vector3d &body_CoMAngularVel_w, const Matrix<double, 6, 1> arm_joint_angle,
                                                  const Matrix<double, 6, 1> arm_joint_vel)
{
  // computeDynamic(wbc_ddq, wbc_LegForces, body_joint_angle, body_joint_vel, body_CoMPos,body_CoMVel,body_CoMAngle,
  //                body_CoMAngularVel_w);
  // 下面用于静力学，但是静力学和动力学极有可能不是一套参数，
  // 需要将雅克比矩阵传入进来，但是在成员函数规范化后有点麻烦，就暂时不使用
  
    temp_trans.setZero();
    temp_trans.block<3, 3>(0, 0) = I;
    temp_trans.block<3, 3>(3, 3) = SE.RotationMatrix;
  if (isUseDynamic)
  {
    computeDynamic(wbc_ddq, _arm_ddq, wbc_LegForces, arm_Gen_F, body_joint_angle, body_joint_vel, body_CoMPos, body_CoMVel, body_CoMAngle,
                   body_CoMAngularVel_w, arm_joint_angle, arm_joint_vel);
  }
  else
  {
    for (size_t i = 0; i < 4; i++)
    {
      BodyJointFeedforwardTorque.row(i) = -wbc_LegForces.row(i) * SE.RotationMatrix * SE.Jacobian[i];
      // BodyJointFeedforwardTorque.row(i) = -wbc_LegForces.row(i) * SE.Jocabian_leg[i];
    }
    
    ArmJointTorque = SE.C_arm - (SE.J_object.transpose()).block(18, 0, 6, 6) * temp_trans * arm_Gen_F;
    // ArmJointTorque = SE.C_arm;
    // ArmJointTorque=(SE.J_object.transpose()).block(18,0,6,6)*arm_Gen_F;
    // printinfocolor((SE.J_object.transpose()).block(18, 0, 6, 6) * temp_trans, GREEN);
    // printinfo(arm_Gen_F.transpose());
  }
}

void Robot_Dynamic_arm::setTorqueLimit(Mat43 &Torque)
{
  // 对力矩进行限制
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (Torque(i, j) > BodyJointTorque_limit_MAX(i, j))
      {
        Torque(i, j) = BodyJointTorque_limit_MAX(i, j);
      }
      else if (Torque(i, j) < BodyJointTorque_limit_MIN(i, j))
      {
        Torque(i, j) = BodyJointTorque_limit_MIN(i, j);
      }
    }
  }
}

void Robot_Dynamic_arm::computeDynamic(const Matrix<double, 18, 1> wbc_ddq,
                                       const Matrix<double, 6, 1> arm_ddq,
                                       const Mat43 &wbc_LegForces,
                                       const Matrix<double, 6, 1> arm_Gen_F,
                                       const Mat43 &body_joint_angle,
                                       const Mat43 &body_joint_vel,
                                       const Vector3d &_CoMPos,
                                       const Vector3d &_CoMVel,
                                       const Vector3d &body_CoMAngle,
                                       const Vector3d &body_CoMAngularVel_w,
                                       const Matrix<double, 6, 1> arm_joint_angle,
                                       const Matrix<double, 6, 1> arm_joint_vel)
{
  q.block<3, 1>(0, 0) = _CoMPos;                       // p
  dq.block<3, 1>(0, 0) = _CoMVel;                      // v
  ddq_d.block<3, 1>(0, 0) = wbc_ddq.block<3, 1>(0, 0); // a

  auto qq = body_CoMAngle;
  q.block<4, 1>(3, 0) = EulerToQuaternion(qq); // theta
  dq.block<3, 1>(3, 0) = body_CoMAngularVel_w; // omega
  ddq_d.block<3, 1>(3, 0) = wbc_ddq.block<3, 1>(3, 0);

  for (int i = 0; i < 4; i++)
  {
    q.block<3, 1>(7 + 3 * i, 0) = (body_joint_angle.row(i)).transpose();
    dq.block<3, 1>(6 + 3 * i, 0) = (body_joint_vel.row(i)).transpose();
    ddq_d.block<3, 1>(6 + 3 * i, 0) =
        (wbc_ddq.block<3, 1>(6 + 3 * i, 0).transpose() + JointddqPD.row(i))
            .transpose();
  }
  q.block<6, 1>(19, 0) = arm_joint_angle;
  dq.block<6, 1>(18, 0) = arm_joint_vel;
  ddq_d.block<6, 1>(18, 0) = arm_ddq;
  mapF2T(wbc_LegForces, arm_Gen_F);

  // 不用每个量都计算出来，直接用rnea进行计算
  //  auto M = pinocchio::crba(model, data, q);
  //  M.triangularView<Lower>() = M.triangularView<Upper>().transpose();
  //  auto C = pinocchio::nonLinearEffects(model, data, q, dq);
  //  auto G = pinocchio::computeGeneralizedGravity(model, data, q);
  //  auto tt = M * ddq_d + C - ExtTorque;

  auto T = rnea(model, data, q, dq, ddq_d) - ExtTorque;
//   Eigen::Matrix<double, 24, 1> T;
 
//  T = rnea(model, data, q, dq, ddq_d) - ExtTorque;
// T =  - ExtTorque;

  // printinfo(ddq_d.transpose());

  auto t_f_leg = rnea(model, data, q, dq, ddq_d).block(6, 0, 12, 1);

  // printinfo(t_f_leg.transpose());

  for (int i = 0; i < 4; i++)
  {
    BodyJointFeedforwardTorque.row(i) = T.block<3, 1>(6 + 3 * i, 0);
  }
  ArmJointTorque = T.block<6, 1>(18, 0);
}

void Robot_Dynamic_arm::mapF2T(const Mat43 &extF, const Matrix<double, 6, 1> arm_Gen_F)
{
  // forwardKinematics(model, data, q, dq, ddq_d);
  framesForwardKinematics(model, data, q);

  computeJointJacobians(model, data, q);

  Matrix<double, 6, 24> Jf[5];
  Matrix<double, 6, 1> ff[5];
  Matrix<double, 24, 1> tt[5];

  ExtTorque.setZero();
  for (int i = 0; i < 4; i++)
  {
    ff[i].setZero();
    ff[i].block(0, 0, 3, 1) = extF.row(i).transpose();
    Jf[i].setZero();
  }
  ff[4] = arm_Gen_F;

  getFrameJacobian(model, data, model.getFrameId("FR_foot"), LOCAL_WORLD_ALIGNED, Jf[0]);
  getFrameJacobian(model, data, model.getFrameId("RR_foot"), LOCAL_WORLD_ALIGNED, Jf[1]);
  getFrameJacobian(model, data, model.getFrameId("RR_foot"), LOCAL_WORLD_ALIGNED, Jf[2]);
  getFrameJacobian(model, data, model.getFrameId("RL_foot"), LOCAL_WORLD_ALIGNED, Jf[3]);
  getFrameJacobian(model, data, model.getFrameId("gripperMover"), LOCAL_WORLD_ALIGNED, Jf[4]);

  for (int i = 0; i < 4; i++)
  {
    tt[i] = Jf[i].transpose() * ff[i];
    ExtTorque += tt[i];
  }
  tt[4] = Jf[4].transpose() * temp_trans * ff[4];
  // ExtTorque += tt[4];
  // printinfo(ExtTorque.transpose());
}

void Robot_Dynamic_arm::JointPdController(const Matrix<double, 4, 3> &q,
                                          const Matrix<double, 4, 3> &q_d,
                                          const Matrix<double, 4, 3> &dq,
                                          const Matrix<double, 4, 3> &dq_d)
{
  for (int i = 0; i < 4; i++)
  {
    JointddqPD.row(i) = (q_d.row(i) - q.row(i)) * kp_joint +
                        (dq_d.row(i) - dq.row(i)) * kd_joint;
  }
}

void Robot_Dynamic_arm::computeArmJointPdForces(const Matrix<double, 6, 1> &arm_joint,
                                                const Matrix<double, 6, 1> &arm_joint_vel,
                                                const Matrix<double, 6, 1> &arm_joint_d,
                                                const Matrix<double, 6, 1> &arm_joint_vel_d)
{
  ArmJointFeedbackTorque = k_p_armjoint.asDiagonal() * (arm_joint_d - arm_joint) + k_v_armjoint.asDiagonal() * (arm_joint_vel_d - arm_joint_vel);
}

// 经过判断，自己绊脚的行为是因为关节pd造成的，系数太大
void Robot_Dynamic_arm::computeJointPdForces(const Mat43 &body_joint_angle,
                                             const Mat43 &body_joint_vel,
                                             const Mat43 &body_joint_angle_d,
                                             const Mat43 &body_joint_vel_d)
{
  for (int i = 0; i < 4; i++)
    BodyJointFeedbackTorque.row(i) =
        (body_joint_angle_d.row(i) - body_joint_angle.row(i)) * k_p_joint +
        (body_joint_vel_d.row(i) - body_joint_vel.row(i)) * k_v_joint;
}

Robot_Dynamic_arm::~Robot_Dynamic_arm() {}