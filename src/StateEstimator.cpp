/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     StateEsimator.cpp
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
#include "../include/StateEstimator.h"
StateEstimator::StateEstimator(/* args */)
{
  cycle = 0;
  Last_body_CoMAngle.setZero();
  body_CoMAngle.setZero();
  body_CoMVel.setZero();
  body_CoMPos.setZero();
  body_CoMPos(2) = 0.145;
  X_last_k_1.setZero();
  X_last_k_1(2) = 0.145;
  body_CoM_offset.setZero();
  last_body_joint_angle.setZero();
  ground_pitch = 0;
  ground_roll = 0;
  init_Read_Joint = true;

  A.setIdentity();
  P.setIdentity();
  Q = 0.0001 * Q.setIdentity(9, 9);
  Q(0, 0) = 0.1;
  Q(5, 5) = 0.001;
  P_pre.setIdentity();
  H.setIdentity();
  R.setIdentity();
  R = 0.5 * R.setIdentity(9, 9);
  R(5, 5) = 0.1;
  I.setIdentity();

  A.block<3, 3>(0, 3) = dt * Matrix3d::Identity();

  B.setZero();
  B.block<3, 3>(0, 0) = 0.5 * dt * dt * Matrix3d::Identity();
  B.block<3, 3>(3, 0) = dt * Matrix3d::Identity();
  B.block<3, 3>(6, 3) = dt * Matrix3d::Identity();

  pinocchio::urdf::buildModel(urdf_filename, model); // 加载urdf模型
  data = pinocchio::Data(model);
  q.setZero(model.nq);  // np：Dimension of the configuration vector
                        // representation.配置向量表示的维度
  dq.setZero(model.nv); // nv：Dimension of the velocity vector
                        // space.速度向量空间的维度
  Arm_link_mass[0](2) = 0.47247481;
  Arm_link_mass[1](2) = 0.67332551;
  Arm_link_mass[2](2) = 1.19132258;
  Arm_link_mass[3](2) = 0.83940874;
  Arm_link_mass[4](2) = 0.56404563;
  Arm_link_mass[5](2) = 0.38938492;
  Arm_link_mass[6](2) = 0.28875807;

  Arm_link_com_wtlastf[0] << 0, 0, 0.0585 / 2.0;
  Arm_link_com_wtlastf[1] << 0, 0, 0.045 / 2.0;
  Arm_link_com_wtlastf[2] << -0.35 / 2.0, 0, 0;
  Arm_link_com_wtlastf[3] << 0.218 / 2.0, 0, 0.057 / 2.0;
  Arm_link_com_wtlastf[4] << 0.07 / 2, 0, 0;
  Arm_link_com_wtlastf[5] << 0.0492 / 2.0, 0, 0;
  Arm_link_com_wtlastf[6] << 0.051 / 2.0, 0, 0;
  LegForce_estimate_method = 0;
  LegForce_z_w_estimate.setConstant(200);
}

void StateEstimator::state_calc(Vector4i &LegPhase, Robot_Parameters &Para)
{
  CollisionCheck();
  RotationMatrix = calc_RotationMatrix(body_CoMAngle);
  RotationMatrix1 = calc_RotationMatrix1(body_CoMAngle);
  calc_Toe_Pos(body_joint_angle, Para);
  calc_Jocabian_dJacobian(body_joint_angle, body_joint_vel, Para);
  calc_Toe_Vel(body_joint_vel);
  convert2worldframe(Para);
  KalmanFilter(LegPhase, body_CoMAcc, body_CoMAngularVel_w, body_CoMAngle, body_CoMVel, ToeVel_wH);
  otherEstimation();
  // Estimate_Object_State(Para);
  vmc_state <<  body_CoMAngle, body_CoMPos, body_CoMAngularVel_w, body_CoMVel, Para.grav;
  mpc_state << body_CoMAngle, body_CoMPos, body_CoMAngularVel_w, body_CoMVel, Para.grav;
  nmpc_state << body_CoMAngle, body_CoMPos, body_CoMAngularVel_w, body_CoMVel, Para.grav;
  nmpc_DRBM_state << Rz * body_CoMAngle, body_CoMPos, body_CoMAngularVel_w, body_CoMVel, Para.grav,
      object_CoMAngle, object_CoMPos, object_CoMAngularVel, object_CoMVel;
}

void StateEstimator::CollisionCheck()
{
  if (LegForce_estimate_method == 0)
  {
    bH2Toe_RotationMatrix(body_joint_angle);
    for (int i = 0; i < 4; i++)
    {
      RealLegContactForces_w.row(i) = (RotationMatrix * bH2Toe_R[i] *
                                       (RealLegContactForces.row(i).transpose()))
                                          .transpose();
    }
  }
  else if (LegForce_estimate_method == 1)
  {
    RealLegContactForces_w.col(2) = LegForce_z_w_estimate;
  }
  for (int i = 0; i < 4; i++)
  {
    if (abs(RealLegContactForces_w(i, 2)) > 30 && (abs(RealLegContactForces_w(i, 2) - LastRealLegContactForces(i, 2)) > 10))
    {
      ForceCount[i]++;
    }

    if (abs(RealLegContactForces_w(i, 2)) > 50 || ForceCount[i] >= 2)
    {
      LegContactState(i) = 1;
      ForceCount[i] = 0;
    }
    else
    {
      LegContactState(i) = 0;
      // ForceCount[i] = 0;
    }
  }

  // printinfo(RealLegContactForces_w);

  LastRealLegContactForces = RealLegContactForces_w;
}

void StateEstimator::Estimate_Object_State(const Robot_Parameters &Para)
{
  q.block<3, 1>(0, 0) = zero;  // p
  dq.block<3, 1>(0, 0) = body_CoMVel; // v
  // q.block<3, 1>(0, 0) = zero;
  // dq.block<3, 1>(0, 0) = zero;

  // q.block<4, 1>(3, 0) = EulerToQuaternion(body_CoMAngle); // theta
  q.block<4, 1>(3, 0) = EulerToQuaternion(zero); // theta
  dq.block<3, 1>(3, 0) = body_CoMAngularVel_w;            // omega
  // q.block<4, 1>(3, 0) = EulerToQuaternion(zero);
  // dq.block<3, 1>(3, 0) = zero;

  for (int i = 0; i < 4; i++)
  {
    q.block<3, 1>(7 + 3 * i, 0) = (body_joint_angle.row(i)).transpose();
    dq.block<3, 1>(6 + 3 * i, 0) = (body_joint_vel.row(i)).transpose();
  }

  // q.block<6, 1>(19, 0) = arm_joint_angle;
  // dq.block<6, 1>(18, 0) = arm_joint_vel;

  forwardKinematics(model, data, q);
  framesForwardKinematics(model, data, q);

  // Matrix3d rotation_matrix = data.oMf[model.getFrameId("gripperMover")].rotation();
  // object_CoMPos = data.oMf[model.getFrameId("gripperMover")].translation() - body_CoMPos;
  // object_CoMAngle = pinocchio::rpy::matrixToRpy(EulerToRotation(body_CoMAngle).transpose() * rotation_matrix);
  // object_CoMPos = data.oMf[model.getFrameId("gripperMover")].translation();
  // object_CoMAngle = pinocchio::rpy::matrixToRpy(rotation_matrix);

  // object_CoMPos = data.oMf[model.getFrameId("gripperMover")].translation();
  // object_CoMAngle = pinocchio::rpy::matrixToRpy(rotation_matrix);
  // object_CoMAngle = pinocchio::rpy::matrixToRpy(rotation_matrix);  //这里不需要转换，因为转换之后就变成机身坐标系了，我希望都在世界坐标系下表达，
  // 但是这样的话姿态就会保持不变，因此还是使用转换后的，或者说机身坐标系下的表达
  object_CoMPos_wrtB = object_CoMPos;
  // object_CoMPos_wrtB=object_CoMPos-body_CoMPos;

  computeJointJacobians(model, data, q);
  computeJointJacobiansTimeVariation(model, data, q, dq);

  // auto C = pinocchio::nonLinearEffects(model, data, q, dq);
  // C_arm = C.tail(6);

  getFrameJacobian(model, data, model.getFrameId("FR_foot"),
                   LOCAL_WORLD_ALIGNED, Jf[0]);
  getFrameJacobian(model, data, model.getFrameId("RR_foot"),
                   LOCAL_WORLD_ALIGNED, Jf[1]);
  getFrameJacobian(model, data, model.getFrameId("FL_foot"),
                   LOCAL_WORLD_ALIGNED, Jf[2]);
  getFrameJacobian(model, data, model.getFrameId("RL_foot"),
                   LOCAL_WORLD_ALIGNED, Jf[3]);

  for (int i = 0; i < 4; i++)
  {
    Jocabian_leg[i] = Jf[i].block<3, 3>(0, 6 + 3 * i);
  }

  printinfo(Jacobian[0]);
  printinfo(Jocabian_leg[0]);


  // printinfocolor(J_object.block(3,0,3,6),RED);

  // object_CoMVel = (J_object * dq).head(3)-body_CoMVel;
  // object_CoMAngularVel = (J_object * dq).tail(3)-body_CoMAngularVel_w;

  object_CoMVel = (J_object * dq).head(3) - body_CoMVel;
  object_CoMAngularVel = (J_object * dq).tail(3) - body_CoMAngularVel_w;

  // object_CoMVel = getVelocity(model,data,model.getFrameId("gripperMover"),LOCAL_WORLD_ALIGNED).linear();
  // object_CoMAngularVel = getVelocity(model,data,model.getFrameId("gripperMover"),LOCAL_WORLD_ALIGNED).angular();

  Vector3d toe[4];
  toe[0] = data.oMf[model.getFrameId("FR_foot")].translation() - body_CoMPos;
  toe[1] = data.oMf[model.getFrameId("RR_foot")].translation() - body_CoMPos;
  toe[2] = data.oMf[model.getFrameId("FL_foot")].translation() - body_CoMPos;
  toe[3] = data.oMf[model.getFrameId("RL_foot")].translation() - body_CoMPos;
  // for(int i=0;i<4;i++){
  //   ToePos_wB.row(i)=toe[i].transpose();
  // }
  // Estimate_Object2Body_F(Para);
}

Matrix<double, 6, 1> StateEstimator::updateObject(const Matrix<double, 6, 1> &arm_joint)
{
  q.block<6, 1>(19, 0) = arm_joint;
  forwardKinematics(model, data, q);
  framesForwardKinematics(model, data, q);
  computeJointJacobians(model, data, q);
  getFrameJacobian(model, data, model.getFrameId("gripperMover"),
                   LOCAL_WORLD_ALIGNED, J_object_temp);
  Matrix3d rotation_matrix_temp = data.oMf[model.getFrameId("gripperMover")].rotation();
  Vector3d object_CoMPos_temp = data.oMf[model.getFrameId("gripperMover")].translation() - body_CoMPos;
  Vector3d object_CoMAngle_temp = pinocchio::rpy::matrixToRpy(EulerToRotation(body_CoMAngle).transpose() * rotation_matrix_temp);
  Matrix<double, 6, 1> Object_p_q_temp;
  Object_p_q_temp << object_CoMPos_temp, object_CoMAngle_temp;
  return Object_p_q_temp;
}

void StateEstimator::Estimate_Object2Body_F(const Robot_Parameters &Para)
{
  Matrix3d rotation[7];
  Vector3d link_origin[7];
  Arm2BodyGForces.setZero();
  for (int i = 0; i < 7; i++)
  {
    rotation[i] = data.oMf[model.getFrameId("link00") + 2 * i].rotation();
    link_origin[i] = data.oMf[model.getFrameId("link00") + 2 * i].translation();
    Arm_link_com_pos_wtb[i] = link_origin[i] + rotation[i] * Arm_link_com_wtlastf[i] - body_CoMPos;
    Arm2BodyGForces.segment(0, 3) += Arm_link_mass[i] * Para.grav;
    Arm2BodyGForces.segment(3, 3) += cross_product(Arm_link_com_pos_wtb[i]) * Arm_link_mass[i] * Para.grav;
  }
}

void StateEstimator::slope_angle_cal(const Mat43 &_ToePos_wB)
{
  double x1 = _ToePos_wB(0, 0); // px(1) 四个足端坐标
  double x2 = _ToePos_wB(1, 0); // px(2)
  double x3 = _ToePos_wB(2, 0); // px(3)
  double x4 = _ToePos_wB(3, 0); // px(4)
  double y1 = _ToePos_wB(0, 1); // py(1)
  double y2 = _ToePos_wB(1, 1); // py(2)
  double y3 = _ToePos_wB(2, 1); // py(3)
  double y4 = _ToePos_wB(3, 1); // py(4)
  double z1 = _ToePos_wB(0, 2); // pz(1)
  double z2 = _ToePos_wB(1, 2); // pz(2)
  double z3 = _ToePos_wB(2, 2); // pz(3)
  double z4 = _ToePos_wB(3, 2); // pz(4)
  int sig_pitch, sig_roll;      // 符号变量 正负1
  Matrix<double, 3, 3> M, mi;
  Vector3d b, a, c;
  Vector3d Nxoy, Ns, Ns_pitch, Ns_roll;

  //    b << x1 + x2 + x3 + x4, y1 + y2 + y3 + y4, z1 + z2 + z3 + z4;
  //    M << x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4, x1 * y1 + x2 * y2 + x3 * y3
  //    + x4 * y4,
  //        x1 * z1 + x2 * z2 + x3 * z3 + x4 * z4, x1 * y1 + x2 * y2 + x3 * y3 +
  //        x4 * y4, y1 * y1 + y2 * y2 + y3 * y3 + y4 * y4, y1 * z1 + y2 * z2 +
  //        y3 * z3 + y4 * z4, x1 * z1 + x2 * z2 + x3 * z3 + x4 * z4, y1 * z1 +
  //        y2 * z2 + y3 * z3 + y4 * z4, z1 * z1 + z2 * z2 + z3 * z3 + z4 * z4;

  //    a = M.inverse() * b;
  //    mi = M.inverse() * M;

  //    Nxoy << 0, 0, 1;
  //    Ns << a(0), a(1), a(2);    // 空间任意平面的法向量  z = a1 + a2*x + a3*y
  //    Ns_pitch << a(0), 0, a(2); // Ns在xoz平面投影 y为0
  //    Ns_roll << 0, a(1), a(2);  // Ns在yoz平面投影 x为0
  //    ground_pitch = pi - acos(Nxoy.dot(Ns_pitch) / (Nxoy.norm() *
  //    Ns_pitch.norm())); //Nxoy.norm()模 ground_roll  = pi -
  //    acos(Nxoy.dot(Ns_roll)  / (Nxoy.norm() * Ns_roll.norm())); std::cout
  //    <<"ground pitch 1: "<<acos(Nxoy.dot(Ns_pitch) / (Nxoy.norm() *
  //    Ns_pitch.norm()))<< std::endl;

  // 测试 和伯韬师兄上面的不太一样，上面的没找到出处
  b << x1 * z1 + x2 * z2 + x3 * z3 + x4 * z4,
      y1 * z1 + y2 * z2 + y3 * z3 + y4 * z4, z1 + z2 + z3 + z4;

  M << x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4,
      x1 * y1 + x2 * y2 + x3 * y3 + x4 * y4, x1 + x2 + x3 + x4,
      x1 * y1 + x2 * y2 + x3 * y3 + x4 * y4,
      y1 * y1 + y2 * y2 + y3 * y3 + y4 * y4, y1 + y2 + y3 + y4,
      x1 + x2 + x3 + x4, y1 + y2 + y3 + y4, 4;

  c = M.inverse() * b;
  a << c(0) / c(2), c(1) / c(2), -1 / c(2);
  mi = M.inverse() * M;

  Nxoy << 0, 0, 1;
  Ns << a(0), a(1), a(2);    // 空间任意平面的法向量  z = a1 + a2*x + a3*y
  Ns_pitch << a(0), 0, a(2); // Ns在xoz平面投影
  Ns_roll << 0, a(1), a(2);  // Ns在yoz平面投影
  ground_pitch = acos(Nxoy.dot(Ns_pitch) / (Nxoy.norm() * Ns_pitch.norm()));
  ground_roll = acos(Nxoy.dot(Ns_roll) / (Nxoy.norm() * Ns_roll.norm()));
  //    std::cout <<"ground pitch 1: "<<acos(Nxoy.dot(Ns_pitch) / (Nxoy.norm() *
  //    Ns_pitch.norm()))<< std::endl;

  if (z1 - z2 + z3 - z4 > 0) // 前进方向 向上的斜坡 \_ <-
    sig_pitch = -1;
  else // 前进方向 向下的斜坡 / <-
    sig_pitch = 1;

  if (z1 + z2 - z3 - z4 > 0)
    sig_roll = -1;
  else
    sig_roll = 1;

  ground_pitch = sig_pitch * ground_pitch;
  ground_roll = sig_roll * ground_roll;
  //    std::cout <<"ground pitch : "<<ground_pitch<< "  ground roll:
  //    "<<ground_roll<<std::endl;
}

void StateEstimator::KalmanFilter(Vector4i &LegPhase, Vector3d &_CoMAcc,
                                  Vector3d &_CoMAngularVel_w,
                                  Vector3d &_CoMAngle, Vector3d &_CoMVel,
                                  Mat43 &_ToeVel_wH)
{
  body_CoMVel_fom_toe.setZero();
  if (LegPhase.sum() == 0)
  {
    body_CoMVel_fom_toe = _CoMVel;
  }
  else
  {
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        if (LegPhase(i) == 1)
        {
          body_CoMVel_fom_toe(j) -=
              ToeVel_wB(i, j); // 因为质心位置等于四腿中触地位置之和
        }
      }
    }

    body_CoMVel_fom_toe = body_CoMVel_fom_toe / double(LegPhase.sum());
  }

  // body_CoMVel_fom_toe = _CoMVel;

  U.block<3, 1>(0, 0) = _CoMAcc;                           // 前三行，质心加速度
  U.block<3, 1>(3, 0) = Rz.transpose() * _CoMAngularVel_w; // 后三行，身体角速度
  Z_k.block<3, 1>(0, 0) = X_last_k_1.block<3, 1>(0, 0) +
                          body_CoMVel_fom_toe * dt; // 位置递增，速度 X 间隔时间
  Z_k.block<3, 1>(3, 0) = body_CoMVel_fom_toe;
  Z_k.block<3, 1>(6, 0) = _CoMAngle;
  // Z_k.block<3, 1>(6, 0) = Rz*_CoMAngle;//错误的，使得机身旋转到一定程度就崩溃

  // 卡尔曼滤波核心

  // 预测
  // 1.先验
  X_pre_k = A * X_last_k_1 + B * U;
  // 2.更新先验误差协方差
  P_pre = A * P * A.transpose() + Q;

  // 校正
  // 1.计算卡尔曼增益
  K = P_pre * H.transpose() * (H * P_pre * H.transpose() + R).inverse();
  // 2.后验估计
  X_k = X_pre_k + K * (Z_k - H * X_pre_k);
  // 3.更新误差协方差
  P = (I - K * H) * P_pre;

  // 估计值更新
  for (int i = 0; i < 3; i++) // xyz
  {
    body_CoMPos(i) = X_k(i, 0);
    _CoMVel(i) = X_k(i + 3, 0);
    _CoMAngle(i) = X_k(i + 6, 0);
  }

  body_CoMPos(2) = 0;
  if (LegPhase.sum() == 0)
  {
    body_CoMPos(2) = body_CoMPos(2) / 4;
  }
  else
  {
    for (int i = 0; i < 4; i++)
    {
      if (LegPhase(i) == 1)
      {
        body_CoMPos(2) -= ToePos_wB(i, 2);
      }
    }
    body_CoMPos(2) = body_CoMPos(2) / double(LegPhase.sum());
  }

  // 更新
  X_last_k_1 = X_k;
  Last_body_CoMAngle = _CoMAngle;
}

void StateEstimator::convert2worldframe(Robot_Parameters &Para)
{
  body_CoMAcc = RotationMatrix * body_CoMAcc.eval();
  body_CoMAcc(2) += -9.8;
  Inertia_w = RotationMatrix * Para.Inertia * (RotationMatrix.transpose());

  // body_CoMVel = Rz.transpose() * body_CoMVel.eval();
  // body_CoMPos = Rz.transpose() * body_CoMPos.eval();  // 读出来的值需要旋转一下,?位置为什么也需要旋转
}

void StateEstimator::otherEstimation()
{
  body_CoM_Quaternion = EulerToQuaternion(body_CoMAngle);
  object_CoM_Quaternion = EulerToQuaternion(object_CoMAngle);
}

void StateEstimator::calc_Toe_Vel(const Mat43 &dq)
{
  body_CoMAngularVel_w = RotationMatrix * body_CoMAngularVel;
  for (int i = 0; i < 4; i++)
  {
    // ToeVel_bH.row(i) = dq.row(i) * Jacobian[i].transpose();
    // // 感觉有些问题，比如没有包含旋转矩阵微分的部分
    // ToeVel_wH.row(i) = ToeVel_bH.row(i) * RotationMatrix.transpose();

    // ToeVel_wB.row(i) = ToeVel_wH.row(i);  //?

    ToeVel_bH.row(i) = (Jacobian[i] * dq.row(i).transpose()).transpose();

    ToeVel_wH.row(i) =
        (RotationMatrix * (ToeVel_bH.row(i).transpose())).transpose() + (cross_product(body_CoMAngularVel_w) * RotationMatrix * (ToePos_bH.row(i).transpose())).transpose();
    ToeVel_wB.row(i) = (RotationMatrix * (ToeVel_bH.row(i).transpose())).transpose() + (cross_product(body_CoMAngularVel_w) * RotationMatrix * ((ToePos_bH + BaseMat).row(i).transpose())).transpose();
  }
}

Matrix3d StateEstimator::calc_RotationMatrix(Vector3d &_CoMAngle)
{
  const AngleAxisd roll(_CoMAngle[0], Vector3d::UnitX());
  const AngleAxisd pitch(_CoMAngle[1], Vector3d::UnitY());
  // 定义世界坐标系z轴随身体旋转
  const AngleAxisd yaw(_CoMAngle[2], Vector3d::UnitZ());

  // 按ZYX顺序的旋转矩阵，四元数转换为旋转矩阵
  return (yaw * pitch * roll).toRotationMatrix();
}

Matrix3d StateEstimator::calc_RotationMatrix1(Vector3d &_CoMAngle)
{
  const AngleAxisd roll(_CoMAngle[0], Vector3d::UnitX());
  const AngleAxisd pitch(_CoMAngle[1], Vector3d::UnitY());
  // 定义世界坐标系z轴随身体旋转
  const AngleAxisd yaw(0, Vector3d::UnitZ());

  // 按ZYX顺序的旋转矩阵，四元数转换为旋转矩阵
  return (yaw * pitch * roll).toRotationMatrix();
}

Mat43 StateEstimator::IK(Mat43 &p, Robot_Parameters &Para)
{
  double L, L1, L2;

  Mat43 q;
  for (int i = 0; i < 2; i++)
  {
    L = pow(pow(p(i, 1), 2) + pow(p(i, 2), 2), 0.5);
    L1 = pow(pow(L, 2) - pow(Para.abd_offset, 2), 0.5);
    L2 = pow(pow(L1, 2) + pow(p(i, 0), 2), 0.5);
    if (p(i, 1) < 0)
      q(i, 0) = -acos(Para.abd_offset / L) - asin(p(i, 2) / L);
    else
      q(i, 0) = M_PI - (acos(Para.abd_offset / L) - asin(p(i, 2) / L));

    q(i, 1) = -(acos((pow(Para.Thigh_Length, 2) + pow(L2, 2) -
                      pow(Para.Calf_Length, 2)) /
                     (2 * Para.Thigh_Length * L2)) -
                atan(p(i, 0) / L1));
    q(i, 2) = -(acos((pow(Para.Thigh_Length, 2) + pow(Para.Calf_Length, 2) -
                      pow(L2, 2)) /
                     (2 * Para.Thigh_Length * Para.Calf_Length)) -
                M_PI);
  }

  for (int i = 2; i < 4; i++)
  {
    L = pow(pow(p(i, 1), 2) + pow(p(i, 2), 2), 0.5);
    L1 = pow(pow(L, 2) - pow(Para.abd_offset, 2), 0.5);
    L2 = pow(pow(L1, 2) + pow(p(i, 0), 2), 0.5);

    if (p(i, 1) > 0)
      q(i, 0) = acos(Para.abd_offset / L) + asin(p(i, 2) / L);
    else
      q(i, 0) = -(M_PI - (acos(Para.abd_offset / L) - asin(p(i, 2) / L)));
    q(i, 1) = -(acos((pow(Para.Thigh_Length, 2) + pow(L2, 2) -
                      pow(Para.Calf_Length, 2)) /
                     (2 * Para.Thigh_Length * L2)) -
                atan(p(i, 0) / L1));
    q(i, 2) = -(acos((pow(Para.Thigh_Length, 2) + pow(Para.Calf_Length, 2) -
                      pow(L2, 2)) /
                     (2 * Para.Thigh_Length * Para.Calf_Length)) -
                M_PI);
  }
  return q;
}

void StateEstimator::bH2Toe_RotationMatrix(const Mat43 &q)
{
  for (int i = 0; i < 4; i++)
  {
    bH2Toe_R[i](0, 0) = cos(-q(i, 2) - q(i, 1)); // 正运动学的旋转矩阵
    bH2Toe_R[i](0, 1) = 0;
    bH2Toe_R[i](0, 2) = sin(-q(i, 2) - q(i, 1));
    bH2Toe_R[i](1, 0) = sin(-q(i, 2) - q(i, 1)) * sin(q(i, 0));
    bH2Toe_R[i](1, 1) = cos(q(i, 0));
    bH2Toe_R[i](1, 2) = -cos(-q(i, 2) - q(i, 1)) * sin(q(i, 0));
    bH2Toe_R[i](2, 0) = -cos(q(i, 0)) * sin(-q(i, 2) - q(i, 1));
    bH2Toe_R[i](2, 1) = sin(q(i, 0));
    bH2Toe_R[i](2, 2) = cos(-q(i, 2) - q(i, 1)) * cos(q(i, 0));
    // cout<<"leg"<<i<<":"<<endl<<bH2Toe_R[i]<<endl;//打印数据
  }
}

void StateEstimator::calc_Toe_Pos(const Mat43 &q, Robot_Parameters &Para)
{
  Rz << cos(body_CoMAngle(2)), -sin(body_CoMAngle(2)), 0, sin(body_CoMAngle(2)),
      cos(body_CoMAngle(2)), 0, 0, 0, 1;
  // 髋关节坐标系下，左右腿髋关节偏置有区别
  for (int i = 0; i < 2; i++)
  {
    ToePos_bH(i, 0) = Para.Thigh_Length * sin(q(i, 1)) +
                      Para.Calf_Length * sin(q(i, 2) + q(i, 1));
    ToePos_bH(i, 1) = (Para.Thigh_Length * cos(q(i, 1)) +
                       Para.Calf_Length * cos(q(i, 2) + q(i, 1))) *
                          sin(q(i, 0)) -
                      Para.abd_offset * cos(q(i, 0));
    ToePos_bH(i, 2) = -(Para.Thigh_Length * cos(q(i, 1)) +
                        Para.Calf_Length * cos(q(i, 2) + q(i, 1))) *
                          cos(q(i, 0)) -
                      Para.abd_offset * sin(q(i, 0));
  }
  for (int i = 2; i < 4; i++)
  {
    ToePos_bH(i, 0) = Para.Thigh_Length * sin(q(i, 1)) +
                      Para.Calf_Length * sin(q(i, 2) + q(i, 1));
    ToePos_bH(i, 1) = (Para.Thigh_Length * cos(q(i, 1)) +
                       Para.Calf_Length * cos(q(i, 2) + q(i, 1))) *
                          sin(q(i, 0)) +
                      Para.abd_offset * cos(q(i, 0));
    ToePos_bH(i, 2) = -(Para.Thigh_Length * cos(q(i, 1)) +
                        Para.Calf_Length * cos(q(i, 2) + q(i, 1))) *
                          cos(q(i, 0)) +
                      Para.abd_offset * sin(q(i, 0));
  }

  // 四个髋关节在身体中心坐标系下的位置坐标
  BaseMat << Para.BodyLength / 2.0 + body_CoM_offset[0],
      -Para.BodyWidth / 2.0 + body_CoM_offset[1], body_CoM_offset[2],

      -Para.BodyLength / 2.0 + body_CoM_offset[0],
      -Para.BodyWidth / 2.0 + body_CoM_offset[1], body_CoM_offset[2],

      Para.BodyLength / 2.0 + body_CoM_offset[0],
      Para.BodyWidth / 2.0 + body_CoM_offset[1], body_CoM_offset[2],

      -Para.BodyLength / 2.0 + body_CoM_offset[0],
      Para.BodyWidth / 2.0 + body_CoM_offset[1], body_CoM_offset[2];
  // for (int i = 0; i < 4; i++) {
  //   ToePos_wH.row(i) =
  //       ToePos_bH.row(i)*(RotationMatrix.transpose());
  //   ToePos_wB.row(i) =
  //       (ToePos_bH + BaseMat).row(i)*(RotationMatrix.transpose());
  // }

  ToePos_wH = ToePos_bH * (RotationMatrix.transpose());
  ToePos_wB = (ToePos_bH + BaseMat) * (RotationMatrix.transpose());
}

void StateEstimator::calc_Jocabian_dJacobian(const Mat43 &q, const Mat43 &dq,
                                             Robot_Parameters &Para)
{
  for (int i = 0; i < 2; i++)
  {
    Jacobian[i](0, 0) = 0;
    Jacobian[i](0, 1) = Para.Thigh_Length * cos(q(i, 1)) +
                        Para.Calf_Length * cos(q(i, 1) + q(i, 2));
    Jacobian[i](0, 2) = Para.Calf_Length * cos(q(i, 1) + q(i, 2));

    Jacobian[i](1, 0) = (Para.Calf_Length * cos(q(i, 1) + q(i, 2)) +
                         Para.Thigh_Length * cos(q(i, 1))) *
                            cos(q(i, 0)) +
                        Para.abd_offset * sin(q(i, 0));
    Jacobian[i](1, 1) = -(Para.Calf_Length * sin(q(i, 1) + q(i, 2)) +
                          Para.Thigh_Length * sin(q(i, 1))) *
                        sin(q(i, 0));
    Jacobian[i](1, 2) =
        -Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * sin(q(i, 0));

    Jacobian[i](2, 0) = (Para.Calf_Length * cos(q(i, 1) + q(i, 2)) +
                         Para.Thigh_Length * cos(q(i, 1))) *
                            sin(q(i, 0)) -
                        Para.abd_offset * cos(q(i, 0));
    Jacobian[i](2, 1) = (Para.Calf_Length * sin(q(i, 1) + q(i, 2)) +
                         Para.Thigh_Length * sin(q(i, 1))) *
                        cos(q(i, 0));
    Jacobian[i](2, 2) =
        Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * cos(q(i, 0));

    dJacobian[i](0, 0) = 0;
    dJacobian[i](0, 1) = -dq(i, 1) * Para.Thigh_Length * sin(q(i, 1)) +
                         Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * (-dq(i, 1) - dq(i, 2));
    dJacobian[i](0, 2) = Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * (-dq(i, 1) - dq(i, 2));

    dJacobian[i](1, 0) =
        dq(i, 0) * Para.abd_offset * cos(q(i, 0)) +
        cos(q(i, 0)) * (-dq(i, 1) * Para.Thigh_Length * sin(q(i, 1)) +
                        Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * (-dq(i, 1) - dq(i, 2))) -
        dq(i, 0) * sin(q(i, 0)) *
            (Para.Calf_Length * cos(q(i, 1) + q(i, 2)) +
             Para.Thigh_Length * cos(q(i, 1)));
    dJacobian[i](1, 1) =
        -sin(q(i, 0)) * (dq(i, 1) * Para.Thigh_Length * cos(q(i, 1)) +
                         Para.Calf_Length * cos(q(i, 1) + q(i, 2)) * (dq(i, 1) + dq(i, 2))) -
        dq(i, 0) * cos(q(i, 0)) *
            (Para.Calf_Length * sin(q(i, 1) + q(i, 2)) +
             Para.Thigh_Length * sin(q(i, 1)));
    dJacobian[i](1, 2) =
        -Para.Calf_Length * cos(q(i, 1) + q(i, 2)) * sin(q(i, 0)) * (dq(i, 1) + dq(i, 2)) -
        dq(i, 0) * Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * cos(q(i, 0));

    dJacobian[i](2, 0) =
        dq(i, 0) * cos(q(i, 0)) *
            (Para.Calf_Length * cos(-(i, 1) + q(i, 2)) +
             Para.Thigh_Length * cos(q(i, 1))) -
        sin(q(i, 0)) * (dq(i, 1) * Para.Thigh_Length * sin(q(i, 1)) +
                        Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * (dq(i, 1) + dq(i, 2))) +
        dq(i, 0) * Para.abd_offset * sin(q(i, 0));
    dJacobian[i](2, 1) =
        cos(q(i, 0)) * (dq(i, 1) * Para.Thigh_Length * cos(q(i, 1)) +
                        Para.Calf_Length * cos(q(i, 1) + q(i, 2)) * (dq(i, 1) + dq(i, 2))) -
        dq(i, 0) * sin(q(i, 0)) *
            (Para.Calf_Length * sin(q(i, 1) + q(i, 2)) +
             Para.Thigh_Length * sin(q(i, 1)));
    dJacobian[i](2, 2) =
        Para.Calf_Length * cos(q(i, 1) + q(i, 2)) * cos(q(i, 0)) * (dq(i, 1) + dq(i, 2)) -
        dq(i, 0) * Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * sin(q(i, 0));
  }
  for (int i = 2; i < 4; i++)
  {
    Jacobian[i](0, 0) = 0;
    Jacobian[i](0, 1) = Para.Thigh_Length * cos(q(i, 1)) +
                        Para.Calf_Length * cos(q(i, 1) + q(i, 2));
    Jacobian[i](0, 2) = Para.Calf_Length * cos(q(i, 1) + q(i, 2));

    Jacobian[i](1, 0) = (Para.Calf_Length * cos(q(i, 1) + q(i, 2)) +
                         Para.Thigh_Length * cos(q(i, 1))) *
                            cos(q(i, 0)) -
                        Para.abd_offset * sin(q(i, 0));
    Jacobian[i](1, 1) = -(Para.Calf_Length * sin(q(i, 1) + q(i, 2)) +
                          Para.Thigh_Length * sin(q(i, 1))) *
                        sin(q(i, 0));
    Jacobian[i](1, 2) =
        -Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * sin(q(i, 0));

    Jacobian[i](2, 0) = (Para.Calf_Length * cos(q(i, 1) + q(i, 2)) +
                         Para.Thigh_Length * cos(q(i, 1))) *
                            sin(q(i, 0)) +
                        Para.abd_offset * cos(q(i, 0));
    Jacobian[i](2, 1) = (Para.Calf_Length * sin(q(i, 1) + q(i, 2)) +
                         Para.Thigh_Length * sin(q(i, 1))) *
                        cos(q(i, 0));
    Jacobian[i](2, 2) =
        Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * cos(q(i, 0));

    dJacobian[i](0, 0) = 0;
    dJacobian[i](0, 1) = -dq(i, 1) * Para.Thigh_Length * sin(q(i, 1)) +
                         Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * (-dq(i, 1) - dq(i, 2));
    dJacobian[i](0, 2) = Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * (-dq(i, 1) - dq(i, 2));

    dJacobian[i](1, 0) =
        -dq(i, 0) * Para.abd_offset * cos(q(i, 0)) +
        cos(q(i, 0)) * (-dq(i, 1) * Para.Thigh_Length * sin(q(i, 1)) +
                        Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * (-dq(i, 1) - dq(i, 2))) -
        dq(i, 0) * sin(q(i, 0)) *
            (Para.Calf_Length * cos(q(i, 1) + q(i, 2)) +
             Para.Thigh_Length * cos(q(i, 1)));
    dJacobian[i](1, 1) =
        -sin(q(i, 0)) * (dq(i, 1) * Para.Thigh_Length * cos(q(i, 1)) +
                         Para.Calf_Length * cos(q(i, 1) + q(i, 2)) * (dq(i, 1) + dq(i, 2))) -
        dq(i, 0) * cos(q(i, 0)) *
            (Para.Calf_Length * sin(q(i, 1) + q(i, 2)) +
             Para.Thigh_Length * sin(q(i, 1)));
    dJacobian[i](1, 2) =
        -Para.Calf_Length * cos(q(i, 1) + q(i, 2)) * sin(q(i, 0)) * (dq(i, 1) + dq(i, 2)) -
        dq(i, 0) * Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * cos(q(i, 0));

    dJacobian[i](2, 0) =
        dq(i, 0) * cos(q(i, 0)) *
            (Para.Calf_Length * cos(-(i, 1) + q(i, 2)) +
             Para.Thigh_Length * cos(q(i, 1))) -
        sin(q(i, 0)) * (dq(i, 1) * Para.Thigh_Length * sin(q(i, 1)) +
                        Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * (dq(i, 1) + dq(i, 2))) -
        dq(i, 0) * Para.abd_offset * sin(q(i, 0));
    dJacobian[i](2, 1) =
        cos(q(i, 0)) * (dq(i, 1) * Para.Thigh_Length * cos(q(i, 1)) +
                        Para.Calf_Length * cos(q(i, 1) + q(i, 2)) * (dq(i, 1) + dq(i, 2))) -
        dq(i, 0) * sin(q(i, 0)) *
            (Para.Calf_Length * sin(q(i, 1) + q(i, 2)) +
             Para.Thigh_Length * sin(q(i, 1)));
    dJacobian[i](2, 2) =
        Para.Calf_Length * cos(q(i, 1) + q(i, 2)) * cos(q(i, 0)) * (dq(i, 1) + dq(i, 2)) -
        dq(i, 0) * Para.Calf_Length * sin(q(i, 1) + q(i, 2)) * sin(q(i, 0));
  }
}

/// @brief
StateEstimator::~StateEstimator() {}