/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     TrajectoryGenerator.cpp
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
#include "../include/TrajectoryGenerator.h"
TrajectoryGenerator::TrajectoryGenerator(/* args */)
{
  vel2pos.setZero();
  angularvel2angle.setZero();
  ToePos_bH_d.setZero();
  ToePos_bH_d.col(2).setConstant(-0.145);
  ToeVel_bH_d.setZero();
  ToeAcc_bH_d.setZero();
  TargetP << 0, 0, 0.145;
  TargetQ.setZero();
  TargetV.setZero();
  TargetW.setZero();
  TargetV_user.setZero();
  TargetW_user.setZero();
  object_TargetP.setZero();
  object_TargetQ.setZero();
  object_TargetV.setZero();
  object_TargetW.setZero();
  body_CoMAngle_d.setZero();
  body_CoMPos_d.setZero();
  body_CoMAcc_d.setZero();
  body_CoMAngle_d_tmp.setZero();
  body_CoMPos_d_tmp.setZero();
  nmpc_body_CoMAngle_d_tmp.setZero();
  nmpc_body_CoMPos_d_tmp.setZero();
  last_body_CoMAngle_d_mpc.setZero();
  last_body_CoMPos_d_mpc.setZero();
  body_CoMAngularAcc_d.setZero();
  SwingOffset.setZero();
  k_p.setZero();
  k_v.setZero();
  k_theta.setZero();
  k_w.setZero();
  init_flag = 1;
  init_flag_object = 1;
  init_flag_object_iter = 1;
  initObjectAngularVel_d.setZero();
  initObjectVel_d.setZero();
  object_CoMAngle_d_tmp.setZero();
  object_CoMPos_d_tmp.setZero();
  UpOrDown_Flag = 0;//初始为Down
}

void TrajectoryGenerator::setTrajectory(
    StateEstimator &SE, Robot_Parameters &Para, bool &ischangegait,
    bool &ischangephase, const Vector4i &LegPhase, const Vector3d &SwingHeight,
    double &time, double &swing_T, int &count)
{
  LinearVelocityCmd(0.5);

  setCoMTrajectory(TargetV, TargetW, TargetP, TargetQ, Para, SE, count);

  setObjectTrajectory(object_TargetV, object_TargetW, SE, Para);

  computeTargetVirtualForces(vmc_state_d, SE.vmc_state);

  computeCoMGenAcc(Gen_vf_d, SE, Para);

  setToeTrajectory(SE, ischangegait, ischangephase, LegPhase, SwingHeight, time,
                   swing_T);

  compute_joint_target(SE, Para, LegPhase, ToePos_bH_d, ToeVel_bH_d, ToeAcc_bH_d);
}

void TrajectoryGenerator::LinearAcc(const double Acc){
  for (int i = 0; i < 3;i++){
    if(TargetV(i)<TargetV_user(i))
      TargetV(i) += Acc * 0.001;
    else if(TargetV(i)>TargetV_user(i))
      TargetV(i) -= Acc * 0.001;
    else if(TargetV(i)==TargetV_user(i))
      TargetV(i) = TargetV_user(i);
    
    if(TargetW(i)<TargetW_user(i))
      TargetW(i) += Acc * 0.001;
    else if(TargetW(i)>TargetW_user(i))
      TargetW(i) -= Acc * 0.001;
    else if(TargetW(i)==TargetW_user(i))
      TargetW(i) = TargetW_user(i);
  }
}

void TrajectoryGenerator::LinearVelocityCmd(const double Acc){
  for (int i = 0; i < 3;i++)
  {
    Linearize(TargetV(i), TargetV_user(i), Acc);
    Linearize(TargetW(i), TargetW_user(i), Acc);
  }

  if(UpOrDown_Flag){
    TargetV(2) = (0.3 - 0.145) / 2.0;
    Linearize(TargetP(2), 0.3, TargetV(2));
    if(TargetP(2)<0.145)
      TargetV(2) = 0;
  }
  else{
    TargetV(2) = -(0.3 - 0.145) / 2.0;
    Linearize(TargetP(2), 0.145, TargetV(2));
    if(TargetP(2)>0.3)
      TargetV(2) = 0;
  }
}

void TrajectoryGenerator::computeCoMGenAcc(const Matrix<double, 6, 1> &VF,
                                           StateEstimator &SE,
                                           Robot_Parameters &Para)
{
  body_CoMAcc_d = VF.block<3, 1>(0, 0) / Para.BQR3_mass;
  body_CoMAngularAcc_d =
      SE.Inertia_w.inverse() *
      (VF.block<3, 1>(3, 0) -
       cross_product(SE.body_CoMAngularVel_w) * SE.Inertia_w * SE.body_CoMAngularVel_w);
  GenCoMAcc << body_CoMAcc_d, body_CoMAngularAcc_d;
}

void TrajectoryGenerator::setObjectTrajectory(const Vector3d &_object_TargetV,
                                              const Vector3d &_object_TargetW,
                                              StateEstimator &SE, Robot_Parameters &Para)
{
  if (init_flag_object)
  {
    initObjectAngle_d = SE.object_CoMAngle;
    // initObjectAngularVel_d = SE.object_CoMAngularVel;
    initObjectPos_d = SE.object_CoMPos;
    // initObjectVel_d = SE.object_CoMVel;
    init_flag_object = 0;
  }
  // ObjectVel_d = initObjectVel_d + SE.RotationMatrix * (_object_TargetV)+SE.body_CoMVel;
  // ObjectAngularVel_d = initObjectAngularVel_d + SE.RotationMatrix * (_object_TargetW)+SE.body_CoMAngularVel_w;
  // printinfo(initObjectPos_d.transpose());
  // printinfo(SE.object_CoMPos.transpose());

  ObjectVel_d = initObjectVel_d + SE.RotationMatrix * (_object_TargetV);
  ObjectAngularVel_d = initObjectAngularVel_d + SE.RotationMatrix * (_object_TargetW);

  object_CoMAngle_d_tmp += SE.Rz.transpose() * ObjectAngularVel_d * dt_NMPC;
  object_CoMPos_d_tmp += ObjectVel_d * dt_NMPC;

  Vector3d delta_p_body; // used as moving distance of body
  delta_p_body.head(2) = SE.body_CoMPos.head(2);
  delta_p_body(2) = 0;

  ObjectPos_d = SE.RotationMatrix * initObjectPos_d + object_CoMPos_d_tmp + SE.RotationMatrix * object_TargetP;
  // ObjectAngle_d = QuaterniondToEuler(Quaterniond(SE.RotationMatrix*EulerToRotation(initObjectAngle_d))) + object_CoMAngle_d_tmp + SE.RotationMatrix * object_TargetQ;
  // ObjectAngle_d = SE.body_CoMAngle + object_CoMAngle_d_tmp + SE.RotationMatrix * object_TargetQ;

  //  ObjectPos_d = initObjectPos_d + object_CoMPos_d_tmp + SE.RotationMatrix * object_TargetP;
  ObjectAngle_d = initObjectAngle_d + object_CoMAngle_d_tmp + SE.RotationMatrix * object_TargetQ;
  // ObjectAngle_d=SE.body_CoMAngle;

  object_CoM_Quaternion_d = EulerToQuaternion(ObjectAngle_d);

  ObjectAcc_d.setZero();

  // // 雅可比迭代得到离目标位姿点最近的点

  // struct timeval t1, t2;
  // gettimeofday(&t1, NULL);

  // temp_arm_joint = SE.arm_joint_angle;

  // int iter = 0;
  // while (1)
  // {
  //   temp_object_state = SE.updateObject(temp_arm_joint);
  //   temp_object_state_error.head(3) = ObjectPos_d - temp_object_state.head(3);
  //   // temp_object_state_error.tail(3)=(EulerToRotation(ObjectAngle_d)*EulerToRotation(temp_object_state.tail(3)).transpose()).eulerAngles(2,1,0);
  //   temp_object_state_error.tail(3) = ObjectAngle_d - temp_object_state.tail(3);
  //   for (int i = 0; i < 3; i++)
  //   {
  //     temp_object_state_error(3 + i) = sign(temp_object_state_error(3 + i)) * min(abs(temp_object_state_error(3 + i)), 2 * pi - abs(temp_object_state_error(3 + i)));
  //   }
  //   if (iter > 20)
  //   {
  //     // cout<<"Break out due to reaching max_numbler_iteration!!!"<<endl;
  //     break;
  //   }
  //   if (temp_object_state_error.norm() < 1e-2)
  //   {
  //     // cout<<"Error is small enough to stop iteration."<<endl;
  //     break;
  //   }

  //   // temp_arm_joint=temp_arm_joint+0.1*((SE.J_object_temp.block(0,18,6,6)).completeOrthogonalDecomposition().pseudoInverse())*temp_object_state_error/0.5;
  //   temp_arm_joint = temp_arm_joint + 0.01 * SE.J_object_temp.block(0, 18, 6, 6).transpose() * ((SE.J_object_temp.block(0, 18, 6, 6) * SE.J_object_temp.block(0, 18, 6, 6).transpose()).inverse()) * temp_object_state_error / 0.5;
  //   iter++;
  // }
  // temp_arm_joint_vel = ((SE.J_object_temp.block(0, 18, 6, 6)).completeOrthogonalDecomposition().pseudoInverse()) * temp_object_state_error / 5.0;

  // if (init_flag_object_iter)
  // {
  //   last_temp_object_state = temp_object_state; // make sure the first iteration right completely
  //   init_flag_object_iter = 0;
  // }

  // // 滤波
  // for (int i = 0; i < 6; i++)
  // {
  //   if (abs(temp_object_state(i) - last_temp_object_state(i)) > 0.03)
  //   {
  //     temp_object_state(i) = last_temp_object_state(i);
  //   }
  // }
  // last_temp_object_state = temp_object_state;

  // // ObjectPos_d = temp_object_state.head(3);

  // // ObjectAngle_d = temp_object_state.tail(3);

  // // printinfo(temp_arm_joint.transpose());

  // gettimeofday(&t2, NULL);
  // // cout << "[Iterative time]: "
  // //      << " ";
  // // cout << double(t2.tv_usec - t1.tv_usec) / 1000 << " " << endl;

  // ObjectDeltaError.head(3) = ObjectPos_d - SE.object_CoMPos; // dp

  // ObjectDeltaError.tail(3) = (EulerToRotation(ObjectAngle_d) * EulerToRotation(SE.object_CoMAngle).transpose()).eulerAngles(2, 1, 0); // dq
  // // ObjectDeltaError.tail(3)=ObjectAngle_d-SE.object_CoMAngle;

  // // printinfo(ObjectAngle_d.transpose());
  // // printinfo(SE.object_CoMAngle.transpose());
  // for (int i = 0; i < 3; i++)
  // {
  //   ObjectDeltaError(3 + i) = sign(ObjectDeltaError(3 + i)) * min(abs(ObjectDeltaError(3 + i)), pi - abs(ObjectDeltaError(3 + i)));
  // }
  nmpc_DRBM_state_d << nmpc_body_CoMAngle_d, nmpc_body_CoMPos_d, body_CoMAngleVel_d, body_CoMVel_d,
      Para.grav, ObjectAngle_d, ObjectPos_d, ObjectAngularVel_d, ObjectVel_d;
}

void TrajectoryGenerator::computeTargetVirtualForces(
    const Matrix<double, robot_state_dim, 1> &_vmc_state_d,
    const Matrix<double, robot_state_dim, 1> &_vmc_state)
{
  Gen_vf_d.block<3, 1>(0, 0) =
      k_p * (_vmc_state_d.block<3, 1>(3, 0) - _vmc_state.block<3, 1>(3, 0)) +
      k_v * (_vmc_state_d.block<3, 1>(9, 0) - _vmc_state.block<3, 1>(9, 0));
  Gen_vf_d.block<3, 1>(3, 0) =
      k_theta *
          (_vmc_state_d.block<3, 1>(0, 0) - _vmc_state.block<3, 1>(0, 0)) +
      k_w * (_vmc_state_d.block<3, 1>(6, 0) - _vmc_state.block<3, 1>(6, 0)); // p,q
  // Gen_vf_d(4) += 1500 * (0.1 - _vmc_state(1));
  // printinfo(_vmc_state_d.transpose());
  // printinfo(_vmc_state.transpose());
  // printinfo(Gen_vf_d);
}

void TrajectoryGenerator::setCoMTrajectory(const Vector3d &_TargetV,
                                           const Vector3d &_TargetW,
                                           const Vector3d &_TargetP,
                                           const Vector3d &_TargetQ,
                                           Robot_Parameters &Para,
                                           StateEstimator &SE, int &count)
{
  Matrix3d tem_Rz = EulerToRotation(Vector3d{0, 0, body_CoMAngle_d(2)});
  body_CoMAngleVel_d = SE.Rz * _TargetW;
  body_CoMVel_d = tem_Rz * _TargetV; // 此处原本是，但是后续应该考虑目标是否与状态无关，有关的话利于稳定，但是可能会造成更大的误差，无关的话理论完备但可能造成更大的误差
  // 如果状态偏了，那么目标不应该跟着偏，目标就是目标，与状态无关
  //  body_CoMAngle_d = SE.body_CoMAngle + body_CoMAngleVel_d * dt;
  //  body_CoMPos_d = SE.body_CoMPos + body_CoMVel_d * dt;
  body_CoMAngle_d_tmp += SE.Rz.transpose() * body_CoMAngleVel_d * dt;
  body_CoMPos_d_tmp += body_CoMVel_d * dt;

  body_CoMAngle_d = body_CoMAngle_d_tmp + SE.Rz * TargetQ;
  body_CoMPos_d = body_CoMPos_d_tmp + SE.Rz * TargetP;
  // body_CoMAngle_d(1) = SE.ground_pitch;
  body_CoMPos_d(2) = _TargetP(2);
  body_CoM_Quaternion_d = EulerToQuaternion(body_CoMAngle_d);
  vmc_state_d << body_CoMAngle_d, body_CoMPos_d, body_CoMAngleVel_d, body_CoMVel_d, Para.grav;

  nmpc_body_CoMAngle_d_tmp += SE.Rz.transpose() * body_CoMAngleVel_d * dt_NMPC;
  nmpc_body_CoMPos_d_tmp += body_CoMVel_d * dt_NMPC;

  nmpc_body_CoMAngle_d = nmpc_body_CoMAngle_d_tmp + SE.Rz * TargetQ;
  nmpc_body_CoMPos_d = nmpc_body_CoMPos_d_tmp + SE.Rz * TargetP;
  // body_CoMAngle_d(1) = SE.ground_pitch;
  nmpc_body_CoMPos_d(2) = _TargetP(2);
  nmpc_state_d << nmpc_body_CoMAngle_d, nmpc_body_CoMPos_d, body_CoMAngleVel_d, body_CoMVel_d, Para.grav;

  if (count == 10)
  {
    for (int i = 0; i < mpc_Horizon; i++)
    {
      body_CoMAngle_d_mpc = last_body_CoMAngle_d_mpc + (i+1) * SE.Rz.transpose() *body_CoMAngleVel_d * dt_MPC +
                            SE.Rz * TargetQ;
      body_CoMPos_d_mpc =
          last_body_CoMPos_d_mpc + (i+1)* body_CoMVel_d * dt_MPC + SE.Rz * TargetP;
      // body_CoMAngle_d_mpc(1) = SE.ground_pitch;
      body_CoMPos_d_mpc(2) = _TargetP(2);
      mpc_state_d.block<robot_state_dim, 1>(robot_state_dim * i, 0) << body_CoMAngle_d_mpc,
          body_CoMPos_d_mpc, body_CoMAngleVel_d, body_CoMVel_d, Para.grav;
    }
    last_body_CoMAngle_d_mpc += SE.Rz.transpose() *body_CoMAngleVel_d * dt_MPC;
    last_body_CoMPos_d_mpc += body_CoMVel_d * dt_MPC;
    count = 0;
  }

  Matrix<double, 13, 1> xd = mpc_state_d.block<13, 1>(0, 0);

  count++;
}
void TrajectoryGenerator::setToeTrajectory(StateEstimator &SE,
                                           bool &ischangegait,
                                           bool &ischangephase,
                                           const Vector4i &LegPhase,
                                           const Vector3d &SwingHeight,
                                           double &time, double &swing_T)
{
  // 步态改变或者相序改变，才更新初始和目标足端信息
  if (ischangephase || init_flag||ischangegait)
  {
    initToePos = SE.ToePos_bH;
    // initToeVel = SE.ToeVel_bH;
    initToeVel.setZero();
    // SE.slope_angle_cal(SE.ToePos_wB);
    ischangephase = 0;
    ischangegait = 0;
    init_flag = 0;
  }
  TargetToePos = SwingOffset;
  for (int i = 0; i < 4; i++)
  {
    TargetToePos.row(i) += 0.5 * (TargetV).transpose() * swing_T +
                           kv * (SE.RotationMatrix.transpose() * SE.body_CoMVel - TargetV).transpose();
    // TargetToePos.row(i) += 0.5 * (TargetV).transpose() * swing_T;
    // TargetToePos(i, 2) = -body_CoMPos_d(2);
    TargetToePos(i, 2) = -TargetP(2);
  }
  // printinfo(TargetToePos);
  TargetToeVel.setZero();

  for (int i = 0; i < 4; i++)
  {
    ToeTraj(SE,initToePos, initToeVel, TargetToePos, TargetToeVel, i, LegPhase,
            SwingHeight, time, swing_T);
  }
}

void TrajectoryGenerator::compute_joint_target(StateEstimator &SE,
                                               Robot_Parameters &Para,
                                               const Vector4i &LegPhase,
                                               Mat43 &_ToePos_bH_d,
                                               Mat43 &_ToeVel_bH_d,
                                               Mat43 &_ToeAcc_bH_d)
{
  // body
  body_joint_angle_d = SE.IK(_ToePos_bH_d, Para);
  for (int i = 0; i < 4; i++)
  {
    if (LegPhase(i) == 0)
    {
      // 转置一下，为了减少代码长度
      body_joint_vel_d.row(i) =
          _ToeVel_bH_d.row(i) * (SE.Jacobian[i].inverse()).transpose();
      // 逆矩阵的导数:∂(A-1)/∂x=-(A-1)·(∂A/∂x)·(A-1)
      body_joint_acc_d.row(i) = (_ToeAcc_bH_d.row(i) -
                                 body_joint_vel_d.row(i) * (SE.dJacobian[i].transpose())) *
                                (SE.Jacobian[i].inverse()).transpose();
    }
    else
    {
      body_joint_vel_d.row(i).setZero();
      body_joint_acc_d.row(i).setZero();
    }
  }

  // arm
  // arm_joint_angularacc_d = (SE.J_object.block(0, 18, 3, 6)).completeOrthogonalDecomposition().pseudoInverse() * (ObjectAcc_d - SE.dJ_object.block(0, 18, 3, 6) * SE.arm_joint_vel);
}

void TrajectoryGenerator::ToeTraj(StateEstimator &SE,Mat43 &initToePos, Mat43 &initToeVel,
                                  Mat43 &TargetToePos_d, Mat43 &TargetToeVel_d,
                                  int &i, const Vector4i &LegPhase,
                                  const Vector3d &SwingHeight, double &time,
                                  double &swing_T)
{
  // if (LegPhase(i) == 1)
  // {
  //   // ToePos_bH_d.row(i) = initToePos.row(i) - TargetV.transpose() * dt; // 此处可能有问题
  //   ToePos_bH_d.row(i).head(2) = (SE.ToePos_bH.row(i) - TargetV.transpose() * dt).head(2); // 此处可能有问题
  //   ToePos_bH_d(i, 2) = SE.ToePos_bH(i, 2);
  //   // ToePos_bH_d.row(i) = SE.ToePos_bH.row(i) - TargetV.transpose() * dt; // 此处可能有问题
  //   // ToePos_bH_d.row(i) = initToePos.row(i); // 此处可能有问题
  //   // ToeVel_bH_d.row(i).setZero();
  //   ToeVel_bH_d.row(i) = zero;
  //   ToeAcc_bH_d.row(i).setZero();
  // }
  if (LegPhase(i) == 1)
  {
    ToePos_bH_d.row(i) = initToePos.row(i) - TargetV.transpose() * dt; // 此处可能有问题
    // ToePos_bH_d.row(i).head(2) = (SE.ToePos_bH.row(i) - TargetV.transpose() * dt).head(2); // 此处可能有问题
    // ToePos_bH_d(i, 2) = initToePos(i, 2);
    ToePos_bH_d(i, 2) = -TargetP(2);
    // ToePos_bH_d.row(i) = SE.ToePos_bH.row(i) - TargetV.transpose() * dt; // 此处可能有问题
    // ToePos_bH_d.row(i) = initToePos.row(i); // 此处可能有问题
    // ToeVel_bH_d.row(i).setZero();
    ToeVel_bH_d.row(i) = -TargetV.transpose();
    ToeAcc_bH_d.row(i).setZero();
  }

  else
  {
    // for (int j = 0; j < 3; j++) {
    //   TSpline_S_V_A(
    //       initToePos(i, j), initToeVel(i, j), 0.0, 0.0,
    //       0.5 * (initToePos(i, j) + TargetToePos_d(i, j)) + SwingHeight(j),
    //       0.5 * swing_T, TargetToePos_d(i, j), TargetToeVel_d(i, j), 0.0,
    //       swing_T, j, time, ToePos_bH_d(i, j), ToeVel_bH_d(i, j),
    //       ToeAcc_bH_d(i, j));
    // }

    QuinticPolynomial(initToePos(i, 0), initToeVel(i, 0), 0,
                      TargetToePos_d(i, 0), TargetToeVel_d(i, 0), 0, swing_T,
                      time, ToePos_bH_d(i, 0), ToeVel_bH_d(i, 0),
                      ToeAcc_bH_d(i, 0));
    QuinticPolynomial(initToePos(i, 1), initToeVel(i, 1), 0,
                      TargetToePos_d(i, 1), TargetToeVel_d(i, 1), 0, swing_T,
                      time, ToePos_bH_d(i, 1), ToeVel_bH_d(i, 1),
                      ToeAcc_bH_d(i, 1));
    if (time <= 0.5 * swing_T) // 前半摆动时间
    {
      QuinticPolynomial(initToePos(i, 2), initToeVel(i, 2), 0,
                        initToePos(i, 2) + SwingHeight(2), 0, 0,
                        0.5 * swing_T, time, ToePos_bH_d(i, 2),
                        ToeVel_bH_d(i, 2), ToeAcc_bH_d(i, 2));
    }

    if ((time > 0.5 * swing_T) && (time <= swing_T)) // 后半摆动时间
    {
      double tH = time - 0.5 * swing_T;
      QuinticPolynomial(initToePos(i, 2) + SwingHeight(2), 0, 0,
                        initToePos(i, 2), TargetToeVel_d(i, 2), 0,
                        0.5 * swing_T, tH, ToePos_bH_d(i, 2), ToeVel_bH_d(i, 2),
                        ToeAcc_bH_d(i, 2));
    }

    if (time > swing_T)
    // 超出规划的摆动时间时,传期望落足点数据
    {
      ToePos_bH_d.row(i) = TargetToePos_d.row(i);
      ToeVel_bH_d.row(i) = TargetToeVel_d.row(i);
      ToeAcc_bH_d.row(i).setZero();
    }
  }

}

TrajectoryGenerator::~TrajectoryGenerator() {}