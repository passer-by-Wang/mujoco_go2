/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     WholeBodyController_arm.cpp
  @brief    WholeBodyController_arm
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
#include "../include/WholeBodyController_arm.h"
WholeBodyController_arm::WholeBodyController_arm(/* args */)
{
  ddq_d.setZero();
  LegForces.setZero();
  pinocchio::urdf::buildModel(urdf_filename, model); // 加载urdf模型
  data = pinocchio::Data(model);
  q.setZero(model.nq);  // np：Dimension of the configuration vector
                        // representation.配置向量表示的维度
  dq.setZero(model.nv); // nv：Dimension of the velocity vector
                        // space.速度向量空间的维度

  CA.setZero();
  ca.setZero();
  CI.setZero();
  ci.setZero();
}

Task WholeBodyController_arm::ConstructFloatingBaseEomTask(const matrix_t M_fb,
                                                           const matrix_t J_sfb,
                                                           const vector_t c_fb)
{
  matrix_t A(6, HO_variables);
  A << M_fb, -J_sfb.transpose();
  vector_t b(A.rows());
  b << -c_fb + Jocabian_arm_t.transpose() * Object_Gen_Forces.tail(3);
  return {A, b, matrix_t(), vector_t()};
}

Task WholeBodyController_arm::ConstructJointTorqueTask(
    const matrix_t M_j, const matrix_t J_j,
    const vector_t c_j)
{
  matrix_t D(12 + 6, HO_variables);
  D << M_j, -J_j.transpose();
  vector_t b(D.rows()), t_max(D.rows()), t_min(D.rows()), max_torque(D.rows()), min_torque(D.rows());
  for (int i = 0; i < 4; i++)
  {
    max_torque(3 * i + 0) = 40;
    max_torque(3 * i + 1) = 25;
    max_torque(3 * i + 2) = 90;
    min_torque(3 * i + 0) = -40;
    min_torque(3 * i + 1) = -25;
    min_torque(3 * i + 2) = -35;
  }
  for (int i = 0; i < 6; i++)
  {
    max_torque(12 + i) = 50;
    min_torque(12 + i) = -50;
  }
  t_max << max_torque - c_j - Jocabian_arm_j.transpose() * Object_Gen_Forces.tail(3);
  t_min << min_torque - c_j - Jocabian_arm_j.transpose() * Object_Gen_Forces.tail(3);

  matrix_t D_tot = Task::concatenateMatrices(D, -D);
  vector_t f_tot = Task::concatenateVectors(t_max, -t_min);
  return {matrix_t(), vector_t(), D_tot, f_tot};
}

Task WholeBodyController_arm::ConstructFrictionConeTask(const Vector4i &_LegPhase)
{
  matrix_t A(3 * (4 - _LegPhase.sum()), HO_variables);
  A.setZero();
  size_t j = 0;
  for (size_t i = 0; i < 4; ++i)
  {
    if (_LegPhase[i] == 0)
    {
      A.block(3 * j, 24 + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
      j++;
    }
  }
  vector_t b(A.rows());
  b.setZero();  // 此处的摆动腿力为0的约束与摆动腿约束和力跟踪约束冲突，暂时不使用
  matrix_t frictionPyramic(6, 3); // clang-format off
  frictionPyramic << 0, 0, -1,
                     0, 0,  1,
                     1, 0, -u,
                    -1, 0, -u,
                     0, 1, -u,
                     0,-1, -u; // clang-format on
  matrix_t D(6 * _LegPhase.sum(), HO_variables);
  vector_t f = VectorXd::Zero(D.rows());
  D.setZero();
  j = 0;
  for (size_t i = 0; i < 4; i++)
  {
    if (_LegPhase[i] == 1)
    {
      D.block(6 * j, 24 + 3 * i, 6, 3) = frictionPyramic;
      f.segment(6 * j, 2) << -f_min, f_max;
      j++;
    }
  };

  return {matrix_t{}, vector_t{}, D, f};
}

Task WholeBodyController_arm::ConstructNoContactMotionTask(
    const Vector4i &_LegPhase)
{
  matrix_t A(3 * _LegPhase.sum(), HO_variables);
  matrix_t Js(3 * _LegPhase.sum(), 24);
  matrix_t dJs(3 * _LegPhase.sum(), 24);
  size_t j = 0;
  for (size_t i = 0; i < 4; ++i)
  {
    if (_LegPhase[i] == 1)
    {
      Js.block<3, 24>(3 * j, 0) = Jf[i].block<3, 24>(0, 0);
      dJs.block<3, 24>(3 * j, 0) = dJf[i].block<3, 24>(0, 0);
      j++;
    }
  }
  A << Js, matrix_t::Zero(3 * _LegPhase.sum(), 15);

  vector_t b(3 * _LegPhase.sum());
  b << -dJs * dq;

  return {A, b, matrix_t(), vector_t()};
}

Task WholeBodyController_arm::ConstructBaseMotionTask()
{
  matrix_t A(6, HO_variables);
  A << MatrixXd::Identity(A.rows(), A.rows()), MatrixXd::Zero(A.rows(), HO_variables - A.rows());
  vector_t b(A.rows());
  b << ddq_d.head(A.rows());
  return {A, b, matrix_t(), vector_t()};
}

Task WholeBodyController_arm::ConstructSwingLegTask(const Vector4i &_LegPhase, const Mat43 &toeAcc)
{
  matrix_t A(3 * (4 - _LegPhase.sum()), HO_variables);
  matrix_t Js(3 * (4 - _LegPhase.sum()), 24);
  matrix_t dJs(3 * (4 - _LegPhase.sum()), 24);
  vector_t swtoe_acc(3 * (4 - _LegPhase.sum()));
  size_t j = 0;
  for (size_t i = 0; i < 4; ++i)
  {
    if (_LegPhase[i] == 0)
    {
      Js.block<3, 24>(3 * j, 0) = Jf[i].block<3, 24>(0, 0);
      dJs.block<3, 24>(3 * j, 0) = dJf[i].block<3, 24>(0, 0);
      // swtoe_acc.segment(3 * j, 3) = -LegForces.row(i);
      swtoe_acc.segment(3 * j, 3) = toeAcc.row(i);
      j++;
    }
  }
  A << Js, matrix_t::Zero(3 * (4 - _LegPhase.sum()), HO_variables - 24);
  vector_t b(A.rows());
  b << swtoe_acc - dJs * dq;
  return {A, b, matrix_t(), vector_t()};
}

Task WholeBodyController_arm::ConstructContactForceTask(const Vector4i &_LegPhase)
{
  //min Forces
  // matrix_t A(3 * 4+3, HO_variables);
  // vector_t b(A.rows());
  // A << matrix_t::Zero(3 * 4+3, HO_variables-15), MatrixXd::Identity(15, 15);
  // b.setZero();

  //track Forces
  matrix_t A(3 * 4+3, HO_variables);
  vector_t b(A.rows());
  A << matrix_t::Zero(3 * 4+3, HO_variables-15), MatrixXd::Identity(15, 15);
  b=f_ext;


  //track Forces
  // matrix_t A(3 * 4 + 3, HO_variables);
  // vector_t b(A.rows());
  // A.setZero();
  // b.setZero();
  // for (size_t i = 0; i < 4; ++i)
  // {
  //   if (_LegPhase[i] == 1)
  //   {
  //     A.block<3, 3>(3 * i, 24 + 3 * i) = matrix_t::Identity(3, 3);
  //     b.segment(3 * i, 3) = f_ext.segment(3 * i, 3);
  //   }
  // }
  // A.block<3, 3>(3 * 4, 24 + 3 * 4) = matrix_t::Identity(3, 3);
  // b.segment(3 * 4, 3) = f_ext.segment(3 * 4, 3);

  return {A, b, matrix_t(), vector_t()};
}

void WholeBodyController_arm::WBC_solve(const Mat43 &body_joint_acc_d,
                                        const Matrix<double, 6, 1> &GenCoMAcc,
                                        Mat43 &LegForces, const Matrix<double, 6, 1> object_Gen_Forces, const Vector4i &LegPhase,
                                        const Mat43 &body_joint_angle,
                                        const Mat43 &body_joint_vel,
                                        const Vector3d &_CoMPos,
                                        const Vector3d &_CoMVel,
                                        const Vector3d &body_CoMAngle,
                                        const Vector3d &body_CoMAngularVel_w,
                                        const Matrix<double, 6, 1> arm_joint_angle,
                                        const Matrix<double, 6, 1> arm_joint_vel,
                                        const Mat43 &toeAcc)
{
  compute_Related_Matrix(body_joint_acc_d, GenCoMAcc, LegForces, object_Gen_Forces, LegPhase,
                         body_joint_angle, body_joint_vel, _CoMPos, _CoMVel, body_CoMAngle, body_CoMAngularVel_w, arm_joint_angle, arm_joint_vel);

  defineConstraints(M_f, Jocabian_f, f_ext, C_f, object_Gen_Forces, ddq_d, CA, ca);

  Quadprog_Solve(G, g0, CE.transpose(), ce, CI.transpose(), ci);

  updateDynamicInput(x_quad);

  // 分层控制

  // struct timeval t1, t2;
  // gettimeofday(&t1, NULL);

  // std::vector<Task> tasks = ConstructTasks(LegPhase, toeAcc);

  // std::vector<std::shared_ptr<HoQp>> opt_problems = ConstructOptProblems(tasks);

  // solution = opt_problems.back()->getSolutions();

  // ddq_d = solution.head(24);

  // for (int i = 0; i < 4; i++)
  // {
  //   for (int j = 0; j < 3; j++)
  //   {
  //     LegForces(i, j) = solution[24 + 3 * i + j];
  //   }
  // }

  // Object_Gen_Forces.head(3) = solution.tail(3);

  // gettimeofday(&t2, NULL);
  // cout << "[HWBC solving time]: "
  //      << " ";
  // cout << double(t2.tv_usec - t1.tv_usec) / 1000 << " " << endl;

  // std::shared_ptr<HoQp> opt_problems0 =
  //     std::shared_ptr<HoQp>(new HoQp(tasks[0], nullptr));
  // std::shared_ptr<HoQp> opt_problems1 =
  //     std::shared_ptr<HoQp>(new HoQp(tasks[1], opt_problems0));
  // std::shared_ptr<HoQp> opt_problems2 =
  //     std::shared_ptr<HoQp>(new HoQp(tasks[2], opt_problems1));
  // std::shared_ptr<HoQp> opt_problems3 =
  //     std::shared_ptr<HoQp>(new HoQp(tasks[3], opt_problems2));
  // std::shared_ptr<HoQp> opt_problems4 =
  //     std::shared_ptr<HoQp>(new HoQp(tasks[4], opt_problems3));

  // printinfo(opt_problems3->getSolutions().transpose());
  // printinfo(opt_problems4->getSolutions().transpose());

  // HoQp hoQp0(task0, nullptr, wbc_input);
  // HoQp hoQp1(task1, std::make_shared<HoQp>(task0, nullptr, wbc_input));
  // printinfo(hoQp0.decisionVarsSolutions_);
  // printinfo(hoQp1.decisionVarsSolutions_);
}

std::vector<std::shared_ptr<HoQp>> WholeBodyController_arm::ConstructOptProblems(
    std::vector<Task> &tasks)
{
  std::vector<std::shared_ptr<HoQp>> opt_problems(tasks.size());

  opt_problems[0] =
      std::shared_ptr<HoQp>(new HoQp(tasks[0], nullptr));
  for (int task_i = 1; task_i < tasks.size(); task_i++)
  {
    opt_problems[task_i] = std::shared_ptr<HoQp>(
        new HoQp(tasks[task_i], opt_problems[task_i - 1]));
  }

  return opt_problems;
}

std::vector<Task> WholeBodyController_arm::ConstructTasks(
    const Vector4i &LegPhase, const Mat43 &toeAcc)
{
  Task fb_eom_task = ConstructFloatingBaseEomTask(M_f_b, Jocabian_f, C_f);
  Task joint_torque_task = ConstructJointTorqueTask(
      M_f_j, Jocabian_j, C_j); // 有问题，多这个任务就废
  Task friction_cone_task = ConstructFrictionConeTask(LegPhase);

  // Task joint_torque_and_friction_task =
  //     joint_torque_task + friction_cone_task;  // 有问题

  Task no_contact_motion_task = ConstructNoContactMotionTask(LegPhase);
  Task Base_Motion_task = ConstructBaseMotionTask();
  Task SwingLegTask = ConstructSwingLegTask(LegPhase, toeAcc);
  Task force_track_task = ConstructContactForceTask(LegPhase);
  std::vector<Task> tasks{fb_eom_task+force_track_task+joint_torque_task + friction_cone_task,
                           no_contact_motion_task,Base_Motion_task+SwingLegTask};

  // std::vector<Task> tasks{fb_eom_task,force_track_task,joint_torque_task+friction_cone_task,no_contact_motion_task,Base_Motion_task};
  return tasks;
}

void WholeBodyController_arm::compute_Related_Matrix(
    const Mat43 &_joint_acc_d, const Matrix<double, 6, 1> &_GenCoMAcc,
    const Mat43 &_LegForces, const Matrix<double, 6, 1> _object_Gen_Forces, const Vector4i &_LegPhase,
    const Mat43 &_joint_angle, const Mat43 &_joint_vel,
    const Vector3d &_CoMPos, const Vector3d &_CoMVel,
    const Vector3d &_CoMAngle, const Vector3d &_CoMAngularVel_w,
    const Matrix<double, 6, 1> arm_joint_angle, const Matrix<double, 6, 1> arm_joint_vel)
{
  ddq_d.block<6, 1>(0, 0) = _GenCoMAcc;
  for (int i = 0; i < 4; i++)
  {
    ddq_d.block<3, 1>(6 + 3 * i, 0) = _joint_acc_d.row(i).transpose();
  }

  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      f_ext(3 * i + j, 0) = _LegForces(i, j);
    }
  }

  f_ext.tail(3) = _object_Gen_Forces.head(3);
  wbc_input << ddq_d, f_ext;

  q.block<3, 1>(0, 0) = _CoMPos;  // p
  dq.block<3, 1>(0, 0) = _CoMVel; // v
  auto qq = _CoMAngle;
  q.block<4, 1>(3, 0) = EulerToQuaternion(qq); // theta
  dq.block<3, 1>(3, 0) = _CoMAngularVel_w;     // omega

  for (int i = 0; i < 4; i++)
  {
    q.block<3, 1>(7 + 3 * i, 0) = (_joint_angle.row(i)).transpose();
    dq.block<3, 1>(6 + 3 * i, 0) = (_joint_vel.row(i)).transpose();
  }
  q.block<6, 1>(19, 0) = arm_joint_angle;
  dq.block<6, 1>(18, 0) = arm_joint_vel;

  framesForwardKinematics(model, data, q);

  computeJointJacobians(model, data, q);
  computeJointJacobiansTimeVariation(model, data, q, dq);

  Jf->setZero();
  dJf->setZero();
  getFrameJacobian(model, data, model.getFrameId("FR_foot"),
                   LOCAL_WORLD_ALIGNED, Jf[0]);
  getFrameJacobian(model, data, model.getFrameId("RR_foot"),
                   LOCAL_WORLD_ALIGNED, Jf[1]);
  getFrameJacobian(model, data, model.getFrameId("FL_foot"),
                   LOCAL_WORLD_ALIGNED, Jf[2]);
  getFrameJacobian(model, data, model.getFrameId("RL_foot"),
                   LOCAL_WORLD_ALIGNED, Jf[3]);
  getFrameJacobian(model, data, model.getFrameId("gripperMover"),
                   LOCAL_WORLD_ALIGNED, Jf[4]);

  getFrameJacobianTimeVariation(model, data, model.getFrameId("FR_foot"),
                                LOCAL_WORLD_ALIGNED, dJf[0]);
  getFrameJacobianTimeVariation(model, data, model.getFrameId("RR_foot"),
                                LOCAL_WORLD_ALIGNED, dJf[1]);
  getFrameJacobianTimeVariation(model, data, model.getFrameId("FL_foot"),
                                LOCAL_WORLD_ALIGNED, dJf[2]);
  getFrameJacobianTimeVariation(model, data, model.getFrameId("RL_foot"),
                                LOCAL_WORLD_ALIGNED, dJf[3]);
  getFrameJacobianTimeVariation(model, data, model.getFrameId("gripperMover"),
                                LOCAL_WORLD_ALIGNED, dJf[4]);

  for (int i = 0; i < 4; i++)
  {
    Jocabian_f.block<3, 6>(3 * i, 0) = Jf[i].block<3, 6>(0, 0);
    Jocabian_j.block<3, 18>(3 * i, 0) = Jf[i].block<3, 18>(0, 6);
  }
  Jocabian_f.block<3, 6>(12, 0) = Jf[4].block<3, 6>(0, 0);
  Jocabian_j.block<3, 18>(12, 0) = Jf[4].block<3, 18>(0, 6);
  Jocabian_arm_t = Jf[4].block<3, 6>(3, 0);
  Jocabian_arm_j = Jf[4].block<3, 18>(3, 6);
  auto M = pinocchio::crba(model, data, q);
  // M.triangularView<Lower>() = M.triangularView<Upper>().transpose();
  M_f = M.block<6, 6>(0, 0);
  M_f_b = M.topRows(6);
  M_f_j = M.block(6, 0, 12 + 6, 24);

  auto C = pinocchio::nonLinearEffects(model, data, q, dq);
  C_f = C.block<6, 1>(0, 0);
  C_j = C.block<18, 1>(6, 0);

  CI_sub << 0, 0, 1, 0, 0, -1, 1, 0, -0.5 * sqrt(2) * u, -1, 0,
      -0.5 * sqrt(2) * u, 0, 1, -0.5 * sqrt(2) * u, 0, -1, -0.5 * sqrt(2) * u;
  ci_sub << f_max, -f_min, 0, 0, 0, 0;

  CA.setZero();
  ca.setZero();
  for (int i = 0; i < 4; i++)
  {
    if (_LegPhase[i] == 1)
    {
      CA.block<6, 3>(6 * i, 3 * i) = CI_sub;
      ca.block<6, 1>(6 * i, 0) = ci_sub;
    }
  }
  G.setIdentity();
  G.block<6, 6>(0, 0) = ratio * (G.block<6, 6>(0, 0)).setIdentity();
  g0.setZero();
};

void WholeBodyController_arm::defineConstraints(
    const Matrix<double, 6, 6> &_M_f, const Matrix<double, 15, 6> &_Jocabian_f,
    const Matrix<double, 15, 1> &_f_ext, const Matrix<double, 6, 1> &_C_f,
    const Matrix<double, 6, 1> &_arm_forces, const Matrix<double, 18, 1> &_ddq_d,
    const Matrix<double, 24, 15> &_CA, const Matrix<double, 24, 1> &_ca)
{
  // 等式约束
  CE << _M_f, -_Jocabian_f.transpose();
  ce = -_Jocabian_f.transpose() * _f_ext + _C_f +
       _M_f * _ddq_d.block<6, 1>(0, 0) - Jocabian_arm_t.transpose() * _arm_forces.tail(3);

  // 不等式约束
  CI << MatrixXd::Zero(24, 6), -_CA;
  ci = _ca - _CA * _f_ext;
}

void WholeBodyController_arm::Quadprog_Solve(const Matrix<double, 21, 21> _G_quad,
                                             const Matrix<double, 21, 1> _g0_quad,
                                             const Matrix<double, 21, 6> _CE_T,
                                             const Matrix<double, 6, 1> _ce,
                                             const Matrix<double, 21, 24> _CI_T,
                                             const Matrix<double, 24, 1> _ci)
{
  G_quad.resize(_G_quad.rows(), _G_quad.cols());
  for (int i = 0; i < _G_quad.rows(); i++)
  {
    for (int j = 0; j < _G_quad.cols(); j++)
    {
      G_quad[i][j] = _G_quad(i, j);
    }
  }

  g0_quad.resize(_g0_quad.rows());
  for (int i = 0; i < _g0_quad.rows(); i++)
  {
    g0_quad[i] = _g0_quad[i];
  }

  CE_quad.resize(_CE_T.rows(), _CE_T.cols());
  for (int i = 0; i < _CE_T.rows(); i++)
  {
    for (int j = 0; j < _CE_T.cols(); j++)
    {
      CE_quad[i][j] = _CE_T(i, j);
    }
  }

  ce_quad.resize(_ce.rows());
  for (int i = 0; i < _ce.rows(); i++)
  {
    ce_quad[i] = _ce[i];
  }

  CI_quad.resize(_CI_T.rows(), _CI_T.cols());
  for (int i = 0; i < _CI_T.rows(); i++)
  {
    for (int j = 0; j < _CI_T.cols(); j++)
    {
      CI_quad[i][j] = _CI_T(i, j);
    }
  }

  ci_quad.resize(_ci.rows());
  for (int i = 0; i < _ci.rows(); i++)
  {
    ci_quad[i] = _ci[i];
  }

  x_quad.resize(_G_quad.rows());
  quadprogpp::solve_quadprog(G_quad, g0_quad, CE_quad, ce_quad, CI_quad,
                             ci_quad, x_quad);
}

void WholeBodyController_arm::updateDynamicInput(
    const quadprogpp::Vector<double> _x_quad)
{
  for (int i = 0; i < 6; i++)
  {
    ddq_d(i) += _x_quad[i];
  }
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      LegForces(i, j) += _x_quad[6 + 3 * i + j];
    }
  }
  for (int i = 0; i < 3; i++)
  {
    Object_Gen_Forces[i] += _x_quad[18 + i];
  }
};

WholeBodyController_arm::~WholeBodyController_arm() {}