/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     WholeBodyController.cpp
  @brief    WholeBodyController
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
#include "../include/WholeBodyController.h"
WholeBodyController::WholeBodyController(/* args */)
{
  ddq_d.setZero();
  LegForces.setZero();
  pinocchio::urdf::buildModel(urdf_filename, model); // 加载urdf模型
  data = pinocchio::Data(model);
  q.setZero(model.nq);  // np：Dimension of the configuration vector
                        // representation.配置向量表示的维度
  dq.setZero(model.nv); // nv：Dimension of the velocity vector
                        // space.速度向量空间的维度
  method = 0;

  CA.setZero();
  ca.setZero();
  CI.setZero();
  ci.setZero();
}

Task WholeBodyController::ConstructFloatingBaseEomTask(const matrix_t M_fb,
                                                       const matrix_t J_sfb,
                                                       const vector_t c_fb)
{
  matrix_t A(6, 18 + 12);
  A << M_fb, -J_sfb.transpose();
  vector_t b(6);
  b << -c_fb;
  return {A, b, matrix_t(), vector_t()};
}

Task WholeBodyController::ConstructJointTorqueTask(
    const Matrix<double, 12, 18> M_j, const Matrix<double, 12, 12> J_j,
    const Matrix<double, 12, 1> c_j)
{
  matrix_t D(12, 18 + 12);
  D << M_j, -J_j.transpose();
  vector_t b(12), F_max(12), F_min(12), max_torque(12), min_torque(12);
  for (int i = 0; i < 4; i++)
  {
    max_torque(3 * i + 0) = 50;
    max_torque(3 * i + 1) = 40;
    max_torque(3 * i + 2) = 120;
    min_torque(3 * i + 0) = -50;
    min_torque(3 * i + 1) = -35;
    min_torque(3 * i + 2) = -120;
  }
  F_max << max_torque - c_j;
  F_min << min_torque - c_j;

  matrix_t D_tot = Task::concatenateMatrices(D, -D);
  vector_t f_tot = Task::concatenateVectors(F_max, -F_min);
  return {matrix_t(), vector_t(), D_tot, f_tot};
}

Task WholeBodyController::ConstructFrictionConeTask(const Vector4i &_LegPhase)
{
  matrix_t A(3 * (4 - _LegPhase.sum()), 18 + 12);
  A.setZero();
  size_t j = 0;
  for (size_t i = 0; i < 4; ++i)
  {
    if (_LegPhase[i] == 0)
    {
      A.block(3 * j, 18 + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
      j++;
    }
  }
  vector_t b(A.rows());
  b.setZero();
  matrix_t frictionPyramic(6, 3); // clang-format off
  frictionPyramic << 0, 0, -1,
                     0, 0,  1,
                     1, 0, -u,
                    -1, 0, -u,
                     0, 1, -u,
                     0,-1, -u; // clang-format on
  matrix_t D(6 * _LegPhase.sum(), 18 + 12);
  vector_t f = VectorXd::Zero(D.rows());
  D.setZero();
  j = 0;
  for (size_t i = 0; i < 4; i++)
  {
    if (_LegPhase[i] == 1)
    {
      D.block(6 * j, 18 + 3 * i, 6, 3) = frictionPyramic;
      f.segment(6 * j, 2) << -f_min, f_max;
      j++;
    }
  };

  return {A, b, D, f};
}

Task WholeBodyController::ConstructNoContactMotionTask(
    const Vector4i &_LegPhase)
{
  matrix_t A(3 * _LegPhase.sum(), 18 + 12);
  matrix_t Js(3 * _LegPhase.sum(), 18);
  matrix_t dJs(3 * _LegPhase.sum(), 18);
  size_t j = 0;
  for (size_t i = 0; i < 4; ++i)
  {
    if (_LegPhase[i] == 1)
    {
      Js.block<3, 18>(3 * j, 0) = Jf[i].block<3, 18>(0, 0);
      dJs.block<3, 18>(3 * j, 0) = dJf[i].block<3, 18>(0, 0);
      j++;
    }
  }
  A << Js, matrix_t::Zero(3 * _LegPhase.sum(), 12);

  vector_t b(3 * _LegPhase.sum());
  b << -dJs * dq;

  return {A, b, matrix_t(), vector_t()};
}

Task WholeBodyController::ConstructBaseMotionTask()
{
  matrix_t A(18, 18 + 12);
  A << MatrixXd::Identity(18, 18), MatrixXd::Zero(18, 12);
  vector_t b(18);
  b << ddq_d;
  return {A, b, matrix_t(), vector_t()};
}

Task WholeBodyController::ConstructContactForceTask()
{
  matrix_t A(3 * 4, 18 + 12);
  vector_t b(A.rows());
  A << matrix_t::Zero(3 * 4, 18), MatrixXd::Identity(12, 12);
  b << f_ext;
  return {A, b, matrix_t(), vector_t()};
}

void WholeBodyController::WBC_solve(const Mat43 &body_joint_acc_d,
                                    const Matrix<double, 6, 1> &GenCoMAcc,
                                    Mat43 &LegForces, const Vector4i &LegPhase,
                                    const Mat43 &body_joint_angle,
                                    const Mat43 &body_joint_vel,
                                    const Vector3d &_CoMPos,
                                    const Vector3d &_CoMVel,
                                    const Vector3d &body_CoMAngle,
                                    const Vector3d &body_CoMAngularVel_w,
                                    const Matrix<double, 6, 1> arm_joint_angle,
                                    const Matrix<double, 6, 1> arm_joint_vel)
{
  compute_Related_Matrix(body_joint_acc_d, GenCoMAcc, LegForces, LegPhase,
                         body_joint_angle, body_joint_vel, _CoMPos, _CoMVel, body_CoMAngle, body_CoMAngularVel_w, arm_joint_angle, arm_joint_vel);

  //松弛优化
  if (method == 0)
  {
    defineConstraints(M_f, Jocabian_f, f_ext, C_f, arm_Gen_Forces, ddq_d, CA, ca);

    Quadprog_Solve(G, g0, CE.transpose(), ce, CI.transpose(), ci);

    updateDynamicInput(x_quad);
  }
  //分层控制
  else if (method == 1)
  {
    std::vector<Task> tasks = ConstructTasks(LegPhase);

    std::vector<std::shared_ptr<HoQp>> opt_problems = ConstructOptProblems(tasks);

    solution = opt_problems.back()->getSolutions();

    ddq_d = solution.head(18);

    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        LegForces(i, j) = solution[18 + 3 * i + j];
      }
    }
  }
}

std::vector<std::shared_ptr<HoQp>> WholeBodyController::ConstructOptProblems(
    std::vector<Task> &tasks)
{
  std::vector<std::shared_ptr<HoQp>> opt_problems(tasks.size());

  opt_problems[0] =
      std::shared_ptr<HoQp>(new HoQp(tasks[0], nullptr, wbc_input));
  for (int task_i = 1; task_i < tasks.size(); task_i++)
  {
    opt_problems[task_i] = std::shared_ptr<HoQp>(
        new HoQp(tasks[task_i], opt_problems[task_i - 1]));
  }

  return opt_problems;
}

std::vector<Task> WholeBodyController::ConstructTasks(
    const Vector4i &LegPhase)
{
  Task fb_eom_task = ConstructFloatingBaseEomTask(M_f_b, Jocabian_f, C_f);
  Task joint_torque_task = ConstructJointTorqueTask(
      M_f_j, Jocabian_j, C_j); // 有问题，多这个任务就废
  Task friction_cone_task = ConstructFrictionConeTask(LegPhase);

  Task joint_torque_and_friction_task =
      joint_torque_task + friction_cone_task; // 有问题

  Task no_contact_motion_task = ConstructNoContactMotionTask(LegPhase);
  Task Base_Motion_task = ConstructBaseMotionTask();
  Task force_track_task = ConstructContactForceTask();
  // std::vector<Task> tasks{fb_eom_task, force_track_task, friction_cone_task,
  //                         no_contact_motion_task, Base_Motion_task};
  std::vector<Task> tasks{fb_eom_task+force_track_task+joint_torque_task + friction_cone_task,
                           no_contact_motion_task,Base_Motion_task};
  return tasks;
}

void WholeBodyController::compute_Related_Matrix(
    const Mat43 &_joint_acc_d, const Matrix<double, 6, 1> &_GenCoMAcc,
    const Mat43 &_LegForces, const Vector4i &_LegPhase,
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

  wbc_input << ddq_d, f_ext;

  q.block<3, 1>(0, 0) = _CoMPos;  // p
  dq.block<3, 1>(0, 0) = _CoMVel; // v
  auto qq = _CoMAngle;
  // qq(2)=0;
  q.block<4, 1>(3, 0) = EulerToQuaternion(qq); // theta
  dq.block<3, 1>(3, 0) = _CoMAngularVel_w;     // omega

  for (int i = 0; i < 4; i++)
  {
    q.block<3, 1>(7 + 3 * i, 0) = (_joint_angle.row(i)).transpose();
    dq.block<3, 1>(6 + 3 * i, 0) = (_joint_vel.row(i)).transpose();
  }
  // q.block<6, 1>(19, 0) = arm_joint_angle;
  // dq.block<6, 1>(18, 0) = arm_joint_vel;

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
  // getFrameJacobian(model, data, model.getFrameId("gripperMover"),
  //                  LOCAL_WORLD_ALIGNED, Jf[4]);

  getFrameJacobianTimeVariation(model, data, model.getFrameId("FR_foot"),
                                LOCAL_WORLD_ALIGNED, dJf[0]);
  getFrameJacobianTimeVariation(model, data, model.getFrameId("RR_foot"),
                                LOCAL_WORLD_ALIGNED, dJf[1]);
  getFrameJacobianTimeVariation(model, data, model.getFrameId("FL_foot"),
                                LOCAL_WORLD_ALIGNED, dJf[2]);
  getFrameJacobianTimeVariation(model, data, model.getFrameId("RL_foot"),
                                LOCAL_WORLD_ALIGNED, dJf[3]);
  // getFrameJacobianTimeVariation(model, data, model.getFrameId("gripperMover"),
  //                               LOCAL_WORLD_ALIGNED, dJf[4]);

  for (int i = 0; i < 4; i++)
  {
    Jocabian_f.block<3, 6>(3 * i, 0) = Jf[i].block<3, 6>(0, 0);
    Jocabian_j.block<3, 12>(3 * i, 0) = Jf[i].block<3, 12>(0, 6);
  }
  // Jocabian_arm_t=Jf[5].block<3, 6>(3, 0);
  auto M = pinocchio::crba(model, data, q);
  // M.triangularView<Lower>() = M.triangularView<Upper>().transpose();
  M_f = M.block<6, 6>(0, 0);
  M_f_b = M.topRows(6);
  M_f_j = M.block(6, 0, 12, M.cols());

  auto C = pinocchio::nonLinearEffects(model, data, q, dq);
  C_f = C.block<6, 1>(0, 0);
  C_j = C.block<12, 1>(6, 0);

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

void WholeBodyController::defineConstraints(
    const Matrix<double, 6, 6> &_M_f, const Matrix<double, 12, 6> &_Jocabian_f,
    const Matrix<double, 12, 1> &_f_ext, const Matrix<double, 6, 1> &_C_f,
    const Matrix<double, 6, 1> &_arm_forces, const Matrix<double, 18, 1> &_ddq_d,
    const Matrix<double, 24, 12> &_CA, const Matrix<double, 24, 1> &_ca)
{
  // 等式约束
  CE << _M_f, -_Jocabian_f.transpose();
  ce = -_Jocabian_f.transpose() * _f_ext + _C_f +
       _M_f * _ddq_d.block<6, 1>(0, 0);

  // 不等式约束
  CI << MatrixXd::Zero(24, 6), -_CA;
  ci = _ca - _CA * _f_ext;
}

void WholeBodyController::Quadprog_Solve(const Matrix<double, 18, 18> _G_quad,
                                         const Matrix<double, 18, 1> _g0_quad,
                                         const Matrix<double, 18, 6> _CE_T,
                                         const Matrix<double, 6, 1> _ce,
                                         const Matrix<double, 18, 24> _CI_T,
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

void WholeBodyController::updateDynamicInput(
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
};

WholeBodyController::~WholeBodyController() {}