/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     ClassManager.cpp
  @brief    Manager
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
#include "../include/ClassManager.h"
ClassManager::ClassManager()
{
  go2_para_config = YAML::LoadFile(
      "/home/wh/simulation/Legged_robot/config/"
      "go2_para.yaml");
  count = 0;
  time = 0;
}

void ClassManager::run(MujocoJoint *bodyjoint[4][3], MujocoImu *imu,
                       MujocoForceSensor *ForceSensor[4])
{

  time += double(timestep) / 1000;

  go2_Mjc.ReadDatafromRobot(bodyjoint, imu, ForceSensor, go2_SE);

  // go2_cmd.GetUserInput(go2_Trj, go2_FSM);

  go2_FSM.setLegPhase(go2_FSM.GaitTag, go2_SE);

  go2_SE.state_calc(go2_FSM.LegPhase, go2_Para);

  go2_Trj.setTrajectory(go2_SE, go2_Para, go2_FSM.ischangegait,
                         go2_FSM.ischangephase, go2_FSM.LegPhase,
                         go2_FSM.current_gait.SwingHeight, go2_FSM.time,
                         go2_FSM.current_gait.swing_T, count);

  go2_OC.computeObjectForces(go2_SE.nmpc_DRBM_state, go2_Trj.nmpc_DRBM_state_d, go2_SE.object_CoMPos_wrtB);

  go2_vmc.computeContactForces(go2_Para, go2_FSM.LegPhase,
                                go2_SE.ToePos_wB, go2_Trj.Gen_vf_d);

  go2_mpc.computeContactForces(go2_Para, go2_SE.Rz, go2_FSM.LegPhase,
                                go2_SE.ToePos_wB, go2_SE.Inertia_w,
                                go2_SE.mpc_state, go2_Trj.mpc_state_d,
                                go2_cmd.isStop);

  go2_nmpc.computeContactForces(go2_SE.nmpc_state, go2_Trj.nmpc_state_d,
                                 go2_SE.ToePos_wB, go2_FSM.LegPhase, go2_SE.Arm2BodyGForces + go2_OC.Object_to_Body_Forces,
                                 go2_cmd.isStop);

  go2_nmpc_rpy.computeContactForces(go2_SE.nmpc_state, go2_Trj.nmpc_state_d, go2_SE.ToePos_wB, go2_FSM.LegPhase, go2_SE.Arm2BodyGForces + go2_OC.Object_to_Body_Forces, go2_cmd.isStop);

  go2_nmpc_DRBM.computeContactForces(go2_SE.nmpc_DRBM_state, go2_Trj.nmpc_DRBM_state_d,
                                      go2_SE.ToePos_wB, go2_SE.object_CoMPos_wrtB, go2_FSM.LegPhase, go2_SE.Arm2BodyGForces + go2_OC.Object_to_Body_Forces, go2_cmd.isStop);

  go2_nmpc_DRBM_rpy.computeContactForces(go2_SE.nmpc_DRBM_state, go2_Trj.nmpc_DRBM_state_d,
                                          go2_SE.ToePos_wB, go2_SE.object_CoMPos_wrtB, go2_FSM.LegPhase, go2_SE.Arm2BodyGForces, go2_cmd.isStop);

  go2_SLC.computeSwingLegForces(go2_Trj.ToePos_bH_d, go2_SE.ToePos_bH,
                                 go2_Trj.ToeVel_bH_d, go2_SE.ToeVel_bH);

  // controllerSwitch2Use(Robot_controller_switch, go2_SE.RotationMatrix,
  //                      go2_FSM.LegPhase, go2_vmc.ContactLegForces,
  //                      go2_SLC.SwingLegForce, go2_mpc.LegForces,
  //                      go2_nmpc.LegForces, go2_nmpc_rpy.LegForces, go2_nmpc_DRBM.LegForces, go2_nmpc_DRBM_rpy.LegForces, go2_wbc_arm.LegForces);

  controllerSwitch2Use(Robot_controller_switch, go2_SE.RotationMatrix,
                       go2_FSM.LegPhase, go2_vmc.ContactLegForces,
                       go2_SLC.SwingLegForce, go2_mpc.LegForces,
                       go2_nmpc.LegForces, go2_nmpc_rpy.LegForces, go2_nmpc_DRBM.LegForces, go2_nmpc_DRBM_rpy.LegForces, go2_wbc.LegForces);

  go2_wbc.WBC_solve(go2_Trj.body_joint_acc_d, go2_Trj.GenCoMAcc,
                     go2_wbc.LegForces, go2_FSM.LegPhase,
                     go2_SE.body_joint_angle, go2_SE.body_joint_vel, go2_SE.body_CoMPos,go2_SE.body_CoMVel,go2_SE.body_CoMAngle,
                     go2_SE.body_CoMAngularVel_w,go2_SE.arm_joint_angle,go2_SE.arm_joint_vel);

  // go2_wbc_arm.WBC_solve(go2_Trj.body_joint_acc_d, go2_Trj.GenCoMAcc, go2_wbc_arm.LegForces, go2_wbc_arm.Object_Gen_Forces,
  //                        go2_FSM.LegPhase, go2_SE.body_joint_angle, go2_SE.body_joint_vel, go2_SE.body_CoMPos, go2_SE.body_CoMVel, go2_SE.body_CoMAngle,
  //                        go2_SE.body_CoMAngularVel_w, go2_SE.arm_joint_angle, go2_SE.arm_joint_vel, go2_Trj.ToeAcc_bH_d * go2_SE.RotationMatrix.transpose());

  // go2_dynamic.computeJointTorque(go2_SE,
  //     go2_wbc.ddq_d, go2_wbc.LegForces,go2_OC.Object_Forces, go2_SE.body_joint_angle,
  //     go2_SE.body_joint_vel, go2_Trj.body_joint_angle_d, go2_Trj.body_joint_vel_d,
  //     go2_SE.body_CoMPos,go2_SE.body_CoMVel,go2_SE.body_CoMAngle, go2_SE.body_CoMAngularVel_w);

  // go2_dynamic.computeJointTorque(go2_SE,
  //                                 go2_wbc_arm.ddq_d, go2_wbc_arm.LegForces, go2_wbc_arm.Object_Gen_Forces, go2_SE.body_joint_angle,
  //                                 go2_SE.body_joint_vel, go2_Trj.body_joint_angle_d, go2_Trj.body_joint_vel_d,
  //                                 go2_SE.body_CoMPos, go2_SE.body_CoMVel, go2_SE.body_CoMAngle, go2_SE.body_CoMAngularVel_w);

  go2_dynamic.computeJointTorque(go2_SE,
                                  go2_wbc.ddq_d, go2_wbc.LegForces, go2_wbc_arm.Object_Gen_Forces, go2_SE.body_joint_angle,
                                  go2_SE.body_joint_vel, go2_Trj.body_joint_angle_d, go2_Trj.body_joint_vel_d,
                                  go2_SE.body_CoMPos, go2_SE.body_CoMVel, go2_SE.body_CoMAngle, go2_SE.body_CoMAngularVel_w);

  // go2_dynamic_arm.computeJointTorque(go2_SE, go2_wbc_arm.ddq_d, go2_Trj.arm_joint_angularacc_d, go2_wbc_arm.LegForces,
  //                                     go2_wbc_arm.Object_Gen_Forces, go2_SE.body_joint_angle,
  //                                     go2_SE.body_joint_vel, go2_Trj.body_joint_angle_d, go2_Trj.body_joint_vel_d,
  //                                     go2_SE.body_CoMPos, go2_SE.body_CoMVel, go2_SE.body_CoMAngle, go2_SE.body_CoMAngularVel_w,
  //                                     go2_SE.arm_joint_angle, go2_SE.arm_joint_vel, go2_Trj.temp_arm_joint, go2_Trj.temp_arm_joint_vel);

  // Execute_Instructions(go2_cmd.isStop, Robot_controller_switch,
  //                      go2_nmpc.is_start||go2_nmpc_DRBM.is_start||go2_nmpc_rpy.is_start||go2_nmpc_DRBM_rpy.is_start,
  //                      go2_dynamic.BodyJointTorque,go2_dynamic.ArmJointTorque,
  //                      go2_SE.body_joint_angle,go2_SE.arm_joint_angle);

  Execute_Instructions(bodyjoint, go2_cmd.isStop, Robot_controller_switch,
                       go2_nmpc.is_start || go2_nmpc_DRBM.is_start || go2_nmpc_rpy.is_start || go2_nmpc_DRBM_rpy.is_start,
                       go2_dynamic.BodyJointTorque,
                       go2_SE.init_body_joint_angle);

  // go2_eef.EEF_from_dynamic(go2_dynamic.tor,go2_dynamic.Jf,go2_SE);

  go2_Info.PrintOutInfo(go2_vmc, go2_dynamic, go2_FSM, go2_Para, go2_SE,
                         go2_SLC, go2_Trj, go2_cmd, go2_wbc,
                         go2_test);

  go2_Log.SaveData(go2_FSM, go2_SE, go2_Trj, go2_SLC,
                    go2_mpc, go2_nmpc, go2_vmc, go2_wbc, go2_dynamic,go2_eef);
}

void ClassManager::Execute_Instructions(MujocoJoint *bodyjoint[4][3],
                                        const bool &isStop,
                                        const int &Robot_controller_switch,
                                        const bool &isStart,
                                        const Mat43 &BodyJointTorque,
                                        const Mat43 &BodyinitPos)
{
  if ((Robot_controller_switch == 3 || Robot_controller_switch == 4 || Robot_controller_switch == 5 ||
       Robot_controller_switch == 6) &&
      isStart == false)
  {
    go2_Mjc.SendBodyJointAngletoRobot(bodyjoint, BodyinitPos);
  }
  else
  {
    go2_Mjc.SendBodyJointTorquetoRobot(bodyjoint, BodyJointTorque);
  }
}

void ClassManager::controllerSwitch2Use(
    const int &_Robot_controller_switch, const Matrix3d &_RotationMatrix,
    const Vector4i &LegPhase, const Mat43 &_vmc_LegForces,
    const Mat43 &_SLC_LegForces, const Mat43 &_MPC_LegForces,
    const Mat43 &_NMPC_LegForces, const Mat43 &_NMPC_RPY_LegForces, const Mat43 &_NMPC_D_LegForces,
    const Mat43 &_NMPC_D_RPY_LegForces, Mat43 &_WBC_LegForces)
{
  // LegForces:外部对机器人的作用力
  for (int i = 0; i < 4; i++)
  {
    if (LegPhase(i) == 0)
    {
      _WBC_LegForces.row(i) = -_SLC_LegForces.row(i) * (_RotationMatrix.transpose());
      // _WBC_LegForces.row(i).setZero();
    }
    else
    {
      switch (_Robot_controller_switch)
      {
      case 1:
        _WBC_LegForces.row(i) = _vmc_LegForces.row(i);
        break;
      case 2:
        _WBC_LegForces.row(i) = _MPC_LegForces.row(i);
        break;
      case 3:
        _WBC_LegForces.row(i) = _NMPC_LegForces.row(i);
        break;
      case 4:
        _WBC_LegForces.row(i) = _NMPC_D_LegForces.row(i);
        break;
      case 5:
        _WBC_LegForces.row(i) = _NMPC_RPY_LegForces.row(i);
        break;
      case 6:
        _WBC_LegForces.row(i) = _NMPC_D_RPY_LegForces.row(i);
        break;
      default:
        break;
      }
    }
  }
  // printinfo(_vmc_LegForces);
  // printinfo(_MPC_LegForces);
  // go2_wbc_arm.Object_Gen_Forces = go2_OC.Object_Forces;
  // go2_wbc_arm.Object_Gen_Forces=go2_nmpc_DRBM_rpy.object_Gen_Forces;
  // printinfocolor(_WBC_LegForces,GREEN);
}

void ClassManager::Configure_para()
{
  Robot_controller_switch =
      go2_para_config["Robot_controller_switch"].as<int>();
  // Robot_Parameters
  go2_Para.BQR3_mass =
      go2_para_config["physical_paras"]["Robot_mass"].as<double>();
  go2_Para.BodyMass =
      go2_para_config["physical_paras"]["BodyMass"].as<double>();
  go2_Para.BodyLength =
      go2_para_config["physical_paras"]["BodyLength"].as<double>();
  go2_Para.BodyWidth =
      go2_para_config["physical_paras"]["BodyWidth"].as<double>();
  go2_Para.abd_offset =
      go2_para_config["physical_paras"]["Abd_offset"].as<double>();
  go2_Para.Thigh_Length =
      go2_para_config["physical_paras"]["Thigh_Length"].as<double>();
  go2_Para.Calf_Length =
      go2_para_config["physical_paras"]["Calf_Length"].as<double>();
  go2_Para.friction_coeff =
      go2_para_config["physical_paras"]["friction_coeff"].as<double>();
  go2_Para.grav = go2_para_config["physical_paras"]["grav"].as<double>();
  for (int i = 0; i < 3; i++)
  {
    go2_Para.Inertia(i, i) =
        go2_para_config["physical_paras"]["Inertia_coeff"][i].as<double>();
  }

  // StateEstimator
  for (int i = 0; i < 9; i++)
  {
    go2_SE.Q(i, i) = go2_para_config["StateEstimator"]["Q"][i].as<double>();
    go2_SE.R(i, i) = go2_para_config["StateEstimator"]["R"][i].as<double>();
  }
  for (int i = 0; i < 3; i++)
  {
    go2_SE.body_CoM_offset[i] =
        go2_para_config["StateEstimator"]["CoMoffset"][i].as<double>();
  }
  go2_SE.LegForce_estimate_method=go2_para_config["StateEstimator"]["LegForce_estimate_method"].as<int>();

  // TrajectoryGenerator
  for (int i = 0; i < 3; i++)
  {
    go2_Trj.k_p(i, i) =
        go2_para_config["TrajectoryGenerator"]["k_p"][i].as<double>();
    go2_Trj.k_v(i, i) =
        go2_para_config["TrajectoryGenerator"]["k_v"][i].as<double>();
    go2_Trj.k_theta(i, i) =
        go2_para_config["TrajectoryGenerator"]["k_theta"][i].as<double>();
    go2_Trj.k_w(i, i) =
        go2_para_config["TrajectoryGenerator"]["k_w"][i].as<double>();
  }
  go2_Trj.SwingOffset(0, 0) =
      go2_para_config["TrajectoryGenerator"]["SwingLegOffset"][0].as<double>();
  go2_Trj.SwingOffset(0, 1) =
      -go2_para_config["TrajectoryGenerator"]["SwingLegOffset"][1].as<double>();
  go2_Trj.SwingOffset(0, 2) =
      go2_para_config["TrajectoryGenerator"]["SwingLegOffset"][2].as<double>();
  go2_Trj.SwingOffset(1, 0) =
      -go2_para_config["TrajectoryGenerator"]["SwingLegOffset"][0].as<double>();
  go2_Trj.SwingOffset(1, 1) =
      -go2_para_config["TrajectoryGenerator"]["SwingLegOffset"][1].as<double>();
  go2_Trj.SwingOffset(1, 2) =
      go2_para_config["TrajectoryGenerator"]["SwingLegOffset"][2].as<double>();
  go2_Trj.SwingOffset(2, 0) =
      go2_para_config["TrajectoryGenerator"]["SwingLegOffset"][0].as<double>();
  go2_Trj.SwingOffset(2, 1) =
      go2_para_config["TrajectoryGenerator"]["SwingLegOffset"][1].as<double>();
  go2_Trj.SwingOffset(2, 2) =
      go2_para_config["TrajectoryGenerator"]["SwingLegOffset"][2].as<double>();
  go2_Trj.SwingOffset(3, 0) =
      -go2_para_config["TrajectoryGenerator"]["SwingLegOffset"][0].as<double>();
  go2_Trj.SwingOffset(3, 1) =
      go2_para_config["TrajectoryGenerator"]["SwingLegOffset"][1].as<double>();
  go2_Trj.SwingOffset(3, 2) =
      go2_para_config["TrajectoryGenerator"]["SwingLegOffset"][2].as<double>();

  // object
  for (int i = 0; i < 3; i++)
  {
    go2_OC.object_k_p(i, i) =
        go2_para_config["Object"]["k_p"][i].as<double>();
    go2_OC.object_k_v(i, i) =
        go2_para_config["Object"]["k_v"][i].as<double>();
    go2_OC.object_k_theta(i, i) =
        go2_para_config["Object"]["k_theta"][i].as<double>();
    go2_OC.object_k_w(i, i) =
        go2_para_config["Object"]["k_w"][i].as<double>();
  }
  go2_OC.object_mass = go2_para_config["Object"]["object_mass"].as<double>();

  // BallanceController
  go2_vmc.f_min = go2_para_config["f_min"].as<double>();
  go2_vmc.f_max = go2_para_config["f_max"].as<double>();
  go2_vmc.alpha_W = go2_para_config["VMC"]["alpha_W"].as<double>();
  go2_vmc.u =
      go2_para_config["physical_paras"]["friction_coeff"].as<double>();
  for (int i = 0; i < 6; i++)
  {
    go2_vmc.S_track(i, i) = go2_para_config["VMC"]["S_track"][i].as<double>();
  }

  // SwingLegController
  for (int i = 0; i < 3; i++)
  {
    go2_SLC.k_p_toe(i, i) =
        go2_para_config["SwingLegController"]["k_p_toe"][i].as<double>();
    go2_SLC.k_v_toe(i, i) =
        go2_para_config["SwingLegController"]["k_v_toe"][i].as<double>();
  }

  // MPC_solver
  go2_mpc.mpcEnable =
      go2_para_config["Controllers_isEnable"]["mpcEnable"].as<bool>();
  go2_mpc.friction_coeff =
      go2_para_config["physical_paras"]["friction_coeff"].as<double>();
  go2_mpc.mpc_discrete_switch =
      go2_para_config["MPC"]["mpc_discrete_switch"].as<double>();
  go2_mpc.f_min = go2_para_config["f_min"].as<double>();
  go2_mpc.f_max = go2_para_config["f_max"].as<double>();
  go2_mpc.weight_f = go2_para_config["MPC"]["weight_f"].as<double>();
  for (int i = 0; i < robot_state_dim; i++)
  {
    go2_mpc.Q_qp_sub(i) = go2_para_config["MPC"]["Q_qp"][i].as<double>();
  }
  for (int i = 0; i < 3; i++)
  {
    go2_mpc.R_qp_sub(i) = go2_para_config["MPC"]["R_qp"][i].as<double>();
  }

  // NMPC_solver
  go2_nmpc.nmpcEnable =
      go2_para_config["Controllers_isEnable"]["nmpcEnable"].as<bool>();
  go2_nmpc.friction_coe =
      go2_para_config["physical_paras"]["friction_coeff"].as<double>();

  go2_nmpc.BQR3_mass =
      go2_para_config["physical_paras"]["Robot_mass"].as<double>();

  go2_nmpc.max_iter = go2_para_config["NMPC"]["max_iter"].as<Eigen::Index>();

  go2_nmpc.isrecompile = go2_para_config["NMPC"]["isrecompile"].as<bool>();

  go2_nmpc.min_friction =
      go2_para_config["NMPC"]["min_friction"].as<double>();

  for (int i = 0; i < 3; i++)
  {
    go2_nmpc.W_position[i] = go2_para_config["NMPC"]["W_p"][i].as<double>();

    go2_nmpc.W_velocity[i] = go2_para_config["NMPC"]["W_v"][i].as<double>();

    go2_nmpc.W_omega[i] = go2_para_config["NMPC"]["W_w"][i].as<double>();

    go2_nmpc.inertia_parameter[i] =
        go2_para_config["physical_paras"]["Inertia_coeff"][i].as<double>();
  };

  for (int i = 0; i < 4; i++)
  {
    go2_nmpc.W_orientation[i] =
        go2_para_config["NMPC"]["W_q"][i].as<double>();
  };

  // NMPC_solver_rpy
  go2_nmpc_rpy.nmpcEnable =
      go2_para_config["Controllers_isEnable"]["nmpcrpyEnable"].as<bool>();
  go2_nmpc_rpy.friction_coe =
      go2_para_config["physical_paras"]["friction_coeff"].as<double>();

  go2_nmpc_rpy.BQR3_mass =
      go2_para_config["physical_paras"]["Robot_mass"].as<double>();

  go2_nmpc_rpy.max_iter = go2_para_config["NMPC_rpy"]["max_iter"].as<Eigen::Index>();

  go2_nmpc_rpy.isrecompile = go2_para_config["NMPC_rpy"]["isrecompile"].as<bool>();

  go2_nmpc_rpy.min_friction =
      go2_para_config["NMPC_rpy"]["min_friction"].as<double>();

  for (int i = 0; i < 3; i++)
  {
    go2_nmpc_rpy.W_position[i] = go2_para_config["NMPC_rpy"]["W_p"][i].as<double>();

    go2_nmpc_rpy.W_velocity[i] = go2_para_config["NMPC_rpy"]["W_v"][i].as<double>();

    go2_nmpc_rpy.W_omega[i] = go2_para_config["NMPC_rpy"]["W_w"][i].as<double>();

    go2_nmpc_rpy.W_min_force[i] = go2_para_config["NMPC_rpy"]["W_f"][i].as<double>();

    go2_nmpc_rpy.inertia_parameter[i] =
        go2_para_config["physical_paras"]["Inertia_coeff"][i].as<double>();

    go2_nmpc_rpy.W_orientation[i] =
        go2_para_config["NMPC_rpy"]["W_q"][i].as<double>();
  };

  // NMPC_solver_DRBM_rpy
  go2_nmpc_DRBM_rpy.nmpcEnable =
      go2_para_config["Controllers_isEnable"]["nmpcDRBMrpyEnable"].as<bool>();
  go2_nmpc_DRBM_rpy.friction_coe =
      go2_para_config["physical_paras"]["friction_coeff"].as<double>();

  go2_nmpc_DRBM_rpy.BQR3_mass =
      go2_para_config["physical_paras"]["Robot_mass"].as<double>();

  go2_nmpc_DRBM_rpy.max_iter = go2_para_config["NMPC_D_rpy"]["max_iter"].as<Eigen::Index>();

  go2_nmpc_DRBM_rpy.isrecompile = go2_para_config["NMPC_D_rpy"]["isrecompile"].as<bool>();

  go2_nmpc_DRBM_rpy.BQR3_mass_object = go2_para_config["Object"]["object_mass"].as<double>();

  // body
  go2_nmpc_DRBM_rpy.min_friction =
      go2_para_config["NMPC_D_rpy"]["min_friction"].as<double>();
  for (int i = 0; i < 3; i++)
  {
    go2_nmpc_DRBM_rpy.W_position[i] = go2_para_config["NMPC_D_rpy"]["W_p"][i].as<double>();

    go2_nmpc_DRBM_rpy.W_velocity[i] = go2_para_config["NMPC_D_rpy"]["W_v"][i].as<double>();

    go2_nmpc_DRBM_rpy.W_omega[i] = go2_para_config["NMPC_D_rpy"]["W_w"][i].as<double>();

    go2_nmpc_DRBM_rpy.W_orientation[i] = go2_para_config["NMPC_D_rpy"]["W_q"][i].as<double>();

    go2_nmpc_DRBM_rpy.inertia_parameter[i] =
        go2_para_config["physical_paras"]["Inertia_coeff"][i].as<double>();
  };

  // object
  go2_nmpc_DRBM_rpy.object_min_friction =
      go2_para_config["NMPC_D_rpy"]["object_min_friction"].as<double>();
  for (int i = 0; i < 3; i++)
  {
    go2_nmpc_DRBM_rpy.W_position_object[i] = go2_para_config["NMPC_D_rpy"]["W_p_object"][i].as<double>();

    go2_nmpc_DRBM_rpy.W_velocity_object[i] = go2_para_config["NMPC_D_rpy"]["W_v_object"][i].as<double>();

    go2_nmpc_DRBM_rpy.W_omega_object[i] = go2_para_config["NMPC_D_rpy"]["W_w_object"][i].as<double>();

    go2_nmpc_DRBM_rpy.W_orientation_object[i] = go2_para_config["NMPC_D_rpy"]["W_q_object"][i].as<double>();

    go2_nmpc_DRBM_rpy.inertia_parameter_object[i] =
        go2_para_config["physical_paras"]["Inertia_coeff_object"][i].as<double>();
  };

  // NMPC_solver_DRBM
  go2_nmpc_DRBM.nmpcEnable =
      go2_para_config["Controllers_isEnable"]["nmpcDRBMEnable"].as<bool>();
  go2_nmpc_DRBM.friction_coe =
      go2_para_config["physical_paras"]["friction_coeff"].as<double>();

  go2_nmpc_DRBM.BQR3_mass =
      go2_para_config["physical_paras"]["Robot_mass"].as<double>();

  go2_nmpc_DRBM.max_iter = go2_para_config["NMPC_D"]["max_iter"].as<Eigen::Index>();

  go2_nmpc_DRBM.isrecompile = go2_para_config["NMPC_D"]["isrecompile"].as<bool>();

  go2_nmpc_DRBM.BQR3_mass_object = go2_para_config["Object"]["object_mass"].as<double>();

  go2_nmpc_DRBM.min_friction =
      go2_para_config["NMPC_D"]["min_friction"].as<double>();

  // body
  for (int i = 0; i < 3; i++)
  {
    go2_nmpc_DRBM.W_position[i] = go2_para_config["NMPC_D"]["W_p"][i].as<double>();

    go2_nmpc_DRBM.W_velocity[i] = go2_para_config["NMPC_D"]["W_v"][i].as<double>();

    go2_nmpc_DRBM.W_omega[i] = go2_para_config["NMPC_D"]["W_w"][i].as<double>();

    go2_nmpc_DRBM.inertia_parameter[i] =
        go2_para_config["physical_paras"]["Inertia_coeff"][i].as<double>();
  };

  for (int i = 0; i < 4; i++)
  {
    go2_nmpc_DRBM.W_orientation[i] =
        go2_para_config["NMPC_D"]["W_q"][i].as<double>();
  };

  // object
  for (int i = 0; i < 3; i++)
  {
    go2_nmpc_DRBM.W_position_object[i] = go2_para_config["NMPC_D"]["W_p_object"][i].as<double>();

    go2_nmpc_DRBM.W_velocity_object[i] = go2_para_config["NMPC_D"]["W_v_object"][i].as<double>();

    go2_nmpc_DRBM.W_omega_object[i] = go2_para_config["NMPC_D"]["W_w_object"][i].as<double>();

    go2_nmpc_DRBM.inertia_parameter_object[i] =
        go2_para_config["physical_paras"]["Inertia_coeff_object"][i].as<double>();
  };

  for (int i = 0; i < 4; i++)
  {
    go2_nmpc_DRBM.W_orientation_object[i] =
        go2_para_config["NMPC_D"]["W_q_object"][i].as<double>();
  };

  // WBC
  go2_wbc.u =
      go2_para_config["physical_paras"]["friction_coeff"].as<double>();
  go2_wbc.f_min = go2_para_config["f_min"].as<double>();
  go2_wbc.f_max = go2_para_config["f_max"].as<double>();
  go2_wbc.ratio = go2_para_config["WBC"]["ratio"].as<double>();
  go2_wbc.method=go2_para_config["WBC"]["method"].as<int>();


  // WBC_Arm
  go2_wbc_arm.u =
      go2_para_config["physical_paras"]["friction_coeff"].as<double>();
  go2_wbc_arm.f_min = go2_para_config["f_min"].as<double>();
  go2_wbc_arm.f_max = go2_para_config["f_max"].as<double>();
  go2_wbc_arm.ratio = go2_para_config["WBC"]["ratio"].as<double>();

  // Dynamic
  go2_dynamic.isUseDynamic =
      go2_para_config["Dynamic"]["isUseDynamic"].as<bool>();
  for (int i = 0; i < 3; i++)
  {
    go2_dynamic.k_p_joint(i, i) =
        go2_para_config["Dynamic"]["k_p_joint"][i].as<double>();
    go2_dynamic.k_v_joint(i, i) =
        go2_para_config["Dynamic"]["k_v_joint"][i].as<double>();
  }

  go2_dynamic_arm.isUseDynamic =
      go2_para_config["Dynamic"]["isUseDynamic"].as<bool>();
  for (int i = 0; i < 3; i++)
  {
    go2_dynamic_arm.k_p_joint(i, i) =
        go2_para_config["Dynamic"]["k_p_joint"][i].as<double>();
    go2_dynamic_arm.k_v_joint(i, i) =
        go2_para_config["Dynamic"]["k_v_joint"][i].as<double>();
  }
  for (int i = 0; i < 6; i++)
  {
    go2_dynamic_arm.k_p_armjoint(i) = go2_para_config["Dynamic"]["k_p_joint_arm"][i].as<double>();
    go2_dynamic_arm.k_v_armjoint(i) = go2_para_config["Dynamic"]["k_v_joint_arm"][i].as<double>();
  }

  // Info
  go2_Info.isPrintOutInfo =
      go2_para_config["Info"]["isPrintOutInfo"].as<bool>();

  // SaveLog
  go2_Log.isSaveLog = go2_para_config["isSaveLog"].as<bool>();
}

ClassManager::~ClassManager() {}