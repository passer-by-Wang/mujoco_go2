/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     user_func.cpp
  @brief    user_func
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
#include "user_func.h"

#include <chrono>
#include <memory>

bitbot::MujocoJoint *bodyjoint[4][3];
bitbot::MujocoJoint *armjoint[6];
bitbot::MujocoImu *Imu;
bitbot::MujocoForceSensor *forcesensor[4];
ClassManager Manager;

void ConfigFunc(const bitbot::MujocoBus &bus, UserData &)
{
  // Get joint device
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
    {
      bodyjoint[i][j] = bus.GetDevice<bitbot::MujocoJoint>(3 * i + j + 1).value();
    }

  for(int i=0;i<6;i++){
    armjoint[i] = nullptr;
  }

  // Get ForceSensor device
  for (int i = 0; i < 4; i++)
    forcesensor[i] = bus.GetDevice<bitbot::MujocoForceSensor>(i + 13).value();

  // Get IMU device
  Imu = bus.GetDevice<bitbot::MujocoImu>(17).value();

  // Loading the parameters for object
  ParametersLoading();
}

void ParametersLoading()
{
  Manager.Configure_para();
}

std::optional<bitbot::StateId> EventfMove(bitbot::EventValue value, UserData &user_data){
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    Manager.go2_Trj.object_TargetP(0)+=0.04;
        cout<<"[Mujoco Manager] object moves forward to "<<Manager.go2_Trj.object_TargetP(0)<<"m in X direction"<<endl;
  }
  return std::nullopt;
}
std::optional<bitbot::StateId> EventbMove(bitbot::EventValue value, UserData &user_data){
if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    Manager.go2_Trj.object_TargetP(0)-=0.04;
        cout<<"[Mujoco Manager] object moves back to "<<Manager.go2_Trj.object_TargetP(0)<<"m in X direction"<<endl;
  }
  return std::nullopt;
}
std::optional<bitbot::StateId> EventUpMove(bitbot::EventValue value, UserData &user_data){
if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    Manager.go2_Trj.object_TargetP(2)+=0.04;
        cout << "[Mujoco Manager] object moves up to " << Manager.go2_Trj.object_TargetP(2)<<"m in Z direction"<<endl;
  }
  return std::nullopt;
}
std::optional<bitbot::StateId> EventDownMove(bitbot::EventValue value, UserData &user_data){
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    Manager.go2_Trj.object_TargetP(2)-=0.04;
        cout << "[Mujoco Manager] object moves down to " << Manager.go2_Trj.object_TargetP(2)<<"m in Z direction"<<endl;
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventLMove(bitbot::EventValue value, UserData &user_data){
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    Manager.go2_Trj.object_TargetP(1)+=0.04;
        cout << "[Mujoco Manager] object moves left to " << Manager.go2_Trj.object_TargetP(1)<<"m in Y direction"<<endl;
  }
  return std::nullopt;
}
std::optional<bitbot::StateId> EventRMove(bitbot::EventValue value, UserData &user_data){
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    Manager.go2_Trj.object_TargetP(1)-=0.04;
        cout << "[Mujoco Manager] object moves right to " << Manager.go2_Trj.object_TargetP(1)<<"m in Y direction"<<endl;
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventWait(bitbot::EventValue, UserData &)
{
  return std::nullopt;
}

std::optional<bitbot::StateId> EventTest(bitbot::EventValue value, UserData &)
{
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
    return static_cast<bitbot::StateId>(States::JointSinPos);
  else
    return std::nullopt;
}

std::optional<bitbot::StateId> EventAdd(bitbot::EventValue value, UserData &user_data)
{
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    if (user_data.sin < 60)
      user_data.sin += 1;
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventMinus(bitbot::EventValue value, UserData &user_data)
{
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    if (user_data.sin > 1)
      user_data.sin -= 1;
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventTrot(bitbot::EventValue value, UserData &user_data)
{
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    Manager.go2_FSM.GaitTag = 1;
    cout << "[Mujoco Manager] press 7   Gait: trot" << endl;
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventForwardMove(bitbot::EventValue value, UserData &user_data)
{
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    if (Manager.go2_FSM.GaitTag == 1 || Manager.go2_FSM.GaitTag == 4)
    {
      Manager.go2_Trj.TargetV_user(0) += 0.2; // 向前加速
      cout << "[Mujoco Manager] press W, accelerate forward with velocity: "
           << Manager.go2_Trj.TargetV_user(0) << endl;
    }
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventBackMove(bitbot::EventValue value, UserData &user_data)
{
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    if (Manager.go2_FSM.GaitTag == 1 || Manager.go2_FSM.GaitTag == 4)
    {
      Manager.go2_Trj.TargetV_user(0) -= 0.2; // 向前减速
      cout << "[Mujoco Manager] press B, decelerate forward with velocity: "
           << Manager.go2_Trj.TargetV_user(0) << endl;
    }
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventRightMove(bitbot::EventValue value, UserData &user_data)
{
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    if (Manager.go2_FSM.GaitTag == 1 || Manager.go2_FSM.GaitTag == 4)
    {
      Manager.go2_Trj.TargetV_user(1) -= 0.15; // 向左减速
      cout << "[Mujoco Manager] press E, decelerate left with velocity: "
           << Manager.go2_Trj.TargetV_user(1) << endl;
    }
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventLeftMove(bitbot::EventValue value, UserData &user_data)
{
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    if (Manager.go2_FSM.GaitTag == 1 || Manager.go2_FSM.GaitTag == 4)
    {
      Manager.go2_Trj.TargetV_user(1) += 0.15; // 向左加速
      cout << "[Mujoco Manager] press Q, accelerate left with velocity: "
           << Manager.go2_Trj.TargetV_user(1) << endl;
    }
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventRightRotate(bitbot::EventValue value, UserData &user_data)
{
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    if (Manager.go2_FSM.GaitTag == 1 || Manager.go2_FSM.GaitTag == 4)
    {
      Manager.go2_Trj.TargetW_user(2) -= 0.15; // 向左减速旋转
      cout << "[Mujoco Manager] press D, Decelerate rotation to the left with "
              "velocity: "
           << Manager.go2_Trj.TargetW_user(2) << endl;
    }
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventLeftRotate(bitbot::EventValue value, UserData &user_data)
{
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    if (Manager.go2_FSM.GaitTag == 1 || Manager.go2_FSM.GaitTag == 4)
    {
      Manager.go2_Trj.TargetW_user(2) += 0.15; // 向左加速旋转
      cout << "[Mujoco Manager] press A, Accelerate rotation to the left with "
              "velocity: "
           << Manager.go2_Trj.TargetW_user(2) << endl;
    }
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventStand(bitbot::EventValue value, UserData &user_data)
{
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    Manager.go2_Trj.TargetV_user.setZero();
    Manager.go2_Trj.TargetW_user.setZero();
    Manager.go2_FSM.GaitTag = 0;
    cout << "[Mujoco Manager] press P   Gait: stand" << endl;
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventZero(bitbot::EventValue value, UserData &user_data)
{
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    Manager.go2_Trj.TargetV_user.setZero();
    Manager.go2_Trj.TargetW_user.setZero();
    cout << "[Mujoco Manager] press S, set velocity to zero" << endl;
  }

  return std::nullopt;
}

std::optional<bitbot::StateId> EventPlanning(bitbot::EventValue value, UserData &user_data){
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    Manager.go2_Trj.TargetP[0] += 0.05;
    printinfo(Manager.go2_Trj.TargetP[0]);
    printinfo(Manager.go2_SE.body_CoMPos[0]);
    cout << "[Mujoco Manager] press Z, start planning." << endl;
  }
  return std::nullopt;
}

std::optional<bitbot::StateId> EventUpOrDown(bitbot::EventValue value, UserData &user_data){
  if (value == static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up))
  {
    if(Manager.go2_Trj.UpOrDown_Flag){
      Manager.go2_Trj.UpOrDown_Flag = 0;
      cout << "[Mujoco Manager] press T, start down." << endl;
    }
    else{
      Manager.go2_Trj.UpOrDown_Flag = 1;
      cout << "[Mujoco Manager] press T, start up." << endl;
    }
    
  }
  return std::nullopt;
}

void StateRun(const bitbot::KernelInterface &kernel, Kernel::ExtraData &extra_data, UserData &user_data)
{
  Manager.run(bodyjoint, Imu, forcesensor);
}

void StateWaiting(const bitbot::KernelInterface &kernel, Kernel::ExtraData &extra_data, UserData &user_data)
{
}

void StateJointSinPos(const bitbot::KernelInterface &kernel, Kernel::ExtraData &extra_data, UserData &user_data)
{
}

void FinishFunc(UserData &)
{
    if (Manager.go2_Log.isSaveLog)
  {
    Manager.go2_Log.SaveData2Log();
  }
  else
  {
    Manager.go2_Log.LogDataBuff->clear();
  }
  std::cout << "finish" << std::endl;
}