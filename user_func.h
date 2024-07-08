/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     user_func.h
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
#pragma once

#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/device/mujoco_force_sensor.h"
#include "bitbot_mujoco/device/mujoco_imu.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include "./include/ClassManager.h"

enum Events
{
  Wait = 1001,
  Test,
  Add,
  Minus,

  Trot = 2001,
  forwardMove = 2002,
  backMove = 2003,
  rightRotate = 2004,
  leftRotate = 2005,
  zeros = 2006,
  stand = 2007,
  rightMove = 2008,
  leftMove = 2009,
  UpMove = 2010,
  DownMove = 2011,
  fMove = 2012,
  bMove = 2013,
  LMove=2014,
  RMove=2015,
  planning=2016,
  UpOrDown=2017
};

enum class States : bitbot::StateId
{
  Run = 1001,
  JointSinPos
};

struct UserData
{
  double sin = 10;
};

using Kernel = bitbot::MujocoKernel<UserData, "time", "sin">;

void ConfigFunc(const bitbot::MujocoBus &bus, UserData &);

std::optional<bitbot::StateId> EventWait(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventTest(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventAdd(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventMinus(bitbot::EventValue value, UserData &user_data);

std::optional<bitbot::StateId> EventTrot(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventForwardMove(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventBackMove(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventLeftMove(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventRightMove(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventRightRotate(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventLeftRotate(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventStand(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventZero(bitbot::EventValue value, UserData &user_data);

std::optional<bitbot::StateId> EventfMove(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventbMove(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventUpMove(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventDownMove(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventLMove(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventRMove(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventUpOrDown(bitbot::EventValue value, UserData &user_data);

std::optional<bitbot::StateId> EventPlanning(bitbot::EventValue value, UserData &user_data);

void ParametersLoading();

void StateRun(const bitbot::KernelInterface &kernel, Kernel::ExtraData &extra_data, UserData &user_data);

void StateWaiting(const bitbot::KernelInterface &kernel, Kernel::ExtraData &extra_data, UserData &user_data);

void StateJointSinPos(const bitbot::KernelInterface &kernel, Kernel::ExtraData &extra_data, UserData &user_data);

void FinishFunc(UserData &);
