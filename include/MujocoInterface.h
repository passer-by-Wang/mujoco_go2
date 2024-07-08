/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     MujocoInterface.h
  @brief    MujocoInterface
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
#include "common.h"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/device/mujoco_force_sensor.h"
#include "bitbot_mujoco/device/mujoco_imu.h"
#include "StateEstimator.h"
using namespace bitbot;
class MujocoInterface
{
private:
    /* data */
public:

    bool initBodyTorque=1;
    bool initBodyPos=1;
    bool initArmTorque=1;
    bool initArmPos=1;

    Matrix<double, 4, 3> InitForceSensorData;

    bool InitForceSensorFlag;

    void ReadJoint(MujocoJoint *bodyjoint[4][3], StateEstimator& SE);

    void ReadImu(MujocoImu *imu, StateEstimator& SE);

    void ReadForceSensor(MujocoForceSensor *ForceSensor[4], StateEstimator& SE);

    void ReadDatafromRobot(MujocoJoint *bodyjoint[4][3], MujocoImu *imu,
                           MujocoForceSensor *ForceSensor[4], StateEstimator &SE);

    void SendBodyJointAngletoRobot(MujocoJoint *bodyjoint[4][3], const Mat43& BodyinitPos);


    void SendBodyJointTorquetoRobot(MujocoJoint *bodyjoint[4][3], const Mat43& BodyJointTorque);

    MujocoInterface(/* args */);
    ~MujocoInterface();
};


