/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     MujocoInterface.cpp
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

#include "../include/MujocoInterface.h"

MujocoInterface::MujocoInterface(/* args */)
{
    InitForceSensorFlag = 1;
}

void MujocoInterface::ReadJoint(MujocoJoint *bodyjoint[4][3],  StateEstimator &SE)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            SE.body_joint_angle(i, j) = bodyjoint[i][j]->GetActualPosition();
            SE.body_joint_vel(i, j) = bodyjoint[i][j]->GetActualVelocity();
        }
    }
    if(SE.init_Read_Joint){
        SE.init_body_joint_angle = SE.body_joint_angle;
        SE.init_Read_Joint = false;
    }
}

void MujocoInterface::ReadImu(MujocoImu *imu, StateEstimator &SE)
{
    SE.body_CoMAcc(0) = imu->GetAccX();
    SE.body_CoMAcc(1) = imu->GetAccY();
    SE.body_CoMAcc(2) = imu->GetAccZ();

    // printinfo(SE.body_CoMAcc.transpose());

    SE.Last_body_CoMAngle = SE.body_CoMAngle;

    SE.body_CoMAngle(0) = imu->GetRoll();
    SE.body_CoMAngle(1) = imu->GetPitch();
    SE.body_CoMAngle(2) = imu->GetYaw();

    SE.body_CoMAngularVel(0) = imu->GetGyroX();
    SE.body_CoMAngularVel(1) = imu->GetGyroY();
    SE.body_CoMAngularVel(2) = imu->GetGyroZ();
    // printinfo(SE.body_CoMAngularVel.transpose());

    SE.body_CoMAngle(2) = imu->GetYaw() + SE.cycle * 2 * pi;

    if (SE.body_CoMAngle(2) - SE.Last_body_CoMAngle(2) < (-1.99 * pi))
    {
        SE.cycle += 1;
        SE.body_CoMAngle(2) = imu->GetYaw() + SE.cycle * 2 * pi;
    }
    else if (SE.body_CoMAngle(2) - SE.Last_body_CoMAngle(2) > (1.99 * pi))
    {
        SE.cycle -= 1;
        SE.body_CoMAngle(2) = imu->GetYaw() + SE.cycle * 2 * pi;
    }
    // printinfo(SE.body_CoMAngle.transpose());
}

void MujocoInterface::ReadForceSensor(MujocoForceSensor *ForceSensor[4], StateEstimator &SE)
{
    if (InitForceSensorFlag)
    {
        for (int i = 0; i < 4; i++)
        {
            InitForceSensorData(i, 0) = ForceSensor[i]->GetForceX();
            InitForceSensorData(i, 1) = ForceSensor[i]->GetForceY();
            InitForceSensorData(i, 2) = ForceSensor[i]->GetForceZ();
        }
        InitForceSensorFlag = 0;
    }

    for (int i = 0; i < 4; i++)
    {
        SE.RealLegContactForces(i, 0) = ForceSensor[i]->GetForceX() - InitForceSensorData(i, 0);
        SE.RealLegContactForces(i, 1) = (ForceSensor[i]->GetForceY() - InitForceSensorData(i, 1));
        SE.RealLegContactForces(i, 2) = (ForceSensor[i]->GetForceZ() - InitForceSensorData(i, 2));
    }
    // printinfo(SE.RealLegContactForces);
}

void MujocoInterface::ReadDatafromRobot(MujocoJoint *bodyjoint[4][3], MujocoImu *imu,
                                        MujocoForceSensor *ForceSensor[4], StateEstimator &SE)
{

    ReadJoint(bodyjoint, SE);
    ReadImu(imu, SE);
    ReadForceSensor(ForceSensor, SE);
}

void MujocoInterface::SendBodyJointAngletoRobot(MujocoJoint *bodyjoint[4][3], const Mat43 &BodyinitPos)
{
    if (initBodyPos)
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                bodyjoint[i][j]->SetMode(MujocoJointMode::POSITION);
            }
        }
        initBodyPos = 0;
    }

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            bodyjoint[i][j]->SetTargetPosition(BodyinitPos(i, j));
        }
    }
}

void MujocoInterface::SendBodyJointTorquetoRobot(MujocoJoint *bodyjoint[4][3], const Mat43 &BodyJointTorque)
{
    if (initBodyTorque)
    {
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                bodyjoint[i][j]->SetMode(MujocoJointMode::TORQUE);
            }
        }
        initBodyTorque = 0;
    }
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            bodyjoint[i][j]->SetTargetTorque(BodyJointTorque(i, j));
        }
    }
}

MujocoInterface::~MujocoInterface()
{
}
