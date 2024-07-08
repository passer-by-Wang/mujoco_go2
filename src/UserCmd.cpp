/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     UserCmd.cpp
  @brief    UserCmd
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

#include "../include/UserCmd.h"

UserCmd::UserCmd(/* args */) {
  ispressed = false;
  Record_flag = false;
  KeyCmd = -1;
  LastKeyCmd = -1;
  isStop = 0;
}

void UserCmd::GetUserInput(TrajectoryGenerator& Trj, FiniteStateMachine& FSM) {
  FSM.ischangegait = 0;

  if (KeyCmd == -1)
    ispressed = false;
  else
    ispressed = true;

  if (LastKeyCmd == KeyCmd && ispressed == true) {
  } else {
    // 字母大写
    switch (KeyCmd) {
      case ' ':
        Record_flag = false;
        isStop = 1;
        cout << "press space, break out!" << endl;
        break;
      case 'A':
        Trj.TargetW(2) += 0.05;  // 向左加速旋转
        cout << "[Mujoco Manager] press A, Accelerate rotation to the left with "
                "velocity: "
             << Trj.TargetW(2) << endl;
        break;
      case 'D':
        Trj.TargetW(2) -= 0.02;  // 向左减速旋转
        cout << "[Mujoco Manager] press D, Decelerate rotation to the left with "
                "velocity: "
             << Trj.TargetW(2) << endl;

        break;
      case 'W':
        Trj.TargetV(0) += 0.2;  // 向前加速
        cout << "[Mujoco Manager] press W, accelerate forward with velocity: "
             << Trj.TargetV(0) << endl;
        break;
      case 'B':
        Trj.TargetV(0) -= 0.2;  // 向前减速
        cout << "[Mujoco Manager] press B, decelerate forward with velocity: "
             << Trj.TargetV(0) << endl;
        break;
      case 'S':
        Trj.TargetV.setZero();
        Trj.TargetW.setZero();
        cout << "[Mujoco Manager] press S, set velocity to zero" << endl;
        break;
      case 'U':
        Trj.TargetP(2, 0) += 0.02;  // 增加身体高度
        cout << "[Mujoco Manager] press up" << endl;
        break;
        break;
      case 'J':
        Trj.TargetP(2, 0) -= 0.02;  // 降低身体高度
        cout << "[Mujoco Manager] press down" << endl;
        break;
      case 315:  // 上箭头'↑'
      {
        Trj.object_TargetP(0)+=0.02;
        cout<<"[Mujoco Manager] object moves forward"<<endl;
        break;
      }
      case 317:  // 下箭头'↓'
      {
        Trj.object_TargetP(0)-=0.02;
        cout<<"[Mujoco Manager] object moves back"<<endl;
        break;
      }
      case 314:  // 左箭头
      { 
        Trj.object_TargetP(1)+=0.02;
        cout << "[Mujoco Manager] object moves left" << endl;
        break;
      }
      case 316:  // 右箭头
      {
        Trj.object_TargetP(1)-=0.02;
        cout << "[Mujoco Manager] object moves right" << endl;
        break;
      }
      case 'I':  // 向上运动
      { 
        Trj.object_TargetP(2)+=0.02;
        cout << "[Mujoco Manager] object moves up" << endl;
        break;
      }
      case 'K':  // 向下运动
      {
        Trj.object_TargetP(2)-=0.02;
        cout << "[Mujoco Manager] object moves down" << endl;
        break;
      }
      case 'T':
        FSM.GaitTag = 1;
        FSM.current_gait = FSM.trot;
        cout << "[Mujoco Manager] press T   Gait: trot" << endl;
        break;
      case 'P':
        Trj.TargetV.setZero();
        Trj.TargetW.setZero();
        FSM.GaitTag = 0;
        FSM.current_gait = FSM.stand;
        cout << "[Mujoco Manager] press P   Gait: stand" << endl;
        break;
      case 'Q':
        Trj.TargetV(1) += 0.05;  // 向左加速
        cout << "[Mujoco Manager] press Q, accelerate left with velocity: "
             << Trj.TargetV(1) << endl;
        break;
      case 'E':
        Trj.TargetV(1) -= 0.05;  // 向左减速
        cout << "[Mujoco Manager] press E, decelerate left with velocity: "
             << Trj.TargetV(1) << endl;
        break;
      case 'M':
        FSM.GaitTag = 2;
        FSM.current_gait = FSM.pace;
        cout << "[Mujoco Manager] press M   Gait: pace" << endl;
        break;
      case 'N':
        FSM.GaitTag = 3;
        FSM.current_gait = FSM.walk;
        cout << "[Mujoco Manager] press N   Gait: walk" << endl;
        break;
      case 'F':
        FSM.GaitTag = 4;
        FSM.current_gait = FSM.flytrot;
        cout << "[Mujoco Manager] press F   Gait: flytrot" << endl;
        break;
      case 'L':
        FSM.GaitTag = 5;
        FSM.current_gait = FSM.bound;
        cout << "[Mujoco Manager] press L   Gait: bound" << endl;
        break;
      default:
        break;
    }
    if (FSM.lastGaitTag != FSM.GaitTag) {
      FSM.ischangegait = 1;
    }
  }
  LastKeyCmd = KeyCmd;
  FSM.lastGaitTag = FSM.GaitTag;
}

UserCmd::~UserCmd() {}