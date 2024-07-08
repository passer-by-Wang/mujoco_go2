/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     FiniteStateMachine.cpp
  @brief    FiniteStateMachine
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

#include "../include/FiniteStateMachine.h"
FiniteStateMachine::FiniteStateMachine(/* args */) {
  phase = 0;
  time = 0;
  ischangephase = 0;
  ischangegait = 0;
  ischangegait1 = 0;
  GaitTag = 0;
  lastGaitTag=GaitTag;

  // 机器人腿部序号定义
  //   LF:3      RF:1
  //   LH:4      RH:2
  stand.GaitSequence.resize(1, 4);
  stand.GaitSequence << 1, 1, 1, 1;  // 步态序列，触地为1，摆动为0
  stand.stand_T = 0.2;  // 支撑相时间，在确定落足点位置中使用
  stand.swing_T = 0;  // 摆动相时间
  stand.SwingHeight << 0, 0, 0;  // 抬腿高度

  trot.GaitSequence.resize(2, 4);
  trot.GaitSequence << 1, 0, 0, 1, 0, 1, 1, 0;
  trot.stand_T = 0.25;
  trot.swing_T = 0.25;
  trot.SwingHeight << 0, 0, 0.08;  // 抬腿高度

  bound.GaitSequence.resize(2, 4);
  bound.GaitSequence << 0, 1, 0, 1, 1, 0, 1, 0;
  bound.stand_T = 0.25;
  bound.swing_T = 0.25;
  bound.SwingHeight << 0, 0, 0.10;

  pace.GaitSequence.resize(2, 4);
  pace.GaitSequence << 1, 1, 0, 0, 0, 0, 1, 1;
  pace.stand_T = 0.25;
  pace.swing_T = 0.25;
  pace.SwingHeight << 0, 0, 0.08;

  walk.GaitSequence.resize(4, 4);   //
  walk.GaitSequence << 0, 1, 1, 1,  // 四腿支撑
      1, 1, 1, 0,                   // 后腿支撑
      1, 1, 0, 1, 1, 0, 1, 1;       // 前腿支撑
  walk.swing_T = 0.2;
  walk.stand_T = 3 * walk.swing_T;
  walk.SwingHeight << 0, 0, 0.1;
  current_gait = stand;
  currentGaitTag=0;

  flytrot.GaitSequence.resize(4, 4);
  flytrot.GaitSequence << 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0;
  flytrot.stand_T = 0.2;
  flytrot.swing_T = 0.3;
  flytrot.SwingHeight << 0.0, 0.0, 0.08;
}

void FiniteStateMachine::setLegPhase(int& Gait, StateEstimator& SE) {

  ischangegait = 0;
  ischangegait1 = 0;
  // if (lastGaitTag != GaitTag)
  // {
  //   ischangegait = 1;
  //   ischangegait1 = 1;
  // }
  if(lastGaitTag != GaitTag&&time<=0.002){
    ischangegait=1;
    lastGaitTag = GaitTag;
    currentGaitTag=Gait;
    switch(currentGaitTag){
      case 0:
        current_gait = stand;
        break;
      case 1:
        current_gait = trot;
        break;
      case 3:
        current_gait = walk;
        break;
      case 4:
        current_gait = flytrot;
        break;
      default:
        break;
    }
  }
  
  time += double(timestep) / 1000.0;

  // stand
  if (currentGaitTag == 0) {
    if (time >= current_gait.stand_T)  // 时间触发，相位序列更新后time时间置零
    {
      time=0.0;
    }
    LegPhase = current_gait.GaitSequence.row(0).transpose();
  }

  // trot
  if (currentGaitTag == 1) {
    if (time <=
        1.00 * current_gait.stand_T)  // 时间触发，相位序列更新后time时间置零
    {
      LegPhase = current_gait.GaitSequence.row(phase).transpose();
    } 
    // else if (time > 0.95 * current_gait.stand_T &&
    //            time <= 1.05 * current_gait.stand_T) {
    //   if (SE.LegContactState.sum() - SE.lastLegContactState.sum() > 0) {
    //     PhaseUpdate();
    //   }

    //   LegPhase = current_gait.GaitSequence.row(phase).transpose();
    // } 
    else if (time > 1.00 * current_gait.stand_T)  // 强制切换
    {
      PhaseUpdate();
      LegPhase = current_gait.GaitSequence.row(phase).transpose();
    }
  }

  //   if (currentGaitTag == 1) {
  //   if (time <=
  //       0.95 * current_gait.stand_T)  // 时间触发，相位序列更新后time时间置零
  //   {
  //     LegPhase = current_gait.GaitSequence.row(phase).transpose();
  //   } 
  //   else if (time > 0.95 * current_gait.stand_T &&
  //              time <= 1.05 * current_gait.stand_T) {
  //     if (SE.LegContactState.sum() - SE.lastLegContactState.sum() > 0) {
  //       PhaseUpdate();
  //     }

  //     LegPhase = current_gait.GaitSequence.row(phase).transpose();
  //   } 
  //   else if (time > 1.05 * current_gait.stand_T)  // 强制切换
  //   {
  //     PhaseUpdate();
  //     LegPhase = current_gait.GaitSequence.row(phase).transpose();
  //   }
  // }

  // pace
  if (currentGaitTag == 2) {
    if (time <=
        0.8 * current_gait.stand_T)  // 时间触发，相位序列更新后time时间置零
    {
      LegPhase = current_gait.GaitSequence.row(phase).transpose();
    } else if (time > 0.8 * current_gait.stand_T &&
               time <= 1.2 * current_gait.stand_T) {
      if (SE.LegContactState.sum() - SE.lastLegContactState.sum() > 0) {
        PhaseUpdate();
      }

      LegPhase = current_gait.GaitSequence.row(phase).transpose();
    } else if (time > 1.2 * current_gait.stand_T)  // 强制切换
    {
      PhaseUpdate();
      LegPhase = current_gait.GaitSequence.row(phase).transpose();
    }
  }

  // walk
  if (currentGaitTag == 3) {
    if (time <=
        0.8 * current_gait.stand_T)  // 时间触发，相位序列更新后time时间置零
    {
      LegPhase = current_gait.GaitSequence.row(phase).transpose();
    } else if (time > 0.8 * current_gait.stand_T &&
               time <= 1.2 * current_gait.stand_T) {
      if (SE.LegContactState.sum() - SE.lastLegContactState.sum() > 0) {
        PhaseUpdate();
      }

      LegPhase = current_gait.GaitSequence.row(phase).transpose();
    } else if (time > 1.2 * current_gait.stand_T)  // 强制切换
    {
      PhaseUpdate();
      LegPhase = current_gait.GaitSequence.row(phase).transpose();
    }
  }

  // flytrot
  if (currentGaitTag == 4) {
    for (int i = 0; i < 4; i++) {
      if (current_gait.GaitSequence.row(phase)(i) == 0)
        time += double(timestep) / 1000;
      else
        time = 0;
    }
    if (current_gait.GaitSequence.row(phase).sum() == 2)  // 对角支撑
    {
      if (time > current_gait.stand_T)  // 时间到了切换
      {
        PhaseUpdate();
        LegPhase = current_gait.GaitSequence.row(phase).transpose();
      } else {
        LegPhase = current_gait.GaitSequence.row(phase).transpose();
      }
    } else if (current_gait.GaitSequence.row(phase).sum() ==
               0)  // 四腿都在摆动相
    {
      LegPhase = current_gait.GaitSequence.row(phase).transpose();
      if (time > ((current_gait.swing_T - current_gait.stand_T)) / 2) {
        PhaseUpdate();
        LegPhase = current_gait.GaitSequence.row(phase).transpose();
      }
    }
  }

  SE.lastLegContactState = SE.LegContactState;
}

void FiniteStateMachine::PhaseUpdate() {
  time = 0.0;  // 步态序列相加1
  phase += 1;  // 时间进度置零
  ischangephase = 1;
  if (phase >=
      current_gait.GaitSequence.rows())  // 限制递增，在步态序列行数间递增
  {
    phase = 0;
  }
}

FiniteStateMachine::~FiniteStateMachine() {}