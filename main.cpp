/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     main.cpp
  @brief    main
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
#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "bitbot_mujoco/device/mujoco_joint.h"

#include "user_func.h"

#define _USE_MATH_DEFINES
#include <math.h>

int main(int argc, char const *argv[])
{
  // 定义kernel对象，参数为配置文件
  Kernel kernel("../../config/bitbot_quadruped_go2.xml");

  // 注册Config函数
  kernel.RegisterConfigFunc(&ConfigFunc);
  // 注册Finish函数
  kernel.RegisterFinishFunc(&FinishFunc);

 // 注册event
  kernel.RegisterEvent("waiting", static_cast<bitbot::StateId>(Events::Wait), &EventWait, true);
  kernel.RegisterEvent("test", static_cast<bitbot::StateId>(Events::Test), &EventTest);
  kernel.RegisterEvent("add", static_cast<bitbot::StateId>(Events::Add), &EventAdd, true);
  kernel.RegisterEvent("minus", static_cast<bitbot::StateId>(Events::Minus), &EventMinus, true);

  kernel.RegisterEvent("Trot", static_cast<bitbot::StateId>(Events::Trot), &EventTrot, true);
  kernel.RegisterEvent("forwardMove", static_cast<bitbot::StateId>(Events::forwardMove), &EventForwardMove, true);
  kernel.RegisterEvent("backMove", static_cast<bitbot::StateId>(Events::backMove), &EventBackMove, true);
  kernel.RegisterEvent("leftMove", static_cast<bitbot::StateId>(Events::leftMove), &EventLeftMove, true);
  kernel.RegisterEvent("rightMove", static_cast<bitbot::StateId>(Events::rightMove), &EventRightMove, true);
  kernel.RegisterEvent("rightRotate", static_cast<bitbot::StateId>(Events::rightRotate), &EventRightRotate, true);
  kernel.RegisterEvent("leftRotate", static_cast<bitbot::StateId>(Events::leftRotate), &EventLeftRotate, true);
  kernel.RegisterEvent("stand", static_cast<bitbot::StateId>(Events::stand), &EventStand, true);
  kernel.RegisterEvent("zeros", static_cast<bitbot::StateId>(Events::zeros), &EventZero, true);
  
  kernel.RegisterEvent("UpMove", static_cast<bitbot::StateId>(Events::UpMove), &EventUpMove, true);
  kernel.RegisterEvent("DownMove", static_cast<bitbot::StateId>(Events::DownMove), &EventDownMove, true);
  kernel.RegisterEvent("fMove", static_cast<bitbot::StateId>(Events::fMove), &EventfMove, true);
  kernel.RegisterEvent("bMove", static_cast<bitbot::StateId>(Events::bMove), &EventbMove, true);
  kernel.RegisterEvent("LMove", static_cast<bitbot::StateId>(Events::LMove), &EventLMove, true);
  kernel.RegisterEvent("RMove", static_cast<bitbot::StateId>(Events::RMove), &EventRMove, true);

  kernel.RegisterEvent("planning", static_cast<bitbot::StateId>(Events::planning), &EventPlanning, true);
  kernel.RegisterEvent("UpOrDown", static_cast<bitbot::StateId>(Events::UpOrDown), &EventUpOrDown, true);

  // 注册state
  // kernel.RegisterState("waiting", static_cast<bitbot::StateId>(States::Waiting), &StateWaiting, {
  //   static_cast<bitbot::StateId>(Events::Test)
  // });
  kernel.RegisterState("Run", static_cast<bitbot::StateId>(States::Run), &StateRun, {});

  // 设置用户的第一个state
  kernel.SetFirstState(static_cast<bitbot::StateId>(States::Run));

  // 运行
  kernel.Run();
  
  return 0;
}
