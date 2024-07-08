#pragma once
#include "BallanceController.h"
#include "Dynamic.h"
#include "FiniteStateMachine.h"
#include "MPC_solver.h"
#include "NMPC_solver.h"
#include "Robot_Parameters.h"
#include "StateEstimator.h"
#include "SwingLegController.h"
#include "TrajectoryGenerator.h"
#include "UserCmd.h"
#include "WholeBodyController.h"
#include "common.h"
#include "test.h"
class Info {
 private:
  /* data */
 public:
  bool isPrintOutInfo;

  /**
   * @brief 打印消息
   * @param  vmc     虚拟模型控制器         
   * @param  Dy       动力学        
   * @param  FSM       状态机       
   * @param  Para      参数器       
   * @param  SE        状态估计器       
   * @param  SLC        摆动腿控制器      
   * @param  Trj        轨迹生成器      
   * @param  cmd         用户输入        
   * @param  WBC         全身控制器     
   * @param  Test          测试   
   */
  void PrintOutInfo(BallanceController& vmc, Robot_Dynamic& Dy,
                    FiniteStateMachine& FSM, Robot_Parameters Para,
                    StateEstimator& SE, SwingLegController& SLC,
                    TrajectoryGenerator& Trj, UserCmd& cmd,
                    WholeBodyController& WBC,
                    test& Test);
  Info(/* args */);
  ~Info();
};
