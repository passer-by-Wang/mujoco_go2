/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     SaveLog.h
  @brief    SaveLog
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
#include "../include/SaveLog.h"
SaveLog::SaveLog(/* args */)
{
  VarCount = 0;
  RunCount = 0;
  isSaveLog = 0;
}

void SaveLog::SaveData(FiniteStateMachine &FSM, StateEstimator &SE,
                       TrajectoryGenerator &Trj,
                       SwingLegController &SLC, MPC_solver &MPC,
                       NMPC_solver &NMPC, BallanceController &VMC,
                       WholeBodyController &WBC, Robot_Dynamic &Dyn,EEF &eef)
{
  append(SE.body_CoMPos,"SE.body_CoMPos");              // 0    3 个   状态估计的 质心位置
  append(SE.body_CoMVel,"SE.body_CoMVel");              // 3    3 个   状态估计的 质心速度
  append(SE.body_CoMAngle,"SE.body_CoMAngle");            // 6    3 个   imu获取的 质心角度
  append(SE.body_CoMAngularVel_w,"SE.body_CoMAngularVel_w");     // 9    3 个   imu获取的 质心角速度
  append(SE.CoMVeloSim,"SE.CoMVeloSim");               // 12   3 个   仿真环境获取的 质心速度
  append(Trj.ToePos_bH_d,"Trj.ToePos_bH_d");             // 15   12个   期望的 足端位置
  append(SE.ToePos_wB,"SE.ToePos_wB");                // 27   12个   状态估计的 足端位置
  append(VMC.ContactLegForces,"VMC.ContactLegForces");        // 39   12个   VMC Legforces
  append(Trj.body_CoMAngle_d,"Trj.body_CoMAngle_d");         // 51     3个     期望的 质心角度
  append(Trj.body_CoMPos_d,"Trj.body_CoMPos_d");           // 54     3个     期望的 质心角度
  append(Trj.body_CoMAngleVel_d,"Trj.body_CoMAngleVel_d");      // 57     3个     期望的 质心角度
  append(Trj.body_CoMVel_d,"Trj.body_CoMVel_d");           // 60     3个     期望的 质心角度
  append(SE.ToePos_bH,"SE.ToePos_bH");                // 63   12个   期望的 足端位置
  append(SE.body_CoM_Quaternion,"SE.body_CoM_Quaternion");      // 75  4 状态质心四元数
  append(Trj.body_CoM_Quaternion_d,"Trj.body_CoM_Quaternion_d");   // 79  4   期望的质心四元数
  append(SE.object_CoM_Quaternion,"SE.object_CoM_Quaternion");    // 83  4 状态质心四元数
  append(Trj.object_CoM_Quaternion_d,"Trj.object_CoM_Quaternion_d"); // 87  4   期望的质心四元数
  append(WBC.LegForces,"WBC.LegForces");           // 91  12  使用的足端力
  append(Trj.ObjectPos_d,"Trj.ObjectPos_d");             // 103  3
  append(Trj.ObjectAngle_d,"Trj.ObjectAngle_d");           // 106  3
  append(SE.object_CoMPos,"SE.object_CoMPos");            // 109  3
  append(SE.object_CoMAngle,"SE.object_CoMAngle");          // 112  3
  append(SE.RealLegContactForces_w.col(2),"SE.RealLegContactForces_w.col(2)"); ///115 4
  append(eef.eefz_modify,"eef.eefz_modify");    //119 4
  append(FSM.LegPhase,"FSM.LegPhase");     //123 4
  append(Trj.mpc_state_d.head(13),"Trj.mpc_state_d.head(13)"); //127 13
  append(Trj.nmpc_state_d,"Trj.nmpc_state_d");   //140 13
  append(SE.body_joint_angle,"SE.body_joint_angle");    //153 12
  append(Trj.body_joint_angle_d,"Trj.body_joint_angle_d");   //165  12
  VarCount = 0;
  RunCount++;
}

void SaveLog::SaveData2Log()
{
  string fileName =
      "/home/wh/simulation/Legged_robot/log/"; // 保存路径,新配置需要修改
  ofstream fout;
  fileName += getTime(); // 以时间命名文件csv文件
  fileName += ".csv";
  fout.open(fileName, ios::out);

  if (RunCount > RunNum)
    RunCount = RunNum;
  for (int j = 0; j < VarNum; j++)
  {
    if (!LogDataNameBuff[j].empty())
      fout << LogDataNameBuff[j].at(0) << ","; // 从数据缓存中输出数据到csv文件
  }
  fout << endl;
  for (int i = 0; i < RunCount; i++)
  {
    for (int j = 0; j < VarNum; j++)
    {
      if (!LogDataBuff[j].empty())
        fout << LogDataBuff[j].at(i) << ","; // 从数据缓存中输出数据到csv文件
    }
    fout << endl;
  }
  cout << "[Save log] file writed" << endl;
  fout.close();
  cout << "[Save log] " << fileName << endl;
}

// 模板
template <typename T>
void SaveLog::append(const DenseBase<T> &mat,const string Name) // 用于矩阵数据
{
  for (int i = 0; i < mat.rows(); i++)
    for (int j = 0; j < mat.cols(); j++)
    {
      if (RunCount == 0)
      {
        LogDataNameBuff[VarCount].push_back(Name + "_" + std::to_string(i + 1) + "_" + std::to_string(j + 1));

        if (RunCount > RunNum)
          LogDataNameBuff[VarCount].pop_front();
      }
      LogDataBuff[VarCount].push_back(
          mat(i, j)); // push_back:将一个新的元素加到最后面

      if (RunCount > RunNum)
        LogDataBuff[VarCount]
            .pop_front(); // pop_front()用于从列表容器的开头删除元素。因此，此函数将容器的大小减小1，因为它从列表的开头删除了元素

      VarCount++;
    }
}
template <typename T>
void SaveLog::append_c(const T &var,const string Name) // 用于单个变量的数据
{
  if (RunCount == 0)
      {
        LogDataNameBuff[VarCount].push_back(Name);

        if (RunCount > RunNum)
          LogDataNameBuff[VarCount].pop_front();
      }
  LogDataBuff[VarCount].push_back(var);

  if (RunCount > RunNum)
    LogDataBuff[VarCount].pop_front();

  VarCount++;
}

SaveLog::~SaveLog()
{

}