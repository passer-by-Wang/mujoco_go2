#include "../include/Info.h"
Info::Info(/* args */) {}

void Info::PrintOutInfo(BallanceController &vmc, Robot_Dynamic &Dy,
                        FiniteStateMachine &FSM, Robot_Parameters Para,
                        StateEstimator &SE, SwingLegController &SLC,
                        TrajectoryGenerator &Trj, UserCmd &cmd,
                        WholeBodyController &WBC,
                        test &Test)
{
  if (isPrintOutInfo)
  {
    printinfo(SE.RotationMatrix*SE.Jacobian[0]);
  }
}

Info::~Info() {}