/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     WholeBodyPlanner.cpp
  @brief    WholeBodyPlanner
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
  2024/3/4   | 1.0.0     | Hua Wang       | Create file
-----------------------------------------------------------------------------

*****************************************************************************/

#include "../include/WholeBodyPlanner.h"

WholeBodyPlanner::WholeBodyPlanner(/* args */)
{
  isCreatedPlanner = false;
  iter_num = 0;
  p_body << 0.01, 0 - 0.01, 0.52;
  Q_body.setZero();
  p_arm << 0.2, 0.0, 0.3;
  Q_arm.setZero();
  const_height = 0.55;

  p_arm_w_d << 1.2, 1.5, 0.7;
  Q_arm_w_d << 0.5, 0.7, 1.2;

  weight_p_body << 100, 100, 100;
  weight_Q_body << 10, 10, 10;
  weight_p_arm << 1000, 1000, 1000;
  weight_Q_arm << 100, 100, 100;
}

void WholeBodyPlanner::config_target_state(const Vector3d &_p_arm_wd, const Vector3d &_Q_arm_wd, const Vector3d &_p_body, 
const Vector3d &_Q_body, const Vector3d &_p_arm, const Vector3d &_Q__arm) {
  p_body = _p_body;
  Q_body = _Q_body;
  p_arm = _p_arm;
  Q_arm = _Q__arm;
  p_arm_w_d = _p_arm_wd;
  Q_arm_w_d = _Q_arm_wd;
  const_height = _p_body[2];
}

void WholeBodyPlanner::create_QCP_Planner(){

  //精度、容差
  tol=1e-10;

  //优化变量维度
  number_variables=12;

  //下边界；
  lb[0] = -INF; lb[1] = -INF; lb[2] = const_height;lb[3] = 0; lb[4] = 0; lb[5] = -pi;
  lb[6] = -INF; lb[7] = -INF; lb[8] = -INF;lb[9] = Q_arm_w_d[0]; lb[10] = Q_arm_w_d[1]; lb[11] = -INF;
  
  //上边界
  ub[0] = INF; ub[1] = INF; ub[2] = const_height;ub[3] = 0; ub[4] = 0; ub[5] = pi;
  ub[6] = INF; ub[7] = INF; ub[8] = INF;ub[9] = Q_arm_w_d[0]; ub[10] = Q_arm_w_d[1]; ub[11] = INF;
  
  //赋予初始值；
  x[0] = 0; x[1] = 0; x[2] = const_height;x[3] = 0.0; x[4] = 0.0; x[5] = 0.0;
  x[6] = 0; x[7] = 0; x[8] = 0.0;x[9] = Q_arm_w_d[0]; x[10] = Q_arm_w_d[1]; x[11] = 0.0;

  //创建优化问题
  opter=nlopt_create(NLOPT_LD_SLSQP, number_variables);

  isCreatedPlanner = true;

  //设置自变量下限；
  nlopt_set_lower_bounds(opter, lb);

  //设置自变量上限；
  nlopt_set_upper_bounds(opter, ub);

  // 目标函数；
  nlopt_set_min_objective(opter,objectFunction, this);

  // 等式约束
  nlopt_add_equality_constraint(opter, equalityConstraints_pos_x, this, tol);
  nlopt_add_equality_constraint(opter, equalityConstraints_pos_y, this, tol);
  nlopt_add_equality_constraint(opter, equalityConstraints_pos_z, this, tol);
  nlopt_add_equality_constraint(opter, equalityConstraints_orien_x, this, tol);
  nlopt_add_equality_constraint(opter, equalityConstraints_orien_y, this, tol);
  nlopt_add_equality_constraint(opter, equalityConstraints_orien_z, this, tol);

  //不等式约束
  nlopt_add_inequality_constraint(opter, inequalityConstraints_arm_upper, this, tol);
  nlopt_add_inequality_constraint(opter, inequalityConstraints_arm_lower, this, tol);

  //设置停止条件
  nlopt_set_ftol_rel(opter, tol);
  nlopt_set_xtol_rel(opter, tol);
  nlopt_set_force_stop(opter, tol);
}

bool WholeBodyPlanner::opp_solve(){

  create_QCP_Planner();

  nlopt_result result = nlopt_optimize(opter, x, &cost);

  if(result){
    printf("迭代次数 i= %d,目标函数值=%g,x={%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g}\n", iter_num,cost, x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10], x[11]);
    printf("cost_of_equalityConstraints_pos_x=%g\n", x[0]+x[6]-p_arm_w_d[0]);
    printf("cost_of_equalityConstraints_pos_y=%g\n", x[1]+x[7]-p_arm_w_d[1]);
    printf("cost_of_equalityConstraints_pos_z=%g\n", x[2]+x[8]-p_arm_w_d[2]);
    printf("cost_of_equalityConstraints_orien_x=%g\n", x[3]+x[9]-Q_arm_w_d[0]);
    printf("cost_of_equalityConstraints_orien_y=%g\n", x[4]+x[10]-Q_arm_w_d[1]);
    printf("cost_of_equalityConstraints_orien_z=%g\n", x[5]+x[11]-Q_arm_w_d[2]);
    if(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]-1.0>0.001){
      printf("cost_of_inequalityConstraints_arm_upper=%g\n", x[3]*x[3]+x[4]*x[4]+x[5]*x[5]-1.0);
    }else{
      printf("cost_of_inequalityConstraints_arm_upper=%g\n", 0.0);
    }

    if(-x[3]*x[3]-x[4]*x[4]-x[5]*x[5]+0.25>0.001){
      printf("cost_of_inequalityConstraints_arm_lower=%g\n", -x[3]*x[3]-x[4]*x[4]-x[5]*x[5]+0.25);
    }else{
      printf("cost_of_inequalityConstraints_arm_lower=%g\n", 0.0);
    }
  }

  return result;
}

double WholeBodyPlanner::objectFunction(unsigned n, const double *x, double *grad, void *data)
{
  WholeBodyPlanner* self = static_cast<WholeBodyPlanner*>(data);
  if (grad)
  {
    grad[0] = self->weight_p_body[0]*(x[0]-self->p_body[0]);
    grad[1] = self->weight_p_body[1]*(x[1]-self->p_body[1]);
    grad[2] = self->weight_p_body[2]*(x[2]-self->p_body[2]);
    grad[3] = self->weight_Q_body[0]*(x[3]-self->Q_body[0]);
    grad[4] = self->weight_Q_body[1]*(x[4]-self->Q_body[1]);
    grad[5] = self->weight_Q_body[2]*(x[5]-self->Q_body[2]);
    grad[6] = self->weight_p_arm[0]*(x[6]-self->p_arm[0]);
    grad[7] = self->weight_p_arm[1]*(x[7]-self->p_arm[1]);
    grad[8] = self->weight_p_arm[2]*(x[8]-self->p_arm[2]);
    grad[9] = self->weight_Q_arm[0]*(x[9]-self->Q_arm[0]);
    grad[10] = self->weight_Q_arm[1]*(x[10]-self->Q_arm[1]);
    grad[11] = self->weight_Q_arm[2]*(x[11]-self->Q_arm[2]);
  }
  self->iter_num++;
  // printf("迭代次数 i= %d, x[0]=%f, x[1]= %f，x[2]= %f，x[3]= %f，，x[4]= %f，x[5]= %f，f(x1,x2,x3,x4,x5,x6)=%f\n",i++,x[0],x[1],x[2],x[3],x[4],x[5],10*(x[0]*x[0]+x[1]*x[1]+x[2]*x[2])+(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]));
  return 0.5*(self->weight_p_body[0]*(x[0]-self->p_body[0])*(x[0]-self->p_body[0])+self->weight_p_body[1]*(x[1]-self->p_body[1])*(x[1]-self->p_body[1])+self->weight_p_body[2]*(x[2]-self->p_body[2])*(x[2]-self->p_body[2])\
            + self->weight_Q_body[0]*(x[3]-self->Q_body[0])*(x[3]-self->Q_body[0])+ self->weight_Q_body[1]*(x[4]-self->Q_body[1])*(x[4]-self->Q_body[1])+ self->weight_Q_body[2]*(x[5]-self->Q_body[2])*(x[5]-self->Q_body[2])\
            +self->weight_p_arm[0]*(x[6]-self->p_arm[0])*(x[6]-self->p_arm[0])+self->weight_p_arm[1]*(x[7]-self->p_arm[1])*(x[7]-self->p_arm[1])+self->weight_p_arm[2]*(x[8]-self->p_arm[2])*(x[8]-self->p_arm[2])\
            + self->weight_Q_arm[0]*(x[9]-self->Q_arm[0])*(x[9]-self->Q_arm[0])+ self->weight_Q_arm[1]*(x[10]-self->Q_arm[1])*(x[10]-self->Q_arm[1])+ self->weight_Q_arm[2]*(x[11]-self->Q_arm[2])*(x[11]-self->Q_arm[2]));
}

double WholeBodyPlanner::equalityConstraints_pos_x(unsigned n, const double *x, double *grad, void *data){
  WholeBodyPlanner* self = static_cast<WholeBodyPlanner*>(data);
  if (grad)
  {
    grad[0] = 1;
    grad[1] = 0;
    grad[2] = 0;
    grad[3] = 0;
    grad[4] = 0;
    grad[5] = 0;
    grad[6] = 1;
    grad[7] = 0;
    grad[8] = 0;
    grad[9] = 0;
    grad[10] = 0;
    grad[11] = 0;
  }
  // printf("迭代次数 i= %d, x[0]=%f, x[1]= %f，x[2]= %f，x[3]= %f，，x[4]= %f，x[5]= %f，f(x1,x2,x3,x4,x5,x6)=%f\n",i++,x[0],x[1],x[2],x[3],x[4],x[5],10*(x[0]*x[0]+x[1]*x[1]+x[2]*x[2])+(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]));
  return x[0]+x[6]-self->p_arm_w_d[0];
}

double WholeBodyPlanner::equalityConstraints_pos_y(unsigned n, const double *x, double *grad, void *data){
  WholeBodyPlanner* self = static_cast<WholeBodyPlanner*>(data);
  if (grad)
  {
    grad[0] = 0;
    grad[1] = 1;
    grad[2] = 0;
    grad[3] = 0;
    grad[4] = 0;
    grad[5] = 0;
    grad[6] = 0;
    grad[7] = 1;
    grad[8] = 0;
    grad[9] = 0;
    grad[10] = 0;
    grad[11] = 0;
  }
  // printf("迭代次数 i= %d, x[0]=%f, x[1]= %f，x[2]= %f，x[3]= %f，，x[4]= %f，x[5]= %f，f(x1,x2,x3,x4,x5,x6)=%f\n",i++,x[0],x[1],x[2],x[3],x[4],x[5],10*(x[0]*x[0]+x[1]*x[1]+x[2]*x[2])+(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]));
  return x[1]+x[7]-self->p_arm_w_d[1];
}

double WholeBodyPlanner::equalityConstraints_pos_z(unsigned n, const double *x, double *grad, void *data){
  WholeBodyPlanner* self = static_cast<WholeBodyPlanner*>(data);
  if (grad)
  {
    grad[0] = 0;
    grad[1] = 0;
    grad[2] = 1;
    grad[3] = 0;
    grad[4] = 0;
    grad[5] = 0;
    grad[6] = 0;
    grad[7] = 0;
    grad[8] = 1;
    grad[9] = 0;
    grad[10] = 0;
    grad[11] = 0;
  }
  // printf("迭代次数 i= %d, x[0]=%f, x[1]= %f，x[2]= %f，x[3]= %f，，x[4]= %f，x[5]= %f，f(x1,x2,x3,x4,x5,x6)=%f\n",i++,x[0],x[1],x[2],x[3],x[4],x[5],10*(x[0]*x[0]+x[1]*x[1]+x[2]*x[2])+(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]));
  return x[2]+x[8]-self->p_arm_w_d[2];
}

double WholeBodyPlanner::equalityConstraints_orien_x(unsigned n, const double *x, double *grad, void *data){
  WholeBodyPlanner* self = static_cast<WholeBodyPlanner*>(data);
  if (grad)
  {
    grad[0] = 0;
    grad[1] = 0;
    grad[2] = 0;
    grad[3] = 1;
    grad[4] = 0;
    grad[5] = 0;
    grad[6] = 0;
    grad[7] = 0;
    grad[8] = 0;
    grad[9] = 1;
    grad[10] = 0;
    grad[11] = 0;
  }
  // printf("迭代次数 i= %d, x[0]=%f, x[1]= %f，x[2]= %f，x[3]= %f，，x[4]= %f，x[5]= %f，f(x1,x2,x3,x4,x5,x6)=%f\n",i++,x[0],x[1],x[2],x[3],x[4],x[5],10*(x[0]*x[0]+x[1]*x[1]+x[2]*x[2])+(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]));
  return x[3]+x[9]-self->Q_arm_w_d[0];
}

double WholeBodyPlanner::equalityConstraints_orien_y(unsigned n, const double *x, double *grad, void *data){
  WholeBodyPlanner* self = static_cast<WholeBodyPlanner*>(data);
  if (grad)
  {
    grad[0] = 0;
    grad[1] = 0;
    grad[2] = 0;
    grad[3] = 0;
    grad[4] = 1;
    grad[5] = 0;
    grad[6] = 0;
    grad[7] = 0;
    grad[8] = 0;
    grad[9] = 0;
    grad[10] = 1;
    grad[11] = 0;
  }
  // printf("迭代次数 i= %d, x[0]=%f, x[1]= %f，x[2]= %f，x[3]= %f，，x[4]= %f，x[5]= %f，f(x1,x2,x3,x4,x5,x6)=%f\n",i++,x[0],x[1],x[2],x[3],x[4],x[5],10*(x[0]*x[0]+x[1]*x[1]+x[2]*x[2])+(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]));
  return x[4]+x[10]-self->Q_arm_w_d[1];
}

double WholeBodyPlanner::equalityConstraints_orien_z(unsigned n, const double *x, double *grad, void *data){
  WholeBodyPlanner* self = static_cast<WholeBodyPlanner*>(data);
  if (grad)
  {
    grad[0] = 0;
    grad[1] = 0;
    grad[2] = 0;
    grad[3] = 0;
    grad[4] = 0;
    grad[5] = 1;
    grad[6] = 0;
    grad[7] = 0;
    grad[8] = 0;
    grad[9] = 0;
    grad[10] = 0;
    grad[11] = 1;
  }
  // printf("迭代次数 i= %d, x[0]=%f, x[1]= %f，x[2]= %f，x[3]= %f，，x[4]= %f，x[5]= %f，f(x1,x2,x3,x4,x5,x6)=%f\n",i++,x[0],x[1],x[2],x[3],x[4],x[5],10*(x[0]*x[0]+x[1]*x[1]+x[2]*x[2])+(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]));
  return x[5]+x[11]-self->Q_arm_w_d[2];
}

double WholeBodyPlanner::inequalityConstraints_arm_upper(unsigned n, const double *x, double *grad, void *data){
  WholeBodyPlanner* self = static_cast<WholeBodyPlanner*>(data);
  if (grad)
  {
    grad[0] = 0;
    grad[1] = 0;
    grad[2] = 0;
    grad[3] = 2*x[3];
    grad[4] = 2*x[4];
    grad[5] = 2*x[5];
    grad[6] = 0;
    grad[7] = 0;
    grad[8] = 0;
    grad[9] = 0;
    grad[10] = 0;
    grad[11] = 0;
  }
  // printf("迭代次数 i= %d, x[0]=%f, x[1]= %f，x[2]= %f，x[3]= %f，，x[4]= %f，x[5]= %f，f(x1,x2,x3,x4,x5,x6)=%f\n",i++,x[0],x[1],x[2],x[3],x[4],x[5],10*(x[0]*x[0]+x[1]*x[1]+x[2]*x[2])+(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]));
  return x[3]*x[3]+x[4]*x[4]+x[5]*x[5]-1.0;
}

double WholeBodyPlanner::inequalityConstraints_arm_lower(unsigned n, const double *x, double *grad, void *data){
  WholeBodyPlanner* self = static_cast<WholeBodyPlanner*>(data);
  if (grad)
  {
    grad[0] = 0;
    grad[1] = 0;
    grad[2] = 0;
    grad[3] = -2*x[3];
    grad[4] = -2*x[4];
    grad[5] = -2*x[5];
    grad[6] = 0;
    grad[7] = 0;
    grad[8] = 0;
    grad[9] = 0;
    grad[10] = 0;
    grad[11] = 1;
  }
  // printf("迭代次数 i= %d, x[0]=%f, x[1]= %f，x[2]= %f，x[3]= %f，，x[4]= %f，x[5]= %f，f(x1,x2,x3,x4,x5,x6)=%f\n",i++,x[0],x[1],x[2],x[3],x[4],x[5],10*(x[0]*x[0]+x[1]*x[1]+x[2]*x[2])+(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]));
  return -x[3]*x[3]-x[4]*x[4]-x[5]*x[5]+0.25;
}

WholeBodyPlanner::~WholeBodyPlanner() {
  if (isCreatedPlanner)
  {
    nlopt_destroy(opter);
  }
  else
  {
    std::cerr << "nlopt_opt object is not created." << std::endl;
  }
}