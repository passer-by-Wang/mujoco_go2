/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     HoQp.h
  @brief    HoQp for wbc
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
  2024/3/9   | 1.0.0     | Hua Wang       | Create file
-----------------------------------------------------------------------------

*****************************************************************************/
//
// Ref: https://github.com/qiayuanl/legged_control
//

#pragma once

#include <memory>
#include <qpOASES.hpp>
#include <utility>

#include "Task.h"
#include "common.h"

// Hierarchical Optimization Quadratic Program
class HoQp {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using HoQpPtr = std::shared_ptr<HoQp>;

  Task task_, stackedTasksPrev_, stackedTasks_;
  HoQpPtr higherProblem_;

  bool hasEqConstraints_{}, hasIneqConstraints_{};
  size_t numSlackVars_{}, numDecisionVars_{};
  matrix_t stackedZPrev_, stackedZ_;
  vector_t stackedSlackSolutionsPrev_, xPrev_;
  Matrix<double, 30, 1> init_input_;
  size_t numPrevSlackVars_{};

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, RowMajor> h_, d_;
  vector_t c_, f_;
  vector_t stackedSlackVars_, slackVarsSolutions_, decisionVarsSolutions_;

  // Convenience matrices that are used multiple times
  matrix_t eyeNvNv_;
  matrix_t zeroNvNx_;

  explicit HoQp(const Task& task) : HoQp(task, nullptr){};

  HoQp(Task task, HoQpPtr higherProblem);

  HoQp(Task task, HoQpPtr higherProblem, Matrix<double, 30, 1> init_input);

  matrix_t getStackedZMatrix() const { return stackedZ_; }

  Task getStackedTasks() const { return stackedTasks_; }

  vector_t getStackedSlackSolutions() const { return stackedSlackVars_; }

  vector_t getSolutions() const {
    vector_t x = xPrev_ + stackedZPrev_ * decisionVarsSolutions_;
    return x;
  }

  size_t getSlackedNumVars() const { return stackedTasks_.d_.rows(); }

 private:
  void initVars();
  void formulateProblem();
  void solveProblem();

  void buildHMatrix();
  void buildCVector();
  void buildDMatrix();
  void buildFVector();

  void buildZMatrix();
  void stackSlackSolutions();
};
