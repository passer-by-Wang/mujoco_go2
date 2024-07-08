/*****************************************************************************
  BQR3 simulation
  Copyright (C) 2023 Hua Wang  wangh@bit.edu.cn.
  This file is part of BQR3.
  @file     test.cpp
  @brief    test
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

#include "../include/test.h"
test::test() {}
void test::abc() {
  A.setRandom();
  B.setZero();
  Matrix<double, 3, 1> aa;

  // printinfo(A.col(0));
  B.block(0, 0, 1, 3) = A.col(0);
  B.block<1, 3>(0, 0) = A.col(0);

  // printinfo(B.row(0));
};

test::~test() {}