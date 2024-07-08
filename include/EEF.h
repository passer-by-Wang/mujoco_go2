#pragma once
#include <iostream>
#include "Eigen/Dense"
#include "eigen3/unsupported/Eigen/MatrixFunctions" //inverse求逆的头
#include "common.h"
#include "StateEstimator.h"
using namespace Eigen;
using namespace std;
class EEF
{
private:
    /* data */
public:
    EEF(/* args */);
    ~EEF();
    Matrix3d jt[4];                         // 四腿的雅可比，只提取出来了单腿关节到足端的三维映射
    Matrix3d jt_[4];                        // jt 的逆，用来反映射
    Matrix<double, 3, 1> te[4], ef[4];      // te是广义扭矩，ef是估计都外部足端力
    Matrix<double, 4, 1> eefz, eefz_modify; // 估计的外部z方向的足底力，把负值给不相信了
    
    /// @brief 估计z方向足底力
    /// @param tor 外部扭矩
    /// @param jf 雅可比矩阵
    void EEF_from_dynamic(VectorXd &tor, Matrix<double, 6, 18> (&jf)[4],StateEstimator& SE);
};
