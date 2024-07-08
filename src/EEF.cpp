#include "../include/EEF.h"

/// @brief 
/// @param tor 外部扭矩
/// @param jf 雅可比矩阵
void EEF::EEF_from_dynamic(VectorXd &tor, Matrix<double, 6, 18> (&jf)[4],StateEstimator& SE)
{
    for (int i = 0; i < 4; i++)
    {
        jt[i] = (jf[i].transpose()).block<3, 3>(6 + 3 * i, 0); // 大的单腿雅可比拿到足底力与关节扭矩的映射
        jt_[i] = jt[i].inverse();                              // 雅可比矩阵的逆
        // cout << "jt:" << jt[i].transpose() << endl;
        te[i] = tor.block<3, 1>(6 + 3 * i, 0); // 四腿对应的广义扭矩
        ef[i] = jt_[i] * te[i];                // 反映射到足底力
        // cout << "ef[" << i << "]: " << ef[i].transpose() << endl;
        eefz(i) = ef[i](2); // 把第i条腿的z力给合成的z力eefz
        eefz_modify(i) = eefz(i);

        if (eefz_modify(i) <= -20)
        {
            eefz_modify(i) *= 0.001; // 不相信负值....
        }
    }
    SE.LegForce_z_w_estimate = eefz_modify;
    // 输出查看
    // cout << "force Z :" << eefz_modify << endl;
}

EEF::EEF() {}

EEF::~EEF() {}