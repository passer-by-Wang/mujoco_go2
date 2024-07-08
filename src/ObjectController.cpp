#include "../include/ObjectController.h"

ObjectController::ObjectController()
{
    object_k_p.setZero();
    object_k_v.setZero();
    object_k_theta.setZero();
    object_k_w.setZero();
    gravity << 0, 0, -9.8;
}

void ObjectController::computeObjectForces(const Matrix<double, robot_state_dim + object_state_dim, 1> &state,
                                           const Matrix<double, robot_state_dim + object_state_dim, 1> &state_d, const Vector3d &Object_to_Body_Pos)
{
    Object_Forces.head(3) = object_k_p * (state.block(16, 0, 3, 1) - state_d.block(16, 0, 3, 1)) +
                            object_k_v * (state.block(22, 0, 3, 1) - state_d.block(22, 0, 3, 1)) + object_mass * gravity;
    Object_Forces.tail(3) = object_k_theta * (state.block(13, 0, 3, 1) - state_d.block(13, 0, 3, 1)) +
                            object_k_w * (state.block(19, 0, 3, 1) - state_d.block(19, 0, 3, 1));

    Object_to_Body_Forces.head(3) = Object_Forces.head(3);
    Object_to_Body_Forces.tail(3) = Object_Forces.tail(3) + cross_product(Object_to_Body_Pos) * Object_Forces.head(3);

    Matrix<double, 6, 1> error = state_d.block(13, 0, 6, 1) - state.block(13, 0, 6, 1);
    // printinfo(error.transpose());
    // printinfo(state.block(13, 0, 6, 1).transpose());
    // printinfo(state_d.block(13, 0, 6, 1).transpose());
    // printinfo(object_k_p);
    // printinfo(object_k_v);
    // printinfo(object_k_theta);
    // printinfo(object_k_w);
    // printinfo(Object_Forces.transpose());
}

ObjectController::~ObjectController()
{
}