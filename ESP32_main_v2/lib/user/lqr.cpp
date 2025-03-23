#include "lqr.h"

LQRController::LQRController() {
    for (int i = 0; i < 6; i++) state[i] = 0.f;
    rpmLeftActual = 0.f;
    rpmRightActual = 0.f;
    initSystemMatrices();
}

void LQRController::initSystemMatrices() {
    A << 0, 1, 0, 0, 0, 0,
         0, -0.008f, 0.6086f, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 2.0834f, -0.6260f, 0, 0, 0,
         0, 0, 0, 0, 0, 1,
         0, 0, 0, 0, 0, 0;

    B << 0, 0,
         1.4267f, 1.4267f,
         0, 0,
         -33.8928f, -33.8928f,
         0, 0,
         -118.4385f, 118.4385f;
}

void LQRController::computeLQR() {
    Matrix<float, 6, 6> Q = Matrix<float, 6, 6>::Zero();
    Matrix<float, 2, 2> R = Matrix<float, 2, 2>::Zero();
    for (int i = 0; i < 6; i++) Q(i, i) = Q_values[i];
    R(0, 0) = R_value;
    R(1, 1) = R_value;

    Matrix<float, 6, 6> P = Q;
    Matrix<float, 6, 6> AT = A.transpose();
    Matrix<float, 2, 6> BT = B.transpose();

    float tolerance = 0.0001f;

    // Riccati 方程迭代求解, 降低计算复杂度
    for (int i = 0; i < 100; i++) {
        Matrix<float, 2, 2> RB = R + BT * P * B;
        Eigen::LDLT<Matrix<float, 2, 2>> ldlt(RB);
        Matrix<float, 2, 6> K_temp = ldlt.solve(BT * P * A);
        Matrix<float, 6, 6> P_next = Q + AT * P * A - (AT * P * B) * K_temp;

        if ((P - P_next).norm() < tolerance) {
            P = P_next;
            break;
        }
        P = P_next;
    }

    Eigen::LDLT<Matrix<float, 2, 2>> ldlt_final(R + BT * P * B);
    K = ldlt_final.solve(BT * P * A);
}

Matrix<float, 2, 1> LQRController::optimalControl() {
    Matrix<float, 6, 1> stateVec;
    for (int i = 0; i < 6; i++) stateVec(i, 0) = state[i];
    Matrix<float, 6, 1> stateError = stateVec - Map<const Matrix<float, 6, 1>>(stateDesire);
    Matrix<float, 2, 1> u = -K * stateError;
    return u;
}
