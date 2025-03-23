#pragma once

#include <ArduinoEigen.h>

using namespace Eigen;

class LQRController {
public:
    // 状态变量 [x dx theta dtheta phi dphi]
    volatile float state[6];
    const float stateDesire[6] = {0.f, 0.f, 1.f, 0.f, 0.f, 0.f};

    // QR 参数
    float Q_values[6] = {9.f, 5.f, 30.f, 0.001f, 0.6f, 0.2f};
    float R_value = 0.5f;

    // 系统矩阵
    Matrix<float, 6, 6> A;
    Matrix<float, 6, 2> B;
    Matrix<float, 2, 6> K;

    // 实际转速
    volatile float rpmLeftActual;
    volatile float rpmRightActual;

    LQRController();

    void initSystemMatrices();
    void computeLQR();
    Matrix<float, 2, 1> optimalControl();
};
