#include "kalman_filters/ekf.hpp"
#include <cmath>

// Constructor
ExtendedKalmanFilter::ExtendedKalmanFilter() {
    x_ = Eigen::VectorXd::Zero(6);                          // Estado: [x, y, z, theta, phi, v]
    P_ = Eigen::MatrixXd::Identity(6, 6);                   // Covarianza inicial
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;            // Ruido del proceso
    R_gps_ = Eigen::MatrixXd::Identity(3, 3) * 0.1;         // Ruido de medición GPS (ahora 3D)
    R_odom_ = Eigen::MatrixXd::Identity(3, 3) * 0.05;       // Ruido de medición Odometría (3D)
    I_ = Eigen::MatrixXd::Identity(6, 6);                   // Matriz identidad
}

// Inicializa el filtro con el estado inicial y la covarianza
void ExtendedKalmanFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
    x_ = x0;
    P_ = P0;
}

// Predicción del estado y covarianza para un modelo de dron en 3D
void ExtendedKalmanFilter::predict(double dt, const Eigen::Vector3d& omega) {
    double theta = x_(3);  // Ángulo de orientación en Z (yaw)
    double phi = x_(4);    // Ángulo de inclinación (pitch)
    double v = x_(5);      // Velocidad

    // Movimiento en 3D (modificado para el modelo de dron)
    Eigen::VectorXd x_pred = x_;
    x_pred(0) += v * cos(theta) * cos(phi) * dt;  // Movimiento en X
    x_pred(1) += v * sin(theta) * cos(phi) * dt;  // Movimiento en Y
    x_pred(2) += v * sin(phi) * dt;               // Movimiento en Z
    x_pred(3) += omega(0) * dt;                   // Cambio de orientación sobre el eje X (roll)
    x_pred(4) += omega(1) * dt;                   // Cambio de inclinación sobre el eje Y (pitch)
    x_pred(5) += omega(2) * dt;                   // Cambio en la velocidad

    // Jacobiano F: Derivado del modelo de movimiento no lineal
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 3) = -v * cos(theta) * sin(phi) * dt;  // Derivada parcial respecto a theta
    F(0, 4) = -v * sin(theta) * sin(phi) * dt;  // Derivada parcial respecto a phi
    F(1, 3) = v * sin(theta) * cos(phi) * dt;   // Derivada parcial respecto a theta
    F(1, 4) = -v * cos(theta) * cos(phi) * dt;  // Derivada parcial respecto a phi
    F(2, 4) = v * cos(phi) * dt;                // Derivada parcial respecto a phi
    F(5, 5) = 1;                                // Velocidad no cambia

    x_ = x_pred;
    P_ = F * P_ * F.transpose() + Q_;
}

// Actualiza el estado con la medición GPS (3D)
void ExtendedKalmanFilter::updateGPS(const Eigen::Vector3d& z) {
    Eigen::Vector3d z_pred = x_.segment<3>(0);  // h(x) = [x, y, z]

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 2) = 1;

    Eigen::Vector3d y = z - z_pred;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_gps_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (I_ - K * H) * P_;
}

// Actualiza el estado con la medición de odometría (3D)
void ExtendedKalmanFilter::updateOdom(const Eigen::Vector3d& v_measured) {
    Eigen::VectorXd z_pred = x_.segment<3>(0);  // h(x) = [v_x, v_y, v_z]

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H(0, 5) = 1;
    H(1, 5) = 1;
    H(2, 5) = 1;

    Eigen::Vector3d y = v_measured - z_pred;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_odom_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (I_ - K * H) * P_;
}
