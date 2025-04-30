#include "kalman_filters/ekf.hpp"
#include <cmath>

// Constructor
ExtendedKalmanFilter::ExtendedKalmanFilter() {
    x_ = Eigen::VectorXd::Zero(6);                          // Estado: [x, y, z, theta, phi, v]
    P_ = Eigen::MatrixXd::Identity(6, 6);                   // Covarianza inicial
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.2;             // Ruido del proceso
    R_gps_ = Eigen::MatrixXd::Identity(3, 3) * 0.1;         // Ruido de medición GPS (3D)
    R_odom_ = Eigen::MatrixXd::Identity(3, 3) * 0.1;        // Ruido de medición Odometría (3D)
    I_ = Eigen::MatrixXd::Identity(6, 6);                   // Matriz identidad
}

// Inicializa el filtro con el estado inicial y la covarianza
void ExtendedKalmanFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
    x_ = x0;
    P_ = P0;
}

// Predicción del estado y covarianza para un modelo de dron en 3D
void ExtendedKalmanFilter::predict(double dt, const Eigen::Vector3d& omega) {
    double theta = x_(3);  // Ángulo yaw
    double phi = x_(4);    // Ángulo pitch
    double v = x_(5);      // Velocidad escalar

    // Predicción del estado
    Eigen::VectorXd x_pred = x_;
    x_pred(0) += v * cos(theta) * cos(phi) * dt;
    x_pred(1) += v * sin(theta) * cos(phi) * dt;
    x_pred(2) += v * sin(phi) * dt;
    x_pred(3) += omega(0) * dt;
    x_pred(4) += omega(1) * dt;
    x_pred(5) += omega(2) * dt;

    // Jacobiano F
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 3) = -v * sin(theta) * cos(phi) * dt;
    F(0, 4) = -v * cos(theta) * sin(phi) * dt;
    F(0, 5) = cos(theta) * cos(phi) * dt;

    F(1, 3) = v * cos(theta) * cos(phi) * dt;
    F(1, 4) = -v * sin(theta) * sin(phi) * dt;
    F(1, 5) = sin(theta) * cos(phi) * dt;

    F(2, 4) = v * cos(phi) * dt;
    F(2, 5) = sin(phi) * dt;

    x_ = x_pred;
    P_ = F * P_ * F.transpose() + Q_;
}

// Actualiza el estado con la medición GPS (posición XYZ)
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

// Actualiza el estado con la medición de odometría (velocidad XYZ)
void ExtendedKalmanFilter::updateOdom(const Eigen::Vector3d& v_measured) {
    double theta = x_(3);
    double phi = x_(4);
    double v = x_(5);

    // Velocidad proyectada según modelo
    Eigen::Vector3d v_pred;
    v_pred(0) = v * cos(theta) * cos(phi);
    v_pred(1) = v * sin(theta) * cos(phi);
    v_pred(2) = v * sin(phi);

    // Jacobiano respecto a v
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H(0, 5) = cos(theta) * cos(phi);
    H(1, 5) = sin(theta) * cos(phi);
    H(2, 5) = sin(phi);

    Eigen::Vector3d y = v_measured - v_pred;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_odom_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (I_ - K * H) * P_;
}
