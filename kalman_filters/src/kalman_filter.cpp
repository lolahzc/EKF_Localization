#include "kalman_filters/ekf.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    x_ = Eigen::VectorXd::Zero(6);
    P_ = Eigen::MatrixXd::Identity(6, 6);
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    R_gps_ = Eigen::MatrixXd::Identity(3, 3) * 0.1;
    R_odom_ = Eigen::MatrixXd::Identity(3, 3) * 0.01;
    I_ = Eigen::MatrixXd::Identity(6, 6);
}

void ExtendedKalmanFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
    x_ = x0;
    P_ = P0;
}

void ExtendedKalmanFilter::predict(double dt) {
    // Modelo no lineal: integración de velocidad
    Eigen::VectorXd x_pred = x_;
    x_pred(0) += x_(3) * dt; // x += vx * dt
    x_pred(1) += x_(4) * dt;
    x_pred(2) += x_(5) * dt;

    // Jacobiano del modelo de transición (F)
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;

    x_ = x_pred;
    P_ = F * P_ * F.transpose() + Q_;
}

void ExtendedKalmanFilter::updateGPS(const Eigen::Vector3d& z) {
    // Modelo de medición GPS: mide posición (no lineal trivial)
    Eigen::Vector3d z_pred = x_.segment<3>(0); // h(x)

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

void ExtendedKalmanFilter::updateOdom(const Eigen::Vector3d& z) {
    // Modelo de medición odom: mide velocidad (vx, vy, vz)
    Eigen::Vector3d z_pred = x_.segment<3>(3); // h(x)

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H(0, 3) = 1;
    H(1, 4) = 1;
    H(2, 5) = 1;

    Eigen::Vector3d y = z - z_pred;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_odom_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (I_ - K * H) * P_;
}
