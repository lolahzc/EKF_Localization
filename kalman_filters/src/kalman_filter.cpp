#include "kalman_filters/ekf.hpp"
#include <iostream>

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    x_ = Eigen::Vector4d::Zero();
    P_ = Eigen::Matrix4d::Identity() * 0.1;
    F_ = Eigen::Matrix4d::Identity();
    Q_ = Eigen::Matrix4d::Identity();
    H_ << 1, 0, 0, 0,
           0, 1, 0, 0;
    set_process_noise(pos_process_noise_, vel_process_noise_);
}

void ExtendedKalmanFilter::initialize(double x0, double y0) {
    x_ << x0, y0, 0, 0;
}

void ExtendedKalmanFilter::set_gps_noise(double noise) {
    gps_noise_ = noise;
}

void ExtendedKalmanFilter::set_process_noise(double pos_noise, double vel_noise) {
    pos_process_noise_ = pos_noise;
    vel_process_noise_ = vel_noise;
    Q_.diagonal() << pos_process_noise_, pos_process_noise_, vel_process_noise_, vel_process_noise_;
}

void ExtendedKalmanFilter::predict(double vx, double vy, double dt) {
    // Actualizar velocidades
    x_(2) = vx;
    x_(3) = vy;
    
    // Actualizar matriz de transición
    F_(0, 2) = dt;
    F_(1, 3) = dt;
    
    // Predicción del estado
    x_ = F_ * x_;
    
    // Actualizar covarianza
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void ExtendedKalmanFilter::update_gps(double x, double y) {
    Eigen::Vector2d z(x, y);
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * gps_noise_;
    
    Eigen::MatrixXd K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R).inverse();
    
    x_ += K * (z - H_ * x_);
    P_ = (Eigen::Matrix4d::Identity() - K * H_) * P_;
}