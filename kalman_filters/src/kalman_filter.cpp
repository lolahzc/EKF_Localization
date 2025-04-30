#include "kalman_filters/ekf.hpp"
#include <iostream>

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    x_ = Eigen::Matrix<double, 6, 1>::Zero();
    P_ = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
    F_ = Eigen::Matrix<double, 6, 6>::Identity();
    Q_ = Eigen::Matrix<double, 6, 6>::Identity();
    H_ <<  1, 0, 0, 0, 0, 0,
           0, 1, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0;
    set_process_noise(pos_process_noise_, vel_process_noise_);
}

void ExtendedKalmanFilter::initialize(double x0, double y0, double z0) {
    x_ << x0, y0, z0, 0, 0, 0;
}

void ExtendedKalmanFilter::set_gps_noise(double noise) {
    gps_noise_ = noise;
}

void ExtendedKalmanFilter::set_process_noise(double pos_noise, double vel_noise) {
    pos_process_noise_ = pos_noise;
    vel_process_noise_ = vel_noise;
    Q_.diagonal() << pos_process_noise_, pos_process_noise_, pos_process_noise_,
                     vel_process_noise_, vel_process_noise_, vel_process_noise_;
}

void ExtendedKalmanFilter::predict(double vx, double vy, double vz, double dt) {
    // Actualizar velocidades
    x_(3) = vx;
    x_(4) = vy;
    x_(5) = vz;
    
    // Actualizar matriz de transición
    F_.setIdentity();
    F_(0, 3) = dt;
    F_(1, 4) = dt;
    F_(2, 5) = dt;
    
    // Predicción del estado
    x_ = F_ * x_;
    
    // Actualizar covarianza
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void ExtendedKalmanFilter::update_gps(double x, double y, double z) {
    Eigen::Vector3d z_meas(x, y, z);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * gps_noise_;
    
    Eigen::Matrix<double, 6, 3> Ht = H_.transpose();
    Eigen::Matrix3d S = H_ * P_ * Ht + R;
    Eigen::Matrix<double, 6, 3> K = P_ * Ht * S.inverse();
    
    x_ += K * (z_meas - H_ * x_);
    P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K * H_) * P_;
}