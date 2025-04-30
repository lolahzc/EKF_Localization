#pragma once

#include <Eigen/Dense>

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();

    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
    void predict(double dt);
    void updateGPS(const Eigen::Vector3d& z);
    void updateOdom(const Eigen::Vector3d& z);

    const Eigen::VectorXd& getState() const { return x_; }

private:
    Eigen::VectorXd x_; // Estado: [x, y, z, vx, vy, vz]
    Eigen::MatrixXd P_; // Covarianza del estado
    Eigen::MatrixXd Q_; // Ruido del proceso
    Eigen::MatrixXd R_gps_;   // Ruido de medición GPS
    Eigen::MatrixXd R_odom_;  // Ruido de medición Odometría

    Eigen::MatrixXd I_; // Matriz identidad
};
