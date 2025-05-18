#pragma once

#include <Eigen/Dense>

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();

    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);    // Inicializa el filtro con el estado inicial y la covarianza
    void predict(double dt, const Eigen::Vector3d& omega);                // Predicción del estado y covarianza
    void updateGPS(const Eigen::Vector3d& z);                             // Actualiza el estado con la medición GPS
    void updateOdom(const Eigen::Vector3d& v_measured);                  // Actualiza el estado con la medición de odometría  
    void updateBeacons(const std::vector<double>& distances, const std::vector<Eigen::Vector3d>& beacon_positions); // Actualiza con distancias a balizas
    void updateAltimeter(double z_measured); // Actualiza con altímetro

    const Eigen::VectorXd& getState() const { return x_; }               // Devuelve el estado actual

private:
    Eigen::VectorXd x_;                                                 // Estado: [x, y, z, theta, phi, v]
    Eigen::MatrixXd P_;                                                 // Covarianza del estado
    Eigen::MatrixXd Q_;                                                 // Ruido del proceso
    Eigen::MatrixXd R_gps_;                                             // Ruido de medición GPS
    Eigen::MatrixXd R_odom_;                                            // Ruido de medición Odometría
    Eigen::MatrixXd R_beacon_;                                          // Ruido de medición Balizas
    Eigen::MatrixXd R_alt_;                                             // Ruido de medición Altímetro

    Eigen::MatrixXd I_;                                                 // Matriz identidad
};
