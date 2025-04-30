#pragma once
#include <eigen3/Eigen/Dense>

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();
    void initialize(double x0, double y0, double z0);
    void predict(double vx, double vy, double vz,double dt);
    void update_gps(double x, double y, double z);
    
    // Configuración de parámetros
    void set_gps_noise(double noise);
    void set_process_noise(double pos_noise, double vel_noise);
    
    Eigen::Vector3d get_position() const { return x_.segment<3>(0); }
    Eigen::Vector3d get_velocity() const { return x_.segment<3>(3); }

private:
    // Estado: [x, y, vx, vy]
    Eigen::Matrix<double, 6, 1> x_;
        
    // Matrices de covarianza
    Eigen::Matrix<double, 6, 6> P_, F_, Q_;
    Eigen::Matrix<double, 3, 6> H_;
    
    // Parámetros de ruido
    double gps_noise_ = 2.0;       // Valor alto inicial para menos confianza en GPS
    double pos_process_noise_ = 0.005;
    double vel_process_noise_ = 0.0005;
};