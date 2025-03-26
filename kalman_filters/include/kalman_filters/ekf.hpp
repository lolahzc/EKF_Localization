#pragma once
#include <eigen3/Eigen/Dense>

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();
    void initialize(double x0, double y0);
    void predict(double vx, double vy, double dt);
    void update_gps(double x, double y);
    
    // Configuración de parámetros
    void set_gps_noise(double noise);
    void set_process_noise(double pos_noise, double vel_noise);
    
    Eigen::Vector2d get_position() const { return x_.head<2>(); }
    Eigen::Vector2d get_velocity() const { return x_.tail<2>(); }

private:
    // Estado: [x, y, vx, vy]
    Eigen::Vector4d x_;
    
    // Matrices de covarianza
    Eigen::Matrix4d P_;
    Eigen::Matrix4d F_;
    Eigen::Matrix4d Q_;
    Eigen::Matrix<double, 2, 4> H_;
    
    // Parámetros de ruido
    double gps_noise_ = 2.0;       // Valor alto inicial para menos confianza en GPS
    double pos_process_noise_ = 0.005;
    double vel_process_noise_ = 0.0005;
};