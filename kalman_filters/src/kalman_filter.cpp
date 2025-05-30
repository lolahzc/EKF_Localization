#include "kalman_filters/ekf.hpp"
#include <cmath>
#include <chrono>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

// Archivo para guardar el coste computacional
std::ofstream timing_file("/ar_ws/src/EKF_Localization/ekf_tiempos.csv", std::ios::app);

// Constructor
ExtendedKalmanFilter::ExtendedKalmanFilter() {
    x_ = Eigen::VectorXd::Zero(6);                          // Estado: [x, y, z, theta, phi, v]
    P_ = Eigen::MatrixXd::Identity(6, 6);                   // Covarianza inicial
    Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.05;             // Ruido del proceso
    R_gps_ = Eigen::MatrixXd::Identity(3, 3) * 0.1;         // Ruido de medición GPS (3D)
    R_odom_ = Eigen::MatrixXd::Identity(3, 3) * 0.05;        // Ruido de medición Odometría (3D)
    R_beacon_ = Eigen::MatrixXd::Identity(1, 1) * 0.04;     // Ruido de medición de balizas (1D)
    R_alt_ = Eigen::MatrixXd::Identity(1, 1) * 0.005;      // Ruido de medición altímetro (1D)
    I_ = Eigen::MatrixXd::Identity(6, 6);                   // Matriz identidad
}

// Inicialización del filtro con el estado inicial y la covarianza
void ExtendedKalmanFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
    x_ = x0;
    P_ = P0;
}

// Predicción del estado y covarianza para un modelo de dron en 3Dmplifica el modelo y es común en robótica cuando se quiere reducir la dimensión del estado y se asume que el dron siempre se mueve "hacia adelante" según su orientación
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

// Actualización del estado con la medición GPS (posición XYZ)
void ExtendedKalmanFilter::updateGPS(const Eigen::Vector3d& z) {
    auto t_start = std::chrono::steady_clock::now();
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

    auto t_end = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("EKF"), "GPS time: %.6f s", elapsed);
    if (timing_file.is_open()) timing_file << "GPS," << std::chrono::system_clock::now().time_since_epoch().count() << "," << elapsed << std::endl;
}

// Actualización del estado con la medición de odometría (velocidad XYZ)
void ExtendedKalmanFilter::updateOdom(const Eigen::Vector3d& v_measured) {
    auto t_start = std::chrono::steady_clock::now();
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

    auto t_end = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("EKF"), "Odom time: %.6f s", elapsed);
    if (timing_file.is_open()) timing_file << "Odom," << std::chrono::system_clock::now().time_since_epoch().count() << "," << elapsed << std::endl;
}

// Actualización del estado con las distancias a balizas
void ExtendedKalmanFilter::updateBeacons(const std::vector<double>& distances, const std::vector<Eigen::Vector3d>& beacon_positions) {
    auto t_start = std::chrono::steady_clock::now();
    // Para cada baliza disponible (distancia >= 0)
    for (size_t i = 0; i < distances.size(); ++i) {
        if (distances[i] < 0.0) continue; // No disponible
        // Predicción de la distancia desde el estado actual (x, y, z)
        double dx = x_(0) - beacon_positions[i](0);
        double dy = x_(1) - beacon_positions[i](1);
        double dz = x_(2) - beacon_positions[i](2);
        double dist_pred = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist_pred < 1e-6) continue; // Evitar división por cero
        // Jacobiano H (1x6), x, y, z
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
        H(0, 0) = dx / dist_pred;
        H(0, 1) = dy / dist_pred;
        H(0, 2) = dz / dist_pred;
        double y = distances[i] - dist_pred;
        
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_beacon_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        x_ = x_ + K * y;
        P_ = (I_ - K * H) * P_;
    }

    auto t_end = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("EKF"), "Beacons time: %.6f s", elapsed);
    if (timing_file.is_open()) timing_file << "Beacons," << std::chrono::system_clock::now().time_since_epoch().count() << "," << elapsed << std::endl;
}

// Actualización del estado con la medición del altímetro (z)
void ExtendedKalmanFilter::updateAltimeter(double z_measured) {
    auto t_start = std::chrono::steady_clock::now();
    // h(x) = z
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
    H(0, 2) = 1.0;
    double y = z_measured - x_(2);
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_alt_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (I_ - K * H) * P_;

    auto t_end = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();
    RCLCPP_DEBUG(rclcpp::get_logger("EKF"), "Altimeter time: %.6f s", elapsed);
    if (timing_file.is_open()) timing_file << "Altimeter," << std::chrono::system_clock::now().time_since_epoch().count() << "," << elapsed << std::endl;
}
