#include "kalman_filters/ekf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

class EKFNode : public rclcpp::Node {
public:
    EKFNode() : Node("ekf_node"), initialized_(false) {
        ekf_ = std::make_unique<ExtendedKalmanFilter>();

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/sensors/odom", 10,
            std::bind(&EKFNode::odom_callback, this, std::placeholders::_1));

        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/sensors/gps", 10,
            std::bind(&EKFNode::gps_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ekf/pos_estimated", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EKFNode::publish_estimate, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        Eigen::Vector3d vel(msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.linear.z);
        ekf_->updateOdom(vel);
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        Eigen::Vector3d pos(msg->latitude, msg->longitude, msg->altitude);

        if (!initialized_) {
            Eigen::VectorXd x0(6);
            x0 << pos(0), pos(1), pos(2), 0.0, 0.0, 0.0;
            Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6, 6) * 0.1;
            ekf_->init(x0, P0);
            initialized_ = true;
            last_time_ = this->now();
        } else {
            auto now = this->now();
            double dt = (now - last_time_).seconds();
            ekf_->predict(dt);
            ekf_->updateGPS(pos);
            last_time_ = now;
        }
    }

    void publish_estimate() {
        if (!initialized_) return;

        const Eigen::VectorXd& x = ekf_->getState();
        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = this->now();
        marker.header.frame_id = "world";

        marker.ns = "ekf_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;  // Tipo de Marker: esfera
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = x(0);
        marker.pose.position.y = x(1);
        marker.pose.position.z = x(2);

        marker.scale.x = 0.1;  // Escala de la esfera en x
        marker.scale.y = 0.1;  // Escala de la esfera en y
        marker.scale.z = 0.1;  // Escala de la esfera en z

        marker.color.a = 0.8;  // Opacidad
        marker.color.r = 0.8;  // Rojo
        marker.color.g = 0.5;  // Verde
        marker.color.b = 0.0;  // Azul

        marker_pub_->publish(marker);
    }

    std::unique_ptr<ExtendedKalmanFilter> ekf_;
    bool initialized_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
