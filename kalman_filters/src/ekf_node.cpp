#include "kalman_filters/ekf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

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

        beacon_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/beacon_distances", 10,
            std::bind(&EKFNode::beacon_callback, this, std::placeholders::_1));

        altimeter_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/altimeter", 10,
            std::bind(&EKFNode::altimeter_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ekf/pos_estimated", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EKFNode::publish_estimate, this));

        beacon_positions_ = {
            Eigen::Vector3d(0.0, 0.0, 0.0),
            Eigen::Vector3d(4.0, 0.0, 2.0),
            Eigen::Vector3d(-5.0, 2.0, 1.0),
            Eigen::Vector3d(1.0, 3.0, 0.5),
            Eigen::Vector3d(-3.0, -1.0, 3.0),
            Eigen::Vector3d(1.5, -3.0, 1.5)
        };
    }
 
private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        Eigen::Vector3d v;
        v << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
        Eigen::Vector3d omega;
        omega << msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;
        ekf_->updateOdom(v);
        last_omega_ = omega;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        Eigen::Vector3d pos(msg->latitude, msg->longitude, msg->altitude);

        if (!initialized_) {
            Eigen::VectorXd x0(6);
            x0 << pos(0), pos(1), pos(2), 0.0, 0.0, 0.0;  // Estado inicial
            Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6, 6) * 0.1;  // Covarianza inicial
            ekf_->init(x0, P0);
            initialized_ = true;
            last_time_ = this->now();
        } else {
            auto now = this->now();
            double dt = (now - last_time_).seconds();
            ekf_->predict(dt, last_omega_);
            ekf_->updateGPS(pos);
            last_time_ = now;
        }
    }

    void beacon_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (!initialized_) return;
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        ekf_->predict(dt, last_omega_);
        ekf_->updateBeacons(msg->data, beacon_positions_);
        last_time_ = now;
    }

    void altimeter_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        if (!initialized_) return;
        if (msg->data >= 0.0) { // -1.0 indica no disponible
            auto now = this->now();
            double dt = (now - last_time_).seconds();
            ekf_->predict(dt, last_omega_);
            ekf_->updateAltimeter(msg->data);
            last_time_ = now;
        }
    }

    void publish_estimate() {
        if (!initialized_) return;

        const Eigen::VectorXd& x = ekf_->getState();

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "ekf";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x(0);
        marker.pose.position.y = x(1);
        marker.pose.position.z = x(2);  

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;

        marker_pub_->publish(marker);
    }

    std::unique_ptr<ExtendedKalmanFilter> ekf_;
    bool initialized_;
    rclcpp::Time last_time_;
    Eigen::Vector3d last_omega_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::vector<Eigen::Vector3d> beacon_positions_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr beacon_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr altimeter_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
