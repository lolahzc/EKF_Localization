#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <random>
#include <chrono>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class RandomPointNode : public rclcpp::Node {
public:
    RandomPointNode() : Node("random_point_node"),
                        x_(0.0), y_(0.0), z_(0.0),
                        vx_(0.0), vy_(0.0), vz_(0.0),
                        roll_(0.0), pitch_(0.0), yaw_(0.0),
                        max_speed_(0.5),
                        max_delta_v_(0.2),
                        max_height_(5.0),
                        gps_count_(0) {
        // Publishers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/moving_point", 10);
        ground_truth_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ground_truth/odom", 10);
        noisy_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/sensors/odom", 10);
        noisy_gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/sensors/gps", 10);
        gps_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/gps_marker", 10);
        
        // Timers
        update_timer_ = this->create_wall_timer(100ms, std::bind(&RandomPointNode::update_callback, this));
        sensor_timer_ = this->create_wall_timer(100ms, std::bind(&RandomPointNode::sensor_callback, this));

        // Random initialization
        gen_ = std::mt19937(std::random_device{}());
        vx_ = std::uniform_real_distribution<>(-max_speed_, max_speed_)(gen_);
        vy_ = std::uniform_real_distribution<>(-max_speed_, max_speed_)(gen_);
        vz_ = std::uniform_real_distribution<>(-max_height_, max_height_)(gen_);
        
        // Parameters
        declare_parameter("odom_noise_variance", 0.01);
        declare_parameter("gps_noise_variance", 0.1);
    }

private:
    void update_callback() {
        update_position();
        publish_marker();
        publish_ground_truth();
    }

    void update_position() {
        const double dt = 0.1;
        
        // Update velocity
        std::uniform_real_distribution<> delta_dist(-max_delta_v_, max_delta_v_);
        vx_ += delta_dist(gen_);
        vy_ += delta_dist(gen_);
        vz_ += delta_dist(gen_);
        
        // Clamp velocity
        vx_ = std::clamp(vx_, -max_speed_, max_speed_);
        vy_ = std::clamp(vy_, -max_speed_, max_speed_);
        vz_ = std::clamp(vz_, -max_height_, max_height_);

        if (std::abs(vz_) < 0.01) {
            vz_ += std::uniform_real_distribution<>(-0.05, 0.05)(gen_); // Stop vertical movement if close to zero
        }
        
        // Update position
        x_ += vx_ * dt;
        y_ += vy_ * dt;
        z_ += vz_ * dt;
        
        // Check boundaries
        check_boundary(x_, vx_, -5.0, 5.0);
        check_boundary(y_, vy_, -5.0, 5.0);
        check_boundary(z_, vz_, 0.0, max_height_);
    }

    void publish_marker() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = now();
        marker.ns = "moving_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Actualizar la posición
        marker.pose.position.x = x_;
        marker.pose.position.y = y_;
        marker.pose.position.z = z_;
        
        // Usar roll, pitch, yaw para la orientación
        tf2::Quaternion quat;
        quat.setRPY(roll_, pitch_, yaw_);
        
        // Convertir el Quaternion a geometry_msgs::msg::Quaternion manualmente
        geometry_msgs::msg::Quaternion quat_msg;
        quat_msg.x = quat.x();
        quat_msg.y = quat.y();
        quat_msg.z = quat.z();
        quat_msg.w = quat.w();

        marker.pose.orientation = quat_msg;
        
        // Corregir escala y color
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        marker_pub_->publish(marker);
    }

    void publish_ground_truth() {
        nav_msgs::msg::Odometry odom;
        
        // Corregir header
        odom.header.stamp = now();
        odom.header.frame_id = "world";
        
        // Corregir posición
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = z_;
        
        // Corregir velocidad
        odom.twist.twist.linear.x = vx_;
        odom.twist.twist.linear.y = vy_;
        odom.twist.twist.linear.z = vz_;
        
        ground_truth_pub_->publish(odom);
    }

    void sensor_callback() {
        publish_noisy_odom();
        publish_noisy_gps();
    }

    void publish_noisy_odom() {
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = now();
        odom.header.frame_id = "odom";
        
        std::normal_distribution<double> noise(
            0.0, 
            get_parameter("odom_noise_variance").as_double()
        );
        
        // Asignación explícita
        odom.twist.twist.linear.x = vx_ + noise(gen_);
        odom.twist.twist.linear.y = vy_ + noise(gen_);
        odom.twist.twist.linear.z = 0.0;
        
        noisy_odom_pub_->publish(odom);
    }

    void publish_noisy_gps() {
        if (gps_count_++ % 10 == 0) {
            std::normal_distribution<double> gps_noise(
                0.0, 
                get_parameter("gps_noise_variance").as_double()
            );
            
            // Publicar NavSatFix original
            auto gps = sensor_msgs::msg::NavSatFix();
            gps.header.stamp = now();
            gps.header.frame_id = "world";
            gps.latitude = x_ + gps_noise(gen_);
            gps.longitude = y_ + gps_noise(gen_);
            gps.altitude = z_ + gps_noise(gen_);
            gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            noisy_gps_pub_->publish(gps);

            // Publicar Marker para visualización
            auto marker = visualization_msgs::msg::Marker();
            marker.header = gps.header;
            marker.ns = "gps_measurements";
            marker.id = gps_count_;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = gps.latitude;
            marker.pose.position.y = gps.longitude;
            marker.pose.position.z = gps.altitude;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            marker.lifetime = rclcpp::Duration(5s);
            
            gps_marker_pub_->publish(marker);
        }
    }

    void check_boundary(double& pos, double& vel, double min, double max) {
        const double rebound_factor = 0.7;
        if (pos > max) {
            pos = max;
            vel = -std::abs(vel) * rebound_factor;
        } else if (pos < min) {
            pos = min;
            vel = std::abs(vel) * rebound_factor;
        }
    }

    // Variables
    double x_, y_, z_;
    double vx_, vy_, vz_;
    double roll_, pitch_, yaw_;
    double max_speed_;
    double max_delta_v_;
    double max_height_;
    int gps_count_;

    std::mt19937 gen_;
    
    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr noisy_odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr noisy_gps_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gps_marker_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr sensor_timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomPointNode>());
    rclcpp::shutdown();
    return 0;
}
