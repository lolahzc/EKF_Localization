#include "kalman_filters/ekf.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

using namespace std::chrono_literals;

class EKFNode : public rclcpp::Node {
public:
    EKFNode() : Node("ekf_node") {
        // Parámetros configurables
        declare_parameter("gps_noise", 2.0);
        declare_parameter("pos_process_noise", 0.005);
        declare_parameter("vel_process_noise", 0.0005);
        
        // Inicializar filtro con parámetros
        ekf_.set_gps_noise(get_parameter("gps_noise").as_double());
        ekf_.set_process_noise(
            get_parameter("pos_process_noise").as_double(),
            get_parameter("vel_process_noise").as_double()
        );

        // Subsriptores
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/sensors/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                last_vx_ = msg->twist.twist.linear.x;
                last_vy_ = msg->twist.twist.linear.y;
                last_vz_ = msg->twist.twist.linear.z;
            });
            
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/sensors/gps", 10,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                if(!initialized_ && msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
                    ekf_.initialize(msg->latitude, msg->longitude, msg->altitude);
                    initialized_ = true;
                }
                last_gps_ = msg;
            });

        // Publicadores
        filtered_pub_ = create_publisher<visualization_msgs::msg::Marker>("/filtered_position", 10);
        gps_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("/gps_measurements", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/filtered_path", 10);

        // Servicio para parámetros dinámicos
        param_service_ = create_service<rcl_interfaces::srv::SetParameters>(
            "ekf_params",
            [this](const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> req,
                    std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> res) {
                for (auto& param : req->parameters) {
                    rcl_interfaces::msg::SetParametersResult result;
                    
                    if (param.name == "gps_noise") {
                        ekf_.set_gps_noise(param.value.double_value);
                        result.successful = true;
                    } else if (param.name == "pos_process_noise") {
                        double vel_noise = get_parameter("vel_process_noise").as_double();
                        ekf_.set_process_noise(param.value.double_value, vel_noise);
                        result.successful = true;
                    } else if (param.name == "vel_process_noise") {
                        double pos_noise = get_parameter("pos_process_noise").as_double();
                        ekf_.set_process_noise(pos_noise, param.value.double_value);
                        result.successful = true;
                    } else {
                        result.successful = false;
                        result.reason = "Parámetro desconocido: " + param.name;
                    }
                    
                    res->results.push_back(result);
                }
            });

        timer_ = create_wall_timer(50ms, [this]() { update(); });
    }

private:
    void update() {
        if(!initialized_) return;
        
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        // Predicción con odometría
        ekf_.predict(last_vx_, last_vy_, last_vz_, dt);
        
        // Corrección con GPS
        if(last_gps_ && last_gps_->status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
            ekf_.update_gps(last_gps_->latitude, last_gps_->longitude, last_gps_->altitude);
            publish_gps_marker();
            last_gps_.reset();
        }
        
        publish_filtered();
        publish_path();
    }

    void publish_filtered() {
        auto pos = ekf_.get_position();
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = now();
        marker.ns = "ekf";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.pose.position.x = pos[0];
        marker.pose.position.y = pos[1];
        marker.pose.position.z = pos[2];
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.a = 0.8;
        
        filtered_pub_->publish(marker);
    }

    void publish_gps_marker() {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = last_gps_->header.stamp;
        marker.ns = "gps";
        marker.id = marker_id_++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.pose.position.x = last_gps_->latitude;
        marker.pose.position.y = last_gps_->longitude;
        marker.pose.position.z = last_gps_->altitude;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.r = 1.0;   
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 0.6;
        marker.lifetime = rclcpp::Duration(3s);
        
        gps_vis_pub_->publish(marker);
    }

    void publish_path() {
        auto pos = ekf_.get_position();
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "world";
        pose.pose.position.x = pos[0];
        pose.pose.position.y = pos[1];
        
        path_.poses.push_back(pose);
        if(path_.poses.size() > 100) path_.poses.erase(path_.poses.begin());
        
        path_.header = pose.header;
        path_pub_->publish(path_);
    }

    ExtendedKalmanFilter ekf_;
    bool initialized_ = false;
    rclcpp::Time last_time_ = this->now();
    double last_vx_ = 0, last_vy_ = 0, last_vz_ = 0;
    sensor_msgs::msg::NavSatFix::SharedPtr last_gps_;
    int marker_id_ = 0;
    nav_msgs::msg::Path path_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr filtered_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gps_vis_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr param_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}