#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <cmath>

using namespace std::chrono_literals;

// Nodo para publicar en RViz
class StaticMapNode : public rclcpp::Node {
public:
    StaticMapNode() : Node("static_map_node") {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/static_map", 10);
        publisher_bea_range_ = this->create_publisher<visualization_msgs::msg::Marker>("/beacon_range", 10);
        timer_ = this->create_wall_timer(
            1s, std::bind(&StaticMapNode::publish_map, this));
        
        timer_bea_range_ = this->create_wall_timer(
            1s, std::bind(&StaticMapNode::publish_bea_ran_, this));


        map_points_ = {
            {0.0, 0.0, 0.0},
            {4.0, 0.0, 2.0},
            {-5.0, 2.0, 1.0},
            {1.0, 3.0, 0.5},
            {-3.0, -1.0, 3.0},
            {1.5, -3.0, 1.5},
        };
    }

    const std::vector<std::array<double, 3>>& get_map_points() const {
        return map_points_;
    }

private:
    void publish_map() {
        int id = 0;
        for (const auto& point : map_points_) {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "static_map";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = point[0];
            marker.pose.position.y = point[1];
            marker.pose.position.z = point[2];

            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;

            marker.lifetime = rclcpp::Duration(0, 0);
            publisher_->publish(marker);
        }
    }

    void publish_bea_ran_() {
        int id = 0;
        for (const auto& point : map_points_) {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "static_map";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = point[0];
            marker.pose.position.y = point[1];
            marker.pose.position.z = point[2];

            marker.pose.orientation.w = 1.0;
            marker.scale.x = 12.0;
            marker.scale.y = 12.0;
            marker.scale.z = 12.0;

            marker.color.a = 0.1;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;

            marker.lifetime = rclcpp::Duration(0, 0);
            publisher_bea_range_->publish(marker);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_bea_range_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_bea_range_;
    std::vector<std::array<double, 3>> map_points_;
};

// Nodo para simular distancias a balizas con límite de alcance
class BeaconDistanceNode : public rclcpp::Node {
public:
    BeaconDistanceNode(const std::vector<std::array<double, 3>>& beacons)
    : Node("beacon_distance_node"), beacons_(beacons), max_range_(6.0) { // 6 metros de alcance máximo
        sub_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/odom", 10,
            std::bind(&BeaconDistanceNode::pose_callback, this, std::placeholders::_1));

        pub_distances_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/beacon_distances", 10);
        pub_distances_real_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/beacon_distances_real", 10);

        std::random_device rd;
        gen_ = std::mt19937(rd());
        noise_ = std::normal_distribution<>(0.0, 0.10);  // 10 cm de ruido gaussiano
        dis_ = std::uniform_real_distribution<>(0.0, 1.0);

    }

private:
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;

        double success_rate = 0.9; // 90% de acierto

        std_msgs::msg::Float64MultiArray distances_msg;
        distances_msg.data.clear();
        std_msgs::msg::Float64MultiArray distances_real_msg;
        distances_real_msg.data.clear();

        for (const auto& beacon : beacons_) {
            double dx = beacon[0] - x;
            double dy = beacon[1] - y;
            double dz = beacon[2] - z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            // Distancia real (sin ruido, pero -1 si no disponible)
            if (dist > max_range_) {
                distances_real_msg.data.push_back(-1.0);
            } else {
                distances_real_msg.data.push_back(dist);
            }

            // Medida con ruido y probabilidad de acierto
            if (dis_(gen_) < success_rate) {   
                if (dist > max_range_) {
                    distances_msg.data.push_back(-1.0);  // No disponible
                } else {
                    double noisy_dist = dist + noise_(gen_);
                    distances_msg.data.push_back(noisy_dist);
                }
            } else {
                distances_msg.data.push_back(-1.0);  // No disponible
            }
        }

        pub_distances_real_->publish(distances_real_msg);
        pub_distances_->publish(distances_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_distances_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_distances_real_;
    std::vector<std::array<double, 3>> beacons_;
    double max_range_;

    std::mt19937 gen_;
    std::normal_distribution<> noise_;
    std::uniform_real_distribution<> dis_;
};


// Nodo para simular altímetro con límite de altura
class AltimeterNode : public rclcpp::Node {
public:
    AltimeterNode() : Node("altimeter_node"), max_altitude_(20.0) { // 20 metros de altura máxima
        sub_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/odom", 10,
            std::bind(&AltimeterNode::pose_callback, this, std::placeholders::_1));

        pub_altitude_ = this->create_publisher<std_msgs::msg::Float64>("/altimeter", 10);

        std::random_device rd;
        gen_ = std::mt19937(rd());
        noise_ = std::normal_distribution<>(0.0, 0.05);  // 5 cm de ruido gaussiano
    }

private:
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double z = msg->pose.pose.position.z;

        std_msgs::msg::Float64 alt_msg;

        if (z > max_altitude_) {
            alt_msg.data = -1.0;  // No disponible
        } else {
            double noisy_z = z + noise_(gen_);
            alt_msg.data = noisy_z;
        }

        pub_altitude_->publish(alt_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_altitude_;
    double max_altitude_;

    std::mt19937 gen_;
    std::normal_distribution<> noise_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto map_node = std::make_shared<StaticMapNode>();
    auto beacon_node = std::make_shared<BeaconDistanceNode>(map_node->get_map_points());
    auto altimeter_node = std::make_shared<AltimeterNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(map_node);
    executor.add_node(beacon_node);
    executor.add_node(altimeter_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
