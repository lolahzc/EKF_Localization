// Dentro de src/ crea static_map_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class StaticMapNode : public rclcpp::Node {
public:
    StaticMapNode() : Node("static_map_node") {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/static_map", 10);
        timer_ = this->create_wall_timer(
            1s, std::bind(&StaticMapNode::publish_map, this));
        
        // Posiciones de los puntos estáticos (personaliza estos valores)
        map_points_ = {
            {0.0, 0.0, 0.0},    // Centro
            {2.0, 0.0, 0.0},    // Derecha
            {-2.0, 0.0, 0.0},   // Izquierda
            {0.0, 2.0, 0.0},    // Arriba
            {0.0, -2.0, 0.0}    // Abajo
        };
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
            marker.color.b = 1.0;  // Azul para puntos estáticos
            
            // Duración infinita (se mostrará hasta que se borre)
            marker.lifetime = rclcpp::Duration(0, 0);
            
            publisher_->publish(marker);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    std::vector<std::array<double, 3>> map_points_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticMapNode>());
    rclcpp::shutdown();
    return 0;
}