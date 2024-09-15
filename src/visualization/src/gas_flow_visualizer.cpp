#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>
#include <cmath>
#include <random>

class GasFlowPublisher : public rclcpp::Node {
public:
    GasFlowPublisher() : Node("gas_flow_publisher"), gen(rd()), dis(-10.0, 10.0)
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("gas_flow_marker", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&GasFlowPublisher::publish_gas_flow, this));
        RCLCPP_INFO(this->get_logger(), "GasFlowPublisher initialized");
    }

private:
    void publish_gas_flow()
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();

        marker.ns = "gas_flow";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;

        // 生成模拟的气体流动点
        const int num_points = 1000;
        for (int i = 0; i < num_points; ++i)
        {
            double t = static_cast<double>(i) / num_points;
            double x = 50 * std::cos(2 * M_PI * t) + dis(gen);
            double y = 50 * std::sin(2 * M_PI * t) + dis(gen);
            double z = 20 * std::sin(4 * M_PI * t) + dis(gen);

            geometry_msgs::msg::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            marker.points.push_back(point);

            // 根据高度设置颜色（蓝色到红色的渐变）
            std_msgs::msg::ColorRGBA color;
            color.r = (z + 20) / 40;  // 假设z的范围是-20到20
            color.b = 1.0 - color.r;
            color.g = 0.0;
            color.a = 0.8;
            marker.colors.push_back(color);
        }

        publisher_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Published gas flow marker");
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<> dis;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting gas flow visualizer");
    auto node = std::make_shared<GasFlowPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}