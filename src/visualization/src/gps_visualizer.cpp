#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

class GPSVisualizer : public rclcpp::Node {
public:
    GPSVisualizer() : Node("gps_visualizer")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "smooth_trajectory", 10, std::bind(&GPSVisualizer::trajectory_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("gps_marker", 10);

        RCLCPP_INFO(this->get_logger(), "GPS Visualizer node has been started.");
    }

private:
    void trajectory_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header = msg->header;
        marker.ns = "gps";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = msg->pose;
        marker.scale.x = 8.0;  // 2 meters diameter
        marker.scale.y = 4.0;
        marker.scale.z = 3.0;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        publisher_->publish(marker);

        RCLCPP_INFO(this->get_logger(), "Visualized trajectory point: %.2f, %.2f, %.2f",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSVisualizer>());
    rclcpp::shutdown();
    return 0;
}