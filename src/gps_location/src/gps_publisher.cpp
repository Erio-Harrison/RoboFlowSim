#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GPSPublisher : public rclcpp::Node {
public:
    GPSPublisher() : Node("gps_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gps_pose", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GPSPublisher::publish_gps, this));
    }

private:
    void publish_gps()
    {
        auto message = geometry_msgs::msg::PoseStamped();
        message.header.stamp = this->now();
        message.header.frame_id = "map";

        // 模拟的GPS坐标
        double lon = 116.4074;  // 经度
        double lat = 39.9042;   // 纬度

        // 转换为局部坐标系（以第一个点为原点）
        static const double origin_lon = 116.4074;
        static const double origin_lat = 39.9042;
        
        // 简单的平面投影（仅适用于小范围）
        const double EARTH_RADIUS = 6371000.0; // 地球半径，单位：米
        message.pose.position.x = EARTH_RADIUS * (lon - origin_lon) * cos(origin_lat * M_PI / 180.0) * M_PI / 180.0;
        message.pose.position.y = EARTH_RADIUS * (lat - origin_lat) * M_PI / 180.0;
        message.pose.position.z = 0.0;  // 假设高度为0

        // 设置一个默认的方向
        message.pose.orientation.w = 1.0;

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published GPS pose: %.2f, %.2f",
                    message.pose.position.x, message.pose.position.y);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSPublisher>());
    rclcpp::shutdown();
    return 0;
}