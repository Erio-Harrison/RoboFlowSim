#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

class TrajectoryPlanner : public rclcpp::Node {
public:
    TrajectoryPlanner() : Node("trajectory_planner"), t_(0.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("smooth_trajectory", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz
            std::bind(&TrajectoryPlanner::publish_smooth_trajectory, this));

        RCLCPP_INFO(this->get_logger(), "Trajectory Planner node has been started.");
    }

private:
    void publish_smooth_trajectory()
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";

        // Generate a figure-8 trajectory
        pose.pose.position.x = 50.0 * std::sin(t_);
        pose.pose.position.y = 25.0 * std::sin(2 * t_);
        pose.pose.position.z = 0.0;

        // Simple orientation change
        pose.pose.orientation.w = std::cos(t_ / 2);
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(t_ / 2);

        publisher_->publish(pose);

        t_ += 0.05;  // Increment time
        if (t_ > 2 * M_PI) t_ -= 2 * M_PI;  // Reset after one full cycle

        RCLCPP_INFO(this->get_logger(), "Published trajectory point: %.2f, %.2f", 
                    pose.pose.position.x, pose.pose.position.y);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double t_;  // Time parameter for trajectory generation
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPlanner>());
    rclcpp::shutdown();
    return 0;
}