#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>
#include <cmath>
#include <random>

struct Particle2D {
    double x, y;
    double vx, vy;
    double lifetime;
};

class GasDiffusion2D : public rclcpp::Node {
public:
    GasDiffusion2D() : Node("gas_diffusion_2d"), 
                               gen(rd()), 
                               dis(-1.0, 1.0),
                               dis_speed(0.0, 1.0)  // 增加速度范围
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("gas_diffusion_2d", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                         std::bind(&GasDiffusion2D::publish_gas_diffusion, this));
        RCLCPP_INFO(this->get_logger(), "GasDiffusion2D initialized");
    }

private:
    void publish_gas_diffusion()
    {
        update_particles();
        auto marker = create_diffusion_marker();
        publisher_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Published 2D gas diffusion");
    }

    void update_particles()
    {
        while (particles.size() < 3000) {  // 增加粒子数量
            Particle2D p;
            p.x = p.y = 0;
            p.vx = dis_speed(gen);
            p.vy = dis_speed(gen);
            p.lifetime = 3.0;  // 减少粒子寿命，使扩散更快
            particles.push_back(p);
        }

        for (auto it = particles.begin(); it != particles.end();) {
            it->x += it->vx;
            it->y += it->vy;
            it->lifetime -= 0.05;

            it->vx += dis(gen) * 0.2;  // 增加随机性
            it->vy += dis(gen) * 0.2;

            if (it->lifetime <= 0) {
                it = particles.erase(it);
            } else {
                ++it;
            }
        }
    }

    visualization_msgs::msg::Marker create_diffusion_marker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "gas_particles_2d";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;  // 减小点的大小，但增加数量
        marker.scale.y = 1;

        for (const auto& p : particles) {
            geometry_msgs::msg::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0.0;
            marker.points.push_back(point);

            std_msgs::msg::ColorRGBA color;
            color.r = 1.0;
            color.g = 0.3 * (1 - p.lifetime / 3.0);  // 添加一些黄色
            color.b = 0.0;
            color.a = std::min(1.0, p.lifetime / 1.5);  // 增加不透明度
            marker.colors.push_back(color);
        }

        return marker;
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Particle2D> particles;
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<> dis;
    std::uniform_real_distribution<> dis_speed;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GasDiffusion2D>());
    rclcpp::shutdown();
    return 0;
}