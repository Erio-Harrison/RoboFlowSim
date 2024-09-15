#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <vector>
#include <cmath>
#include <random>

struct Particle {
    double x, y, z;
    double vx, vy, vz;
    double lifetime;
};

class RealisticGasDiffusion : public rclcpp::Node {
public:
    RealisticGasDiffusion() : Node("gas_diffusion"), 
                             gen(rd()), 
                             dis(-1.0, 1.0),
                             dis_speed(0.0, 0.5)
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("gas_diffusion", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                         std::bind(&RealisticGasDiffusion::publish_gas_diffusion, this));
        RCLCPP_INFO(this->get_logger(), "RealisticGasDiffusion initialized");
    }

private:
    void publish_gas_diffusion()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        update_particles();

        auto particles_marker = create_particles_marker();
        marker_array.markers.push_back(particles_marker);

        publisher_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published gas diffusion");
    }

    void update_particles()
    {
        while (particles.size() < 2000) {  // 增加粒子数量
            Particle p;
            p.x = p.y = p.z = 0;
            p.vx = dis_speed(gen);
            p.vy = dis_speed(gen);
            p.vz = dis_speed(gen);
            p.lifetime = 5.0;  // 减少粒子寿命，使扩散更快
            particles.push_back(p);
        }

        for (auto it = particles.begin(); it != particles.end();) {
            it->x += it->vx;
            it->y += it->vy;
            it->z += it->vz;
            it->lifetime -= 0.05;

            it->vx += dis(gen) * 0.1;
            it->vy += dis(gen) * 0.1;
            it->vz += dis(gen) * 0.1;

            if (it->lifetime <= 0) {
                it = particles.erase(it);
            } else {
                ++it;
            }
        }
    }

    visualization_msgs::msg::Marker create_particles_marker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "gas_particles";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;  // 增大点的大小
        marker.scale.y = 1;

        for (const auto& p : particles) {
            geometry_msgs::msg::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            marker.points.push_back(point);

            std_msgs::msg::ColorRGBA color;
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = std::min(1.0, p.lifetime / 2.5);  // 增加不透明度
            marker.colors.push_back(color);
        }

        return marker;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Particle> particles;
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<> dis;
    std::uniform_real_distribution<> dis_speed;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealisticGasDiffusion>());
    rclcpp::shutdown();
    return 0;
}