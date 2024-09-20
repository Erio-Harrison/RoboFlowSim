#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <random>
#include <vector>
#include <cmath>

class VisualizationNode : public rclcpp::Node
{
public:
    VisualizationNode() : Node("visualization_node"), gen_(rd_())
    {
        this->declare_parameter("max_particles", 2000);
        this->declare_parameter("particle_lifetime", 15.0);
        this->declare_parameter("update_interval", 0.05);
        this->declare_parameter("particle_size", 0.05);
        this->declare_parameter("initial_speed", 1.0);
        this->declare_parameter("gravity_effect", 0.2);
        this->declare_parameter("initial_height", 3.0);
        this->declare_parameter("horizontal_spread", 0.5);
        this->declare_parameter("vertical_spread", 0.3);
        this->declare_parameter("speed_decay", 0.02);
        this->declare_parameter("diffusion_strength", 0.5);
        this->declare_parameter("max_distance", 10.0);

        max_particles_ = this->get_parameter("max_particles").as_int();
        particle_lifetime_ = this->get_parameter("particle_lifetime").as_double();
        update_interval_ = this->get_parameter("update_interval").as_double();
        particle_size_ = this->get_parameter("particle_size").as_double();
        initial_speed_ = this->get_parameter("initial_speed").as_double();
        gravity_effect_ = this->get_parameter("gravity_effect").as_double();
        initial_height_ = this->get_parameter("initial_height").as_double();
        horizontal_spread_ = this->get_parameter("horizontal_spread").as_double();
        vertical_spread_ = this->get_parameter("vertical_spread").as_double();
        speed_decay_ = this->get_parameter("speed_decay").as_double();
        diffusion_strength_ = this->get_parameter("diffusion_strength").as_double();
        max_distance_ = this->get_parameter("max_distance").as_double();

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("gas_visualization", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(update_interval_),
            std::bind(&VisualizationNode::publish_gas_diffusion, this));

        dis_speed_ = std::uniform_real_distribution<>(initial_speed_ * 0.8, initial_speed_ * 1.2);
        dis_horizontal_ = std::normal_distribution<>(0.0, horizontal_spread_);
        dis_vertical_ = std::normal_distribution<>(0.0, vertical_spread_);
    }

private:
    struct Particle {
        double x, y, z;
        double vx, vy, vz;
        double lifetime;
        double initial_lifetime;
    };

    void publish_gas_diffusion()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        auto pipe_marker = create_pipe_marker();
        marker_array.markers.push_back(pipe_marker);

        update_particles();
        auto particles_marker = create_particles_marker();
        marker_array.markers.push_back(particles_marker);

        publisher_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published gas diffusion and pipe");
    }

    visualization_msgs::msg::Marker create_pipe_marker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "pipe";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = -5.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = initial_height_ / 2.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.5; 
        marker.scale.y = 0.5;
        marker.scale.z = initial_height_; 

        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 1.0;

        return marker;
    }

    void update_particles()
    {
        while (particles_.size() < static_cast<size_t>(max_particles_)) {
            Particle p;
            p.x = -5.0;
            p.y = dis_horizontal_(gen_);
            p.z = initial_height_ + dis_vertical_(gen_);
            double speed = dis_speed_(gen_);
            double angle_horizontal = dis_horizontal_(gen_);
            double angle_vertical = dis_vertical_(gen_);
            p.vx = speed * std::cos(angle_vertical) * std::cos(angle_horizontal);
            p.vy = speed * std::cos(angle_vertical) * std::sin(angle_horizontal);
            p.vz = speed * std::sin(angle_vertical);
            p.lifetime = particle_lifetime_;
            p.initial_lifetime = particle_lifetime_;
            particles_.push_back(p);
        }

        for (auto it = particles_.begin(); it != particles_.end();) {

            it->x += it->vx * update_interval_;
            it->y += it->vy * update_interval_;
            it->z += it->vz * update_interval_;
            
            it->vz -= gravity_effect_ * update_interval_;

            it->vx += dis_horizontal_(gen_) * diffusion_strength_ * update_interval_;
            it->vy += dis_horizontal_(gen_) * diffusion_strength_ * update_interval_;
            it->vz += dis_vertical_(gen_) * diffusion_strength_ * update_interval_;

            it->vx *= (1.0 - speed_decay_ * update_interval_);
            it->vy *= (1.0 - speed_decay_ * update_interval_);
            it->vz *= (1.0 - speed_decay_ * update_interval_);

            if (it->z <= 0) {
                it->z = -it->z;
                it->vz = -it->vz * 0.5;
            }

            it->lifetime -= update_interval_;

            if (it->lifetime <= 0 || it->x >= max_distance_ || std::abs(it->y) > max_distance_ / 2) {
                it = particles_.erase(it);
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
        marker.scale.x = particle_size_;
        marker.scale.y = particle_size_;

        for (const auto& p : particles_) {
            geometry_msgs::msg::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            marker.points.push_back(point);

            std_msgs::msg::ColorRGBA color;
            
            double distance_factor = p.x / max_distance_;
            double lifetime_factor = p.lifetime / p.initial_lifetime;
            double fade_factor = std::min(1.0 - distance_factor, lifetime_factor);

            color.r = 1.0;
            color.g = 1.0 - fade_factor;
            color.b = 0.0;
            color.a = fade_factor;

            marker.colors.push_back(color);
        }

        return marker;
    }

    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> dis_speed_;
    std::normal_distribution<> dis_horizontal_;
    std::normal_distribution<> dis_vertical_;

    std::vector<Particle> particles_;
    int max_particles_;
    double particle_lifetime_;
    double update_interval_;
    double particle_size_;
    double initial_speed_;
    double gravity_effect_;
    double initial_height_;
    double horizontal_spread_;
    double vertical_spread_;
    double speed_decay_;
    double diffusion_strength_;
    double max_distance_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizationNode>());
    rclcpp::shutdown();
    return 0;
}