#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <cmath>
#include <random>

class GasDispersionNode : public rclcpp::Node
{
public:
    GasDispersionNode() : Node("gas_dispersion_node")
    {
        this->declare_parameter("source_position", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("wind_velocity", std::vector<double>{1.0, 0.0, 0.0});
        this->declare_parameter("diffusion_coefficient", 0.1);
        this->declare_parameter("num_filaments", 1000);
        this->declare_parameter("release_rate", 10.0);
        
        source_position_ = this->get_parameter("source_position").as_double_array();
        wind_velocity_ = this->get_parameter("wind_velocity").as_double_array();
        diffusion_coefficient_ = this->get_parameter("diffusion_coefficient").as_double();
        num_filaments_ = this->get_parameter("num_filaments").as_int();
        release_rate_ = this->get_parameter("release_rate").as_double();

        concentration_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("gas_concentration", 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                         std::bind(&GasDispersionNode::update_and_publish, this));

        filaments_.resize(num_filaments_);
        for (auto& filament : filaments_)
        {
            filament.position = source_position_;
            filament.width = 0.1;  // Initial width
        }
    }

private:
    struct Filament
    {
        std::vector<double> position;
        double width;
    };

    void update_and_publish()
    {
        update_filaments();
        publish_concentration();
    }

    void update_filaments()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<> d(0, 0.1);  // For stochastic turbulence

        for (auto& filament : filaments_)
        {
            // Advection
            filament.position[0] += wind_velocity_[0] * 0.1;
            filament.position[1] += wind_velocity_[1] * 0.1;
            filament.position[2] += wind_velocity_[2] * 0.1;

            // Turbulence (stochastic process)
            filament.position[0] += d(gen);
            filament.position[1] += d(gen);
            filament.position[2] += d(gen);

            // Molecular diffusion (increase width)
            filament.width += std::sqrt(2 * diffusion_coefficient_ * 0.1);
        }

        // Release new filaments
        static int count = 0;
        if (count++ % 10 == 0)  // Release every 1 second (10 * 0.1s)
        {
            for (int i = 0; i < release_rate_; ++i)
            {
                if (filaments_.size() < static_cast<size_t>(num_filaments_))
                {
                    Filament new_filament;
                    new_filament.position = source_position_;
                    new_filament.width = 0.1;
                    filaments_.push_back(new_filament);
                }
            }
        }
    }

    void publish_concentration()
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(3 + num_filaments_ * 4);  // x, y, z, concentration for each filament

        for (size_t i = 0; i < filaments_.size(); ++i)
        {
            msg.data[i*4] = filaments_[i].position[0];
            msg.data[i*4+1] = filaments_[i].position[1];
            msg.data[i*4+2] = filaments_[i].position[2];
            msg.data[i*4+3] = calculate_concentration(filaments_[i]);
        }

        concentration_pub_->publish(msg);
    }

    double calculate_concentration(const Filament& filament)
    {
        // Simplified concentration calculation
        return 1.0 / (filament.width * filament.width * filament.width);
    }

    std::vector<Filament> filaments_;
    std::vector<double> source_position_;
    std::vector<double> wind_velocity_;
    double diffusion_coefficient_;
    int num_filaments_;
    double release_rate_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr concentration_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GasDispersionNode>());
    rclcpp::shutdown();
    return 0;
}