#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <cmath>

class GasSensorNode : public rclcpp::Node
{
public:
    GasSensorNode() : Node("gas_sensor_node")
    {
        this->declare_parameter("sensor_position", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("sensor_type", "TGS2600");
        this->declare_parameter("rise_time", 1.8);  // seconds
        this->declare_parameter("decay_time", 20.7);  // seconds

        sensor_position_ = this->get_parameter("sensor_position").as_double_array();
        sensor_type_ = this->get_parameter("sensor_type").as_string();
        rise_time_ = this->get_parameter("rise_time").as_double();
        decay_time_ = this->get_parameter("decay_time").as_double();

        concentration_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "gas_concentration", 10, std::bind(&GasSensorNode::concentration_callback, this, std::placeholders::_1));

        sensor_output_pub_ = this->create_publisher<std_msgs::msg::Float64>("sensor_output", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                         std::bind(&GasSensorNode::publish_sensor_output, this));

        last_concentration_ = 0.0;
        current_output_ = 0.0;
    }

private:
    void concentration_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        double closest_concentration = 0.0;
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < msg->data.size(); i += 4)
        {
            double dx = msg->data[i] - sensor_position_[0];
            double dy = msg->data[i+1] - sensor_position_[1];
            double dz = msg->data[i+2] - sensor_position_[2];
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (distance < min_distance)
            {
                min_distance = distance;
                closest_concentration = msg->data[i+3];
            }
        }

        last_concentration_ = closest_concentration;
    }

    void publish_sensor_output()
    {
        double target_output = calculate_sensor_response(last_concentration_);
        
        // Apply rise/decay time constants
        double time_constant = (target_output > current_output_) ? rise_time_ : decay_time_;
        double alpha = 1.0 - std::exp(-0.1 / time_constant);  // 0.1 is the update interval in seconds
        current_output_ = current_output_ + alpha * (target_output - current_output_);

        auto msg = std_msgs::msg::Float64();
        msg.data = current_output_;
        sensor_output_pub_->publish(msg);
    }

    double calculate_sensor_response(double concentration)
    {
        // Simplified sensor response model
        // You may want to implement more accurate models for different TGS sensors
        if (sensor_type_ == "TGS2600")
        {
            return 1.0 / (1.0 + std::pow(concentration / 1000.0, 0.8));  // Example for ethanol
        }
        else if (sensor_type_ == "TGS2611")
        {
            return 1.0 / (1.0 + std::pow(concentration / 5000.0, 0.6));  // Example for methane
        }
        else if (sensor_type_ == "TGS2620")
        {
            return 1.0 / (1.0 + std::pow(concentration / 3000.0, 0.7));  // Example for alcohol
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown sensor type. Using default response.");
            return 1.0 / (1.0 + concentration / 1000.0);
        }
    }

    std::vector<double> sensor_position_;
    std::string sensor_type_;
    double rise_time_;
    double decay_time_;
    double last_concentration_;
    double current_output_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr concentration_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sensor_output_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GasSensorNode>());
    rclcpp::shutdown();
    return 0;
}