#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

class HardwareStatusNode : public rclcpp::Node
{
public:
    HardwareStatusNode() : Node("hardware_status_publisher_cpp")
    
    {
        pub_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1) , std::bind(&HardwareStatusNode::publish_station, this));
    }

private:
    // Add your private methods and members here
    void publish_station()
    {
        auto message = my_robot_interfaces::msg::HardwareStatus();
        message.temperature = 45.5;
        message.are_motors_ready = true;
        message.debug_message = "Motors are ready";
        pub_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_ ;P
    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr pub_ ;
};


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<HardwareStatusNode>();
    RCLCPP_INFO(node->get_logger(), "Hello World! from node2");
    rclcpp::spin(node);
    rclcpp::shutdown();
    //oka to go
    return 0;

}

