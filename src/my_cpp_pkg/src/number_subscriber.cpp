#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"



class NumberSub : public rclcpp::Node
{
public:
    NumberSub() : Node("number_counter")
    {
        sub_ = this->create_subscription<std_msgs::msg::Int64>("number", 10, std::bind(&NumberSub::sub_station, this, std::placeholders::_1));
        pub_ = this->create_publisher<std_msgs::msg::Int64>("number_count", 10);
    }

private:
    // Add your private methods and members here
    void sub_station(std_msgs::msg::Int64::SharedPtr message)
    {
        RCLCPP_INFO(this->get_logger(), "Received integer: '%ld'", message->data);
        result = message->data + result ;
        auto output_msg = std_msgs::msg::Int64();
        output_msg.data = result;
        pub_->publish(output_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_ ;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_ ;
    int64_t result = 0;
};




int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<NumberSub>();
    // RCLCPP_INFO(node3->get_logger(), "Hello World! from node2");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
