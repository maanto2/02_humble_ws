#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MySubNode : public rclcpp::Node
{
public:
    MySubNode() : Node("robot_station_sub")
    
    {
        sub_ = this->create_subscription<std_msgs::msg::String>("robot_station", 10, std::bind(&MySubNode::sub_station, this, std::placeholders::_1));
    }

private:
    // Add your private methods and members here
    void sub_station(std_msgs::msg::String::SharedPtr message)
    {
        RCLCPP_INFO(this->get_logger(), "Getting data from PUblisher, %s", message->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ ;
};


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<MySubNode>();
    // RCLCPP_INFO(node3->get_logger(), "Hello World! from node2");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}

