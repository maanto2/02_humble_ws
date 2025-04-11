#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyPubNode : public rclcpp::Node
{
public:
    MyPubNode() : Node("robot_station_pub")
    
    {
        pub_ = this->create_publisher<std_msgs::msg::String>("robot_station", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1) , std::bind(&MyPubNode::publish_station, this));
    }

private:
    // Add your private methods and members here
    void publish_station()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello World! from robot_station_pub";
        RCLCPP_INFO(this->get_logger(), "Hello World! from timer callback %d", counter);
        counter++;
        pub_->publish(message);
    }
    int counter = 0;
    rclcpp::TimerBase::SharedPtr timer_ ;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ ;
};


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<MyPubNode>();
    // RCLCPP_INFO(node3->get_logger(), "Hello World! from node2");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}

