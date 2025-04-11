#include "rclcpp/rclcpp.hpp"



class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_first_node")
    
    {
        
        RCLCPP_INFO(this->get_logger(), "Hello World! using this-> instead of MyNode->");
        timer_  = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyNode::timer_callback, this));
    }

private:
    // Add your private methods and members here
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello World! from timer callback %d", counter);
        counter++;
    }
    rclcpp::TimerBase::SharedPtr timer_ ;
    int counter = 0;
};


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    auto node1  = rclcpp::Node::make_shared("my_first_node");
    auto node2 = std::make_shared<rclcpp::Node>("my_second_node");

    RCLCPP_INFO(node1->get_logger(), "Hello World! from node1");
    RCLCPP_INFO(node2->get_logger(), "Hello World! from node2");

    auto node3 = std::make_shared<MyNode>();
    // RCLCPP_INFO(node3->get_logger(), "Hello World! from node2");

    rclcpp::spin(node3);
    rclcpp::shutdown();
    return 0;

}


