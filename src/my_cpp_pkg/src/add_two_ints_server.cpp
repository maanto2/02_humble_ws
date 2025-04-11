#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsServer : public rclcpp::Node
{
public:
    AddTwoIntsServer() : Node("add_two_ints_server")
    {
        // Create a service server
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints", std::bind(&AddTwoIntsServer::add, this, std::placeholders::_1, std::placeholders::_2));
        
    }

private:
    // Add your private methods and members here
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_ ;
    
    void add(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
              example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        // Perform the addition
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request: a=%ld b=%ld", request->a, request->b);
        RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", response->sum);
    }
};


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<AddTwoIntsServer>();
    // RCLCPP_INFO(node3->get_logger(), "Hello World! from node2");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}