#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClient : public rclcpp::Node
{
public:
    AddTwoIntsClient() : Node("add_two_ints_client")
    
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }


    void send_request(const  int a, const int b)
        {
            // Wait for the service to be available
            while (!client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_INFO(this->get_logger(), "Waiting for service");
            }


            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = a; 
            request->b = b;
            client_->async_send_request(request, std::bind(&AddTwoIntsClient::call_back, this, std::placeholders::_1));
        }
private:
    void call_back(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "SUm %d", (int) response->sum);
    }
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

};


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<AddTwoIntsClient>();
    int a = std::stoi(argv[1]);
    int b = std::stoi(argv[2]);
    node->send_request(a, b);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
