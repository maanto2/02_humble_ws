#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class NumberPub : public rclcpp::Node
{
public:
    NumberPub() : Node("number_publisher")
    
    {
        pub_ = this->create_publisher<std_msgs::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1) , std::bind(&NumberPub::publish_station, this));
    }

private:
    // Add your private methods and members here
    void publish_station()
    {
        srand((unsigned) time(NULL));
        int random = 100 + (rand() % 101);

        auto message = std_msgs::msg::Int64();
        message.data = random;
        RCLCPP_INFO(this->get_logger(), "Hello World! from timer callback %d", counter);
        counter++;
        pub_->publish(message);
    }
    int counter = 0;
    int random_number = 0;
    rclcpp::TimerBase::SharedPtr timer_ ;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_ ;
};


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<NumberPub>();
    // RCLCPP_INFO(node3->get_logger(), "Hello World! from node2");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}

