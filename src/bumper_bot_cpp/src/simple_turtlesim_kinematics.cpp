#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <turtlesim/msg/pose.hpp>



class SimpleTurtleSimKinematics: public rclcpp::Node
{
    public:
        SimpleTurtleSimKinematics() : Node("simple_turtlesim_kinematics")
        {

            t1_pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
                "/turtle1/pose", 10, std::bind(&SimpleTurtleSimKinematics::t1_pose_callback, this, std::placeholders::_1));

            t2_pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
                "/turtle2/pose", 10, std::bind(&SimpleTurtleSimKinematics::t2_pose_callback, this, std::placeholders::_1));
            

        }
    private:

        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr t1_pose_subscriber_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr t2_pose_subscriber_;

        float t1_x_, t1_y_;
        float t2_x_, t2_y_;
        float t_vx_, t_vy_;
        float t1_theta_, t2_theta_;
        float rad_theta_, deg_theta_;


        void t1_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
        {
            t1_x_ = msg->x;
            t1_y_ = msg->y;
            t1_theta_ = msg->theta;

        }
        void t2_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
        {
           t2_x_ = msg->x;
           t2_y_ = msg->y;
           t2_theta_ = msg->theta;

              t_vx_ = t2_x_ - t1_x_;
              t_vy_ = t2_y_ - t1_y_;

              rad_theta_ = t1_theta_ - t2_theta_;
              deg_theta_ = rad_theta_ * (180.0 / 3.14159265358979323846);


                RCLCPP_INFO(this->get_logger(), "Translation from T2 to T1: (%.2f, %.2f)", t_vx_, t_vy_);
                RCLCPP_INFO_STREAM(this->get_logger(), 
                                "Rotation MArtix from T1 -T2:\n"<<
                                 "|R11       R12|:   |"<< std::cos(rad_theta_) <<"\t"<< -std::sin(rad_theta_) <<" | \n"<<
                                 "|R21       R22|:   |"<< std::sin(rad_theta_) <<"\t"<< std::cos(rad_theta_) <<" | \n");
        }


};

//main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTurtleSimKinematics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}