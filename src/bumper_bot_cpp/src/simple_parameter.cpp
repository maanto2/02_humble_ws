#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <rcl_interfaces/msg/set_parameters_result.hpp>


class SimpleParameter : public rclcpp::Node
{
    public:
        SimpleParameter() : Node("simple_parameter")
        {
            // Declare parameters with default values
            this->declare_parameter<int>("int_param", 20);
            this->declare_parameter<std::string>("string_param", "Hello_World");

            param_callback_handle_ =  add_on_set_parameters_callback(
                std::bind(&SimpleParameter::on_parameter_change, this, std::placeholders::_1));

        }
    private:

        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> & parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            for (const auto & parameter : parameters)
            {
                if(parameter.get_name()  == "int_param" && parameter.get_type() ==rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    RCLCPP_INFO(this->get_logger(), "int_param changed to %ld", parameter.as_int());
                    result.successful = true;
                }

                if(parameter.get_name() == "string_param" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
                {
                    RCLCPP_INFO(this->get_logger(), "string_param changed to %s", parameter.as_string().c_str());
                    result.successful = true;
                }
            }
            return result;
        }


};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleParameter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}