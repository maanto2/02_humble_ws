import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter


class SimpleParameter(Node):
    """
    A simple parameter class that allows you to set and get parameters.
    """

    def __init__(self):
        super().__init__('simple_parameter')


        self.declare_parameter('int_param', 20)
        self.declare_parameter('string_param', 'hello world')

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info('Simple parameter node started.')


    def parameter_callback(self, params):
        changed =SetParametersResult()

        for param in params:
            if param.name == 'int_param' and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f'Changed int_param to {param.value}')
                changed.successful = True

            if param.name == 'string_param' and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f'Changed int_param to {param.value}')
                changed.successful = True
        
        return changed


if __name__ == '__main__':
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()

            